#pragma once

/**
 * @file striker_mode.hpp
 * @brief APN guidance flight mode — registered with PX4 via px4_ros2_cpp
 *
 * API target: px4-ros2-interface-lib release/1.16
 *
 * ─────────────────────────────────────────────────────────────────────────────
 *  CHANGES vs previous version
 * ─────────────────────────────────────────────────────────────────────────────
 *
 *  GUARD — No-goal / QGC-direct activation
 *    updateSetpoint() now checks hasActiveGoal() first.  If PX4 or QGC
 *    switches to this mode without an action server goal (e.g. the operator
 *    manually selects "APN Strike" in QGC), completed(Interrupted) is called
 *    immediately so PX4 falls back to its previous mode.
 *    No guidance setpoints are ever published without an explicit ROS2 goal.
 *
 *  ACTIVATION CALLBACK
 *    setOnActivateCallback() lets the owning node learn the instant PX4
 *    actually activates this mode.  The node uses this to enable RecoveryMode
 *    availability in QGC (it should only appear selectable while strike is
 *    active).
 *
 *  _goal_ready flag
 *    Separate from _active_goal — set in setGoalHandle(), cleared only in
 *    onDeactivate().  Lets the guard distinguish "goal is being set up" from
 *    "genuinely no goal" during the brief window between setGoalHandle() and
 *    the first onActivate() tick.
 */

#include <rclcpp/rclcpp.hpp>
#include <rclcpp_action/rclcpp_action.hpp>
#include <px4_ros2/components/mode.hpp>
#include <px4_ros2/control/setpoint_types/experimental/attitude.hpp>
#include <px4_ros2/odometry/attitude.hpp>
#include <px4_ros2/odometry/local_position.hpp>
#include <px4_ros2/odometry/airspeed.hpp>
#include <px4_ros2_striker/action/strike.hpp>
#include <Eigen/Dense>
#include <Eigen/Geometry>
#include <cmath>
#include <functional>
#include <memory>
#include <mutex>
#include <string>

namespace apn_fw {

// ── Tuning ────────────────────────────────────────────────────────────────────
static constexpr double APN_N             = 4.5;
static constexpr double MAX_ROLL_RAD      = 55.0 * M_PI / 180.0;
static constexpr double MAX_PITCH_RAD     = 40.0 * M_PI / 180.0;
static constexpr double APN_THRUST        = 1.0;
static constexpr double HIT_RADIUS        = 5.0;
static constexpr double TGO_LOOKAHEAD_S   = 0.15;
static constexpr double MAX_LAT_ACCEL     = 2.0 * 9.81;
static constexpr double AIRSPEED_MIN      = 5.0;
static constexpr double G                 = 9.81;
static constexpr double KP_PITCH          = 1.5;
static constexpr double VC_DIVERGE_THRESH = -2.0;
static constexpr double ALPHA_TGT_VEL     = 0.25;
static constexpr double ALPHA_TGT_ACCEL   = 0.15;
static constexpr double TGT_VEL_GATE      = 0.5;
static constexpr int    STRIKE_FB_TICKS   = 50;   // ~1 Hz at 50 Hz rate

using StrikeAction = px4_ros2_striker::action::Strike;
using StrikeHandle = rclcpp_action::ServerGoalHandle<StrikeAction>;

inline Eigen::Vector3d lpf3(double a, const Eigen::Vector3d& x,
                             const Eigen::Vector3d& xp)
{ return a * x + (1.0 - a) * xp; }

// ═════════════════════════════════════════════════════════════════════════════
class StrikerMode : public px4_ros2::ModeBase
{
public:
    explicit StrikerMode(rclcpp::Node& node)
    : px4_ros2::ModeBase(node, Settings{"APN Strike", true})
    , _node(node)
    {
        _attitude_sp = std::make_shared<px4_ros2::AttitudeSetpointType>(*this);
        _local_pos   = std::make_shared<px4_ros2::OdometryLocalPosition>(*this);
        _attitude    = std::make_shared<px4_ros2::OdometryAttitude>(*this);
        _airspeed    = std::make_shared<px4_ros2::OdometryAirspeed>(*this);

        setSkipMessageCompatibilityCheck();
        RCLCPP_INFO(node.get_logger(),
            "[StrikerMode] Constructed. Call doRegister() to activate with PX4.");
    }

    // ── Lifecycle ─────────────────────────────────────────────────────────────
    void onActivate() override
    {
        _fb_tick      = 0;
        _cancelled    = false;
        _hit_detected = false;

        const bool has_goal = hasActiveGoal();
        RCLCPP_INFO(_node.get_logger(),
            "[StrikerMode] Activated — goal present: %s",
            has_goal ? "YES" : "NO (QGC direct — will yield immediately)");

        // Notify the owning node so it can enable RecoveryMode in QGC.
        // Only fire if we have a legitimate ROS2 goal — QGC-direct activations
        // will be rejected in the first updateSetpoint() tick anyway.
        if (has_goal && _on_activate_cb) {
            _on_activate_cb();
        }
    }

    void onDeactivate() override
    {
        std::lock_guard<std::mutex> lk(_goal_mutex);
        _goal_ready = false;

        if (!_active_goal) return;

        auto result = std::make_shared<StrikeAction::Result>();

        if (_hit_detected) {
            result->hit             = true;
            result->miss_distance_m = static_cast<float>(_final_range_m);
            result->termination     = "hit";
            _active_goal->succeed(result);
            RCLCPP_INFO(_node.get_logger(),
                "[StrikerMode] HIT — miss_distance=%.2fm", _final_range_m);
        } else {
            result->hit             = false;
            result->miss_distance_m = static_cast<float>(
                (target_pos_ned_ - current_pos()).norm());
            result->termination     = _cancelled ? "cancelled" : "aborted";
            _active_goal->abort(result);
            RCLCPP_WARN(_node.get_logger(),
                "[StrikerMode] Deactivated — termination=%s",
                result->termination.c_str());
        }

        _active_goal.reset();
        _hit_detected = false;
        _cancelled    = false;
    }

    // ── Core control loop ─────────────────────────────────────────────────────
    void updateSetpoint(float dt_s) override
    {
        // ── GUARD: no ROS2 goal → yield back to PX4 immediately ──────────────
        // Fires when QGC/GCS directly selects "APN Strike" without an action
        // goal.  completed(Interrupted) tells PX4 to fall back to its previous
        // mode; no attitude setpoint is ever published in this branch.
        if (!hasActiveGoal()) {
            RCLCPP_WARN_THROTTLE(_node.get_logger(), *_node.get_clock(), 2000,
                "[StrikerMode] Active without goal (QGC direct?) — yielding.");
            completed(px4_ros2::Result::Interrupted);
            return;
        }

        // ── Cancellation check ────────────────────────────────────────────────
        {
            std::lock_guard<std::mutex> lk(_goal_mutex);
            if (_active_goal && _active_goal->is_canceling()) {
                _cancelled    = true;
                _hit_detected = false;
                completed(px4_ros2::Result::Success);
                return;
            }
        }

        // ── Geometry ──────────────────────────────────────────────────────────
        const Eigen::Vector3d pos   = current_pos();
        const Eigen::Vector3d vel   = current_vel();
        const Eigen::Vector3d R_vec = target_pos_ned_ - pos;
        const double          R_mag = std::max(R_vec.norm(), 0.1);
        const Eigen::Vector3d lhat  = R_vec / R_mag;
        const Eigen::Vector3d V_rel = target_vel_ned_ - vel;
        const double          Vc    = -(V_rel.dot(lhat));
        const double          t_go  = (Vc > 1.0) ? (R_mag / Vc) : 999.0;

        // ── Terminal condition ─────────────────────────────────────────────────
        if (R_mag < HIT_RADIUS || t_go < TGO_LOOKAHEAD_S) {
            _hit_detected  = true;
            _final_range_m = R_mag;
            completed(px4_ros2::Result::Success);
            return;
        }

        // ── APN Guidance ───────────────────────────────────────────────────────
        const Eigen::Vector3d a_cmd = compute_apn_accel(pos, vel);
        const auto [roll, pitch, yaw] = ned_accel_to_attitude(a_cmd, R_vec);

        const Eigen::Quaternionf q =
            (Eigen::AngleAxisf(static_cast<float>(yaw),   Eigen::Vector3f::UnitZ()) *
             Eigen::AngleAxisf(static_cast<float>(pitch),  Eigen::Vector3f::UnitY()) *
             Eigen::AngleAxisf(static_cast<float>(roll),   Eigen::Vector3f::UnitX()))
            .cast<float>();

        const Eigen::Vector3f thrust_sp{static_cast<float>(APN_THRUST), 0.f, 0.f};
        _attitude_sp->update(q, thrust_sp);

        // ── Feedback (throttled) ───────────────────────────────────────────────
        if (++_fb_tick >= STRIKE_FB_TICKS) {
            _fb_tick = 0;
            publish_feedback(R_vec, R_mag, Vc, t_go, roll, pitch, dt_s);
        }
    }

    // ── Public API ────────────────────────────────────────────────────────────

    /**
     * Register a callback that fires when PX4 actually activates this mode
     * AND a valid ROS2 action goal is present.  Used by the owning node to
     * enable RecoveryMode availability in QGC.
     */
    void setOnActivateCallback(std::function<void()> cb)
    {
        _on_activate_cb = std::move(cb);
    }

    void setTarget(const Eigen::Vector3d& pos_ned)
    {
        target_pos_ned_     = pos_ned;
        target_vel_ned_     = Eigen::Vector3d::Zero();
        target_accel_ned_   = Eigen::Vector3d::Zero();
        target_pos_prev_    = pos_ned;
        target_vel_prev_    = Eigen::Vector3d::Zero();
        _target_initialized = false;
        _target_time_valid  = false;
        _hit_detected       = false;
        _cancelled          = false;
    }

    void updateTargetPosition(const Eigen::Vector3d& p_new, double dt)
    {
        if (_target_initialized) {
            const Eigen::Vector3d raw_vel = (p_new - target_pos_prev_) / dt;
            target_vel_ned_ = lpf3(ALPHA_TGT_VEL, raw_vel, target_vel_ned_);

            if (target_vel_ned_.norm() > TGT_VEL_GATE) {
                const Eigen::Vector3d raw_acc =
                    (target_vel_ned_ - target_vel_prev_) / dt;
                target_accel_ned_ = lpf3(ALPHA_TGT_ACCEL, raw_acc, target_accel_ned_);
            } else {
                target_accel_ned_ = lpf3(ALPHA_TGT_ACCEL,
                                         Eigen::Vector3d::Zero(), target_accel_ned_);
            }
            target_vel_prev_ = target_vel_ned_;
        } else {
            _target_initialized = true;
        }
        target_pos_prev_ = p_new;
        target_pos_ned_  = p_new;
    }

    void setGoalHandle(std::shared_ptr<StrikeHandle> handle)
    {
        std::lock_guard<std::mutex> lk(_goal_mutex);
        _active_goal  = handle;
        _goal_ready   = true;
        _hit_detected = false;
        _cancelled    = false;
        _fb_tick      = 0;
    }

    /**
     * Returns true only when an action goal is present AND is_active().
     * Guards against QGC-direct activation and goal teardown races.
     */
    bool hasActiveGoal() const
    {
        std::lock_guard<std::mutex> lk(_goal_mutex);
        return _goal_ready
            && _active_goal != nullptr
            && _active_goal->is_active();
    }

private:
    Eigen::Vector3d current_pos() const
    {
        const auto p = _local_pos->positionNed();
        return {p.x(), p.y(), p.z()};
    }

    Eigen::Vector3d current_vel() const
    {
        const auto v = _local_pos->velocityNed();
        return {v.x(), v.y(), v.z()};
    }

    Eigen::Quaterniond current_attitude() const
    {
        const auto q = _attitude->attitude();
        return {q.w(), q.x(), q.y(), q.z()};
    }

    double current_airspeed() const
    {
        const float tas = _airspeed->trueAirspeed();
        return (tas > static_cast<float>(AIRSPEED_MIN)) ? tas : 15.0;
    }

    Eigen::Vector3d compute_apn_accel(const Eigen::Vector3d& pos,
                                      const Eigen::Vector3d& vel) const
    {
        const Eigen::Vector3d R_vec = target_pos_ned_ - pos;
        double R_mag = R_vec.norm();
        if (R_mag < 0.1) R_mag = 0.1;

        const Eigen::Vector3d lhat = R_vec / R_mag;
        double V_mag = vel.norm();
        if (V_mag < 1.0) V_mag = 1.0;
        const Eigen::Vector3d v_hat = vel / V_mag;

        const Eigen::Vector3d V_rel = target_vel_ned_ - vel;
        const double          Vc    = -(V_rel.dot(lhat));

        const double he = std::acos(std::clamp(v_hat.dot(lhat), -1.0, 1.0));
        if (Vc < VC_DIVERGE_THRESH && he > M_PI / 2.0) {
            const Eigen::Vector3d pull_raw = lhat - lhat.dot(v_hat) * v_hat;
            const double pn = pull_raw.norm();
            if (pn > 1e-6) return (pull_raw / pn) * MAX_LAT_ACCEL;
            return Eigen::Vector3d::Zero();
        }

        const Eigen::Vector3d Omega = R_vec.cross(V_rel) / (R_mag * R_mag);
        const Eigen::Vector3d a_PN  = Omega.cross(v_hat) * (APN_N * V_mag);

        const Eigen::Vector3d a_tgt_perp =
            target_accel_ned_ - target_accel_ned_.dot(v_hat) * v_hat;
        const Eigen::Vector3d a_aug = a_tgt_perp * (APN_N / 2.0);

        Eigen::Vector3d a_cmd = a_PN + a_aug;
        if (a_cmd.norm() > MAX_LAT_ACCEL) {
            a_cmd = a_cmd.normalized() * MAX_LAT_ACCEL;
        }
        return a_cmd;
    }

    struct RPY { double roll, pitch, yaw; };

    RPY ned_accel_to_attitude(const Eigen::Vector3d& a_cmd,
                              const Eigen::Vector3d& R_vec) const
    {
        const Eigen::Vector3d a_horiz{a_cmd(0), a_cmd(1), 0.0};
        const Eigen::Matrix3d Rbody =
            current_attitude().normalized().toRotationMatrix().transpose();
        const double roll = std::clamp(
            std::atan2((Rbody * a_horiz)(1), G),
            -MAX_ROLL_RAD, MAX_ROLL_RAD);

        const Eigen::Vector3d vel = current_vel();
        const double R_hz = std::max(
            std::sqrt(R_vec(0)*R_vec(0) + R_vec(1)*R_vec(1)), 0.1);
        const double los_el  = std::atan2(R_vec(2), R_hz);
        const double v_horiz = std::max(
            std::sqrt(vel(0)*vel(0) + vel(1)*vel(1)), 1.0);
        const double fpa   = std::atan2(vel(2), v_horiz);
        const double pitch = std::clamp(
            -KP_PITCH * (los_el - fpa), -MAX_PITCH_RAD, MAX_PITCH_RAD);

        const double v_gnd = std::sqrt(vel(0)*vel(0) + vel(1)*vel(1));
        const double yaw   = (v_gnd > 1.0)
            ? std::atan2(vel(1), vel(0))
            : std::atan2(R_vec(1), R_vec(0));

        return {roll, pitch, yaw};
    }

    void publish_feedback(const Eigen::Vector3d& R_vec, double R_mag,
                          double Vc, double t_go,
                          double roll, double pitch, float /*dt*/) const
    {
        std::lock_guard<std::mutex> lk(_goal_mutex);
        if (!_active_goal || !_active_goal->is_active()) return;

        const Eigen::Vector3d lhat  = R_vec / R_mag;
        const Eigen::Vector3d V_rel = target_vel_ned_ - current_vel();
        const double Vtan = (V_rel - lhat * V_rel.dot(lhat)).norm();
        const double R_hz = std::max(
            std::sqrt(R_vec(0)*R_vec(0) + R_vec(1)*R_vec(1)), 0.1);
        const Eigen::Vector3d vel   = current_vel();
        const double v_horiz = std::max(
            std::sqrt(vel(0)*vel(0) + vel(1)*vel(1)), 1.0);

        auto fb        = std::make_shared<StrikeAction::Feedback>();
        fb->range_m    = static_cast<float>(R_mag);
        fb->vc_ms      = static_cast<float>(Vc);
        fb->vtan_ms    = static_cast<float>(Vtan);
        fb->los_el_deg = static_cast<float>(
            std::atan2(R_vec(2), R_hz) * 180.0 / M_PI);
        fb->fpa_deg    = static_cast<float>(
            std::atan2(vel(2), v_horiz) * 180.0 / M_PI);
        fb->roll_deg   = static_cast<float>(roll  * 180.0 / M_PI);
        fb->pitch_deg  = static_cast<float>(pitch * 180.0 / M_PI);
        fb->ias_ms     = static_cast<float>(current_airspeed());
        fb->t_go_s     = static_cast<float>(t_go);
        _active_goal->publish_feedback(fb);

        RCLCPP_INFO(_node.get_logger(),
            "[STRIKE] r=%.1fm tgo=%.2fs | Vc=%.1f Vtan=%.1f m/s | "
            "LOS=%.1f deg FPA=%.1f deg | roll=%.1f deg pitch=%.1f deg",
            R_mag, t_go, Vc, Vtan,
            fb->los_el_deg, fb->fpa_deg,
            roll * 180.0/M_PI, pitch * 180.0/M_PI);
    }

    // ── Members ───────────────────────────────────────────────────────────────
    rclcpp::Node&                                   _node;
    std::shared_ptr<px4_ros2::AttitudeSetpointType>  _attitude_sp;
    std::shared_ptr<px4_ros2::OdometryLocalPosition> _local_pos;
    std::shared_ptr<px4_ros2::OdometryAttitude>      _attitude;
    std::shared_ptr<px4_ros2::OdometryAirspeed>      _airspeed;

    Eigen::Vector3d target_pos_ned_{Eigen::Vector3d::Zero()};
    Eigen::Vector3d target_vel_ned_{Eigen::Vector3d::Zero()};
    Eigen::Vector3d target_accel_ned_{Eigen::Vector3d::Zero()};
    Eigen::Vector3d target_pos_prev_{Eigen::Vector3d::Zero()};
    Eigen::Vector3d target_vel_prev_{Eigen::Vector3d::Zero()};
    bool            _target_initialized{false};
    bool            _target_time_valid{false};

    bool   _hit_detected{false};
    bool   _cancelled{false};
    bool   _goal_ready{false};   // latches from setGoalHandle() until onDeactivate()
    double _final_range_m{0.0};

    std::function<void()>         _on_activate_cb;  // fires on valid activation
    mutable std::mutex            _goal_mutex;
    std::shared_ptr<StrikeHandle> _active_goal;
    int                           _fb_tick{0};
};

}  // namespace apn_fw