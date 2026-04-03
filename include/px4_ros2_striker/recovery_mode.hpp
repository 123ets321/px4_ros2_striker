#pragma once

/**
 * @file recovery_mode.hpp
 * @brief Recovery waypoint flight mode — registered with PX4 via px4_ros2_cpp
 *
 * API target: px4-ros2-interface-lib release/1.16
 *
 * ─────────────────────────────────────────────────────────────────────────────
 *  WHY trajectory_setpoint AND goto_setpoint BOTH FAIL ON FIXED-WING
 * ─────────────────────────────────────────────────────────────────────────────
 *
 *  PX4's FixedwingPositionControl reads from position_setpoint_triplet (set by
 *  the navigator).  It does NOT subscribe to trajectory_setpoint (MC path) or
 *  goto_setpoint (MC goto mode).  Publishing either topic in an external mode
 *  produces no attitude output — the FW controller keeps executing its last
 *  commanded attitude and the aircraft flies straight through the waypoint and
 *  beyond.  This is why distance increases at exactly the strike Vc in the log.
 *
 *  The ONLY setpoint type that reliably drives fixed-wing through px4_ros2_cpp
 *  is AttitudeSetpointType — as proved by StrikerMode working correctly.
 *
 * ─────────────────────────────────────────────────────────────────────────────
 *  SOLUTION — attitude-based waypoint guidance
 * ─────────────────────────────────────────────────────────────────────────────
 *
 *  We implement a simple bank-to-turn + FPA controller that produces attitude
 *  commands using the same AttitudeSetpointType as StrikerMode:
 *
 *  Lateral (heading):
 *    desired_yaw   = atan2(err_E, err_N)          bearing to waypoint
 *    heading_err   = wrap_pi(desired_yaw - vel_yaw)
 *    lat_accel     = KP_HDG * heading_err * V      L1-like lateral demand
 *    roll_cmd      = atan2(lat_accel, g)            saturated at MAX_ROLL
 *
 *  Longitudinal (altitude):
 *    los_el        = atan2(-err_D, dist_horiz)     positive = target above
 *    fpa           = atan2(-vel_D, vel_horiz)       positive = climbing
 *    pitch_cmd     = KP_PITCH * (los_el - fpa)     saturated at MAX_PITCH
 *
 *  Yaw setpoint  = velocity heading (FW bank-to-turn; no sideslip command)
 *  Thrust        = RECOVERY_THRUST (fixed cruise throttle)
 *
 *  Arrival radius = 150 m (hard floor) — FW physically cannot close within
 *  the NAV_LOITER_RAD (120 m) circle, so < 150 m would never trigger arrival.
 */

#include <rclcpp/rclcpp.hpp>
#include <rclcpp_action/rclcpp_action.hpp>
#include <px4_ros2/components/mode.hpp>
#include <px4_ros2/control/setpoint_types/experimental/attitude.hpp>
#include <px4_ros2/odometry/local_position.hpp>
#include <px4_ros2/odometry/attitude.hpp>
#include <px4_ros2/odometry/airspeed.hpp>
#include <px4_msgs/msg/vehicle_command.hpp>
#include <px4_ros2_striker/action/recover.hpp>
#include <Eigen/Dense>
#include <Eigen/Geometry>
#include <cmath>
#include <memory>
#include <mutex>
#include <string>

namespace apn_fw {

// ── Guidance tuning ───────────────────────────────────────────────────────────
static constexpr double REC_KP_HDG        = 1.8;              // heading → lat-accel gain
static constexpr double REC_MAX_LAT_ACCEL = 1.5 * 9.81;      // [m/s²]  ~1.5 g lateral
static constexpr double REC_MAX_ROLL_RAD  = 45.0*M_PI/180.0;  // [rad]
static constexpr double REC_MAX_PITCH_RAD = 25.0*M_PI/180.0;  // [rad]
static constexpr double REC_KP_PITCH      = 1.5;
static constexpr double REC_THRUST_CRUISE = 0.70;             // normal cruise throttle
static constexpr double REC_THRUST_LO     = 0.45;             // reduced throttle when overspeed
static constexpr double REC_OVERSPEED_MS  = 30.0;             // [m/s] threshold — reduce throttle above this
static constexpr double REC_AIRSPEED_MIN  = 5.0;              // [m/s] fallback if sensor invalid
static constexpr double REC_G             = 9.81;

// ── Arrival / feedback ────────────────────────────────────────────────────────
// NOTE: MIN_FW_ARRIVAL_R is NOT constrained by NAV_LOITER_RAD here — we use
// our own arrival check with attitude control, not PX4's navigator.
// We check HORIZONTAL distance only, so altitude error doesn’t block arrival.
// At 29 m/s / 45° bank, turn radius ≈ 86m.  Set floor to 150m so the
// proportional heading controller has room to straighten before the trigger.
static constexpr double DEFAULT_ARRIVAL_R = 150.0;   // [m]  horizontal
static constexpr double MIN_FW_ARRIVAL_R  = 150.0;   // [m]  horizontal floor
static constexpr int    RECOVER_FB_TICKS  = 25;      // throttle: feedback every ~0.5 s

// ── Final mode constants (for DO_SET_MODE after arrival) ─────────────────────
static constexpr float REC_PX4_CUSTOM     = 1.0f;  // base_mode = custom
static constexpr float REC_PX4_MAIN_AUTO  = 4.0f;  // custom_main = AUTO
static constexpr float REC_PX4_SUB_LOITER = 3.0f;  // AUTO.LOITER (Hold)
static constexpr float REC_PX4_SUB_RTL    = 5.0f;  // AUTO.RTL

using RecoverAction = px4_ros2_striker::action::Recover;
using RecoverHandle = rclcpp_action::ServerGoalHandle<RecoverAction>;

// ─────────────────────────────────────────────────────────────────────────────
static inline double wrap_pi(double a)
{
    while (a >  M_PI) a -= 2.0 * M_PI;
    while (a < -M_PI) a += 2.0 * M_PI;
    return a;
}

// ═════════════════════════════════════════════════════════════════════════════
class RecoveryMode : public px4_ros2::ModeBase
{
public:
    explicit RecoveryMode(rclcpp::Node& node)
    : px4_ros2::ModeBase(node, Settings{"APN Recovery", true})
    , _node(node)
    {
        // AttitudeSetpointType — the only setpoint path that drives FW via
        // px4_ros2_cpp external mode (same as StrikerMode).
        _attitude_sp = std::make_shared<px4_ros2::AttitudeSetpointType>(*this);
        _local_pos   = std::make_shared<px4_ros2::OdometryLocalPosition>(*this);
        _attitude    = std::make_shared<px4_ros2::OdometryAttitude>(*this);
        _airspeed    = std::make_shared<px4_ros2::OdometryAirspeed>(*this);

        // Publisher for DO_SET_MODE command issued on arrival.
        auto cmd_qos = rclcpp::QoS(rclcpp::KeepLast(1))
                           .best_effort().durability_volatile();
        _cmd_pub = node.create_publisher<px4_msgs::msg::VehicleCommand>(
            "/fmu/in/vehicle_command", cmd_qos);

        setSkipMessageCompatibilityCheck();
        RCLCPP_INFO(node.get_logger(),
            "[RecoveryMode] Constructed. Call doRegister() to activate with PX4.");
    }

    // ── Lifecycle ─────────────────────────────────────────────────────────────
    void onActivate() override
    {
        _fb_tick   = 0;
        _arrived   = false;
        _cancelled = false;

        const bool has_goal = hasActiveGoal();

        if (!has_goal) {
            if (!_available) {
                RCLCPP_WARN(_node.get_logger(),
                    "[RecoveryMode] Activated from QGC but NOT available "
                    "(StrikerMode not active). Will yield immediately.");
                return;
            }
            // Strike active, no explicit goal → fly to home.
            _waypoint_ned   = _default_waypoint;
            _arrival_radius = DEFAULT_ARRIVAL_R;
            RCLCPP_INFO(_node.get_logger(),
                "[RecoveryMode] QGC-direct (no goal) → default home "
                "[%.1f %.1f %.1f] NED r=%.0fm",
                _default_waypoint.x(),
                _default_waypoint.y(),
                _default_waypoint.z(),
                _arrival_radius);
        } else {
            RCLCPP_INFO(_node.get_logger(),
                "[RecoveryMode] ROS2 goal → WP=[%.1f %.1f %.1f] r=%.1fm",
                _waypoint_ned.x(), _waypoint_ned.y(), _waypoint_ned.z(),
                _arrival_radius);
        }
    }

    void onDeactivate() override
    {
        std::lock_guard<std::mutex> lk(_goal_mutex);
        _goal_ready = false;

        if (!_active_goal) return;

        // If arrival was successful, updateSetpoint() already closed the goal.
        // We only need to abort if the mode was deactivated for another reason.
        if (_active_goal->is_active()) {
            auto result = std::make_shared<RecoverAction::Result>();
            result->arrived     = false;
            result->termination = _cancelled ? "cancelled" : "aborted";
            _active_goal->abort(result);
            RCLCPP_WARN(_node.get_logger(),
                "[RecoveryMode] Deactivated \u2014 termination=%s",
                result->termination.c_str());
        }

        _active_goal.reset();
        _arrived   = false;
        _cancelled = false;
    }

    // ── Core control loop (called at ~50 Hz) ──────────────────────────────────
    void updateSetpoint(float /*dt_s*/) override
    {
        // ── GUARD: not available + no goal → yield immediately ────────────────
        if (!_available && !hasActiveGoal()) {
            RCLCPP_WARN_THROTTLE(_node.get_logger(), *_node.get_clock(), 2000,
                "[RecoveryMode] Not available & no goal — yielding.");
            completed(px4_ros2::Result::Interrupted);
            return;
        }

        // ── Cancellation check ────────────────────────────────────────────────
        {
            std::lock_guard<std::mutex> lk(_goal_mutex);
            if (_active_goal && _active_goal->is_canceling()) {
                _cancelled = true;
                _arrived   = false;
                completed(px4_ros2::Result::Success);
                return;
            }
        }

        // ── Geometry ──────────────────────────────────────────────────────────
        const Eigen::Vector3d pos       = current_pos_ned();
        const Eigen::Vector3d vel       = current_vel_ned();
        const Eigen::Vector3d err       = _waypoint_ned - pos;     // NED error vector
        const double dist_horiz         = std::sqrt(err(0)*err(0) + err(1)*err(1));
        const double dist               = err.norm();               // 3D (for guidance)

        // Arrival: use HORIZONTAL distance only.
        if (dist_horiz < _arrival_radius) {
            if (!_arrived) {
                _arrived = true;
                RCLCPP_INFO(_node.get_logger(),
                    "[RecoveryMode] \u2714 Horizontal arrival (%.1fm < %.1fm) — requesting %s.",
                    dist_horiz, _arrival_radius, _final_mode.c_str());
                
                // Immediately tell PX4 to change modes (Hold or RTL)
                publish_final_mode_cmd();

                // Tell the external mode controller we are done
                completed(px4_ros2::Result::Success);

                // Unblock the python script immediately instead of waiting for onDeactivate
                std::lock_guard<std::mutex> lk(_goal_mutex);
                if (_active_goal && _active_goal->is_active()) {
                    auto result = std::make_shared<RecoverAction::Result>();
                    result->arrived     = true;
                    result->termination = "arrived";
                    _active_goal->succeed(result);
                }
            }

            // Command level cruise flight (current yaw, 0 roll, 0 pitch) 
            // while we sit here waiting for PX4 to actually execute the mode switch.
            // If we just 'return' here, we starve the controller.
            const double yaw = std::atan2(vel(1), vel(0));
            Eigen::Quaternionf q_lvl_f(
                Eigen::AngleAxisf(static_cast<float>(yaw), Eigen::Vector3f::UnitZ()) *
                Eigen::AngleAxisf(0.0f,                    Eigen::Vector3f::UnitY()) *
                Eigen::AngleAxisf(0.0f,                    Eigen::Vector3f::UnitX())
            );

            const Eigen::Vector3f thrust_sp{static_cast<float>(REC_THRUST_CRUISE), 0.f, 0.f};
            _attitude_sp->update(q_lvl_f, thrust_sp);
            return;
        }

        // ── Lateral guidance (bank-to-turn) ───────────────────────────────────
        // Bearing to waypoint in NED (atan2(East, North))
        const double desired_yaw  = std::atan2(err(1), err(0));
        const double vel_horiz    = std::max(
            std::sqrt(vel(0)*vel(0) + vel(1)*vel(1)), 1.0);
        const double current_yaw  = std::atan2(vel(1), vel(0));
        const double hdg_err      = wrap_pi(desired_yaw - current_yaw);

        // L1-style lateral acceleration demand → roll
        const double V            = std::max(vel.norm(), 1.0);
        const double lat_accel    = std::clamp(
            REC_KP_HDG * hdg_err * V, -REC_MAX_LAT_ACCEL, REC_MAX_LAT_ACCEL);
        const double roll         = std::clamp(
            std::atan2(lat_accel, REC_G), -REC_MAX_ROLL_RAD, REC_MAX_ROLL_RAD);

        // ── Longitudinal guidance (FPA control) ───────────────────────────────
        // NED: err(2) < 0 means target is above current position → need to climb
        const double dist_horiz_clamped = std::max(dist_horiz, 0.1);
        // los_el > 0 → target above; vel(2) < 0 in NED when climbing
        const double los_el       = std::atan2(-err(2), dist_horiz_clamped);
        const double fpa          = std::atan2(-vel(2), vel_horiz);
        const double pitch        = std::clamp(
            REC_KP_PITCH * (los_el - fpa), -REC_MAX_PITCH_RAD, REC_MAX_PITCH_RAD);

        // ── Attitude quaternion (ZYX: yaw=velocity heading, pitch, roll) ───────
        // Yaw = velocity heading so the FW tracks via bank-to-turn, not sideslip.
        const double yaw = current_yaw;

        const Eigen::Quaternionf q =
            (Eigen::AngleAxisf(static_cast<float>(yaw),   Eigen::Vector3f::UnitZ()) *
             Eigen::AngleAxisf(static_cast<float>(pitch),  Eigen::Vector3f::UnitY()) *
             Eigen::AngleAxisf(static_cast<float>(roll),   Eigen::Vector3f::UnitX()))
            .cast<float>();

        // ── Adaptive thrust ───────────────────────────────────────────────────
        // Post-dive airspeed can exceed 60+ m/s.  Running cruise throttle (0.70)
        // at that speed fights the pitch-up and extends altitude loss.  Drop to
        // a lower throttle until airspeed bleeds back to cruise range.
        const float spd_ms   = _airspeed->trueAirspeed();
        const bool  valid_as = (spd_ms > static_cast<float>(REC_AIRSPEED_MIN));
        const float thrust_cmd = (valid_as && spd_ms > static_cast<float>(REC_OVERSPEED_MS))
            ? static_cast<float>(REC_THRUST_LO)
            : static_cast<float>(REC_THRUST_CRUISE);

        const Eigen::Vector3f thrust_sp{thrust_cmd, 0.f, 0.f};
        _attitude_sp->update(q, thrust_sp);

        // ── Feedback (throttled) ───────────────────────────────────────────────
        if (++_fb_tick >= RECOVER_FB_TICKS) {
            _fb_tick = 0;
            publish_feedback(err, dist, hdg_err, roll, pitch, los_el, fpa, thrust_cmd);
        }
    }

    // ── Public API ────────────────────────────────────────────────────────────

    void setAvailable(bool available)
    {
        _available = available;
        RCLCPP_INFO(_node.get_logger(),
            "[RecoveryMode] Availability → %s",
            available ? "ENABLED (strike active)" : "DISABLED (no strike)");
    }

    void setDefaultWaypoint(const Eigen::Vector3d& ned)
    {
        _default_waypoint = ned;
        RCLCPP_INFO(_node.get_logger(),
            "[RecoveryMode] Default home WP → [%.1f %.1f %.1f] NED",
            ned.x(), ned.y(), ned.z());
    }

    void setWaypoint(const Eigen::Vector3d& pos_ned,
                     const std::string& final_mode,
                     double arrival_radius)
    {
        _waypoint_ned   = pos_ned;
        _final_mode     = final_mode;
        _arrival_radius = std::max(
            (arrival_radius > 0.0) ? arrival_radius : DEFAULT_ARRIVAL_R,
            MIN_FW_ARRIVAL_R);
        _arrived   = false;
        _cancelled = false;
    }

    void setGoalHandle(std::shared_ptr<RecoverHandle> handle)
    {
        std::lock_guard<std::mutex> lk(_goal_mutex);
        _active_goal = handle;
        _goal_ready  = true;
        _arrived     = false;
        _cancelled   = false;
        _fb_tick     = 0;
    }

    bool hasActiveGoal() const
    {
        std::lock_guard<std::mutex> lk(_goal_mutex);
        return _goal_ready
            && _active_goal != nullptr
            && _active_goal->is_active();
    }

    bool isAvailable() const { return _available; }
    const std::string& finalMode() const { return _final_mode; }

private:
    // ── Odometry helpers ──────────────────────────────────────────────────────
    Eigen::Vector3d current_pos_ned() const
    {
        const auto p = _local_pos->positionNed();
        return {p.x(), p.y(), p.z()};
    }

    Eigen::Vector3d current_vel_ned() const
    {
        const auto v = _local_pos->velocityNed();
        return {v.x(), v.y(), v.z()};
    }

    // ── Final mode dispatch (called from onDeactivate on arrival) ─────────────
    void publish_final_mode_cmd()
    {
        if (!_cmd_pub) return;
        const float sub = (_final_mode == "rtl") ? REC_PX4_SUB_RTL : REC_PX4_SUB_LOITER;
        px4_msgs::msg::VehicleCommand cmd{};
        cmd.timestamp       = _node.get_clock()->now().nanoseconds() / 1000;
        cmd.command         = px4_msgs::msg::VehicleCommand::VEHICLE_CMD_DO_SET_MODE;
        cmd.param1          = REC_PX4_CUSTOM;
        cmd.param2          = REC_PX4_MAIN_AUTO;
        cmd.param3          = sub;
        cmd.target_system   = 1;
        cmd.target_component = 1;
        cmd.source_system   = 1;
        cmd.source_component = 1;
        cmd.from_external   = true;
        _cmd_pub->publish(cmd);
    }

    // ── Feedback ──────────────────────────────────────────────────────────────
    void publish_feedback(const Eigen::Vector3d& err, double dist,
                          double hdg_err, double roll, double pitch,
                          double los_el, double fpa, float thrust_cmd) const
    {
        std::lock_guard<std::mutex> lk(_goal_mutex);
        if (!_active_goal || !_active_goal->is_active()) return;

        const double dist_hz = std::sqrt(err(0)*err(0) + err(1)*err(1));
        const double alt_err = std::abs(err(2));

        auto fb          = std::make_shared<RecoverAction::Feedback>();
        fb->dist_m       = static_cast<float>(dist);
        fb->dist_horiz_m = static_cast<float>(dist_hz);
        fb->alt_err_m    = static_cast<float>(alt_err);
        _active_goal->publish_feedback(fb);

        RCLCPP_INFO(_node.get_logger(),
            "[RECOVERY] dist=%.1fm (hz=%.1f alt=%.1f) | "
            "hdg_err=%.1f° roll=%.1f° pitch=%.1f° "
            "LOS=%.1f° FPA=%.1f° thr=%.2f | wp=[%.1f %.1f %.1f]",
            dist, dist_hz, alt_err,
            hdg_err  * 180.0/M_PI,
            roll     * 180.0/M_PI,
            pitch    * 180.0/M_PI,
            los_el   * 180.0/M_PI,
            fpa      * 180.0/M_PI,
            static_cast<double>(thrust_cmd),
            _waypoint_ned.x(), _waypoint_ned.y(), _waypoint_ned.z());
    }

    // ── Members ───────────────────────────────────────────────────────────────
    rclcpp::Node&                                    _node;
    std::shared_ptr<px4_ros2::AttitudeSetpointType>  _attitude_sp;
    std::shared_ptr<px4_ros2::OdometryLocalPosition> _local_pos;
    std::shared_ptr<px4_ros2::OdometryAttitude>      _attitude;
    std::shared_ptr<px4_ros2::OdometryAirspeed>      _airspeed;
    rclcpp::Publisher<px4_msgs::msg::VehicleCommand>::SharedPtr _cmd_pub;

    Eigen::Vector3d _waypoint_ned{Eigen::Vector3d::Zero()};
    Eigen::Vector3d _default_waypoint{Eigen::Vector3d::Zero()};
    std::string     _final_mode{"hold"};
    double          _arrival_radius{DEFAULT_ARRIVAL_R};

    bool _arrived{false};
    bool _cancelled{false};
    bool _goal_ready{false};
    bool _available{false};

    int _fb_tick{0};

    mutable std::mutex              _goal_mutex;
    std::shared_ptr<RecoverHandle>  _active_goal;
};

}  // namespace apn_fw