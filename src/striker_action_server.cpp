/**
 * @file apn_fw_action_server.cpp
 * @brief Fixed-Wing APN Guidance Striker — ROS2 Action Server
 *
 * ─────────────────────────────────────────────────────────────────────────────
 *  ARCHITECTURE
 * ─────────────────────────────────────────────────────────────────────────────
 *
 *  Two independent action servers:
 *
 *    /strike_action   (px4_ros2_striker/action/Strike)
 *    /recover_action  (px4_ros2_striker/action/Recover)
 *
 *  Execution model: TIMER-DRIVEN
 *    - goal_cb()    validates and accepts/rejects the incoming goal, stores the
 *                   GoalHandle, updates state machine. Returns immediately.
 *    - cancel_cb()  signals acceptance of cancel request. Returns immediately.
 *    - 50 Hz timer  does ALL work: guidance, setpoint publishing, feedback,
 *                   and calls succeed()/abort()/canceled() on the handle.
 *
 *  This avoids threading hazards — no mutex needed on guidance state because
 *  the timer callback and all ROS topic callbacks share the same executor thread
 *  (SingleThreadedExecutor with reentrant callback groups for the action servers).
 *
 * ─────────────────────────────────────────────────────────────────────────────
 *  STATE MACHINE
 * ─────────────────────────────────────────────────────────────────────────────
 *
 *   IDLE
 *     strike goal accepted  ──────────────────────────────► STRIKE
 *     recover goal accepted ──────────────────────────────► RECOVERY
 *
 *   STRIKE
 *     R < HIT_RADIUS || t_go < TGO_LOOKAHEAD_S ──────────► TERMINAL
 *     recover goal accepted (preempts strike) ────────────► RECOVERY
 *     strike cancelled / aborted  ────────────────────────► IDLE
 *     new strike goal accepted (preempts old) ────────────► STRIKE (new goal)
 *
 *   TERMINAL  (absorbing — vehicle crashed, timer cancelled)
 *     all new goals rejected
 *
 *   RECOVERY
 *     dist < arrival_radius ──────────────────────────────► RECOVERED
 *     recover cancelled / aborted ────────────────────────► IDLE
 *     new strike goal accepted (preempts recovery) ────────► STRIKE
 *
 *   RECOVERED
 *     HOLD/RTL commanded, node stays alive pumping OCM heartbeats.
 *     new strike goal accepted ────────────────────────────► STRIKE
 *     new recover goal accepted ───────────────────────────► RECOVERY
 *
 * ─────────────────────────────────────────────────────────────────────────────
 *  GUIDANCE LAW  — True Proportional Navigation (TPN) + Augmented PN
 * ─────────────────────────────────────────────────────────────────────────────
 *
 *  R       = p_target − p_drone
 *  V_rel   = v_target − v_drone
 *  λ̂       = R / |R|
 *  v̂       = v_drone / |v_drone|           flight-path unit vector
 *  Vc      = −(V_rel · λ̂)
 *  Ω       = (R × V_rel) / |R|²
 *
 *  a_PN    = N · |V_drone| · (Ω × v̂)      True PN
 *  a_aug   = (N/2) · [a_tgt − (a_tgt·v̂)v̂] Augmented (target maneuver)
 *  a_cmd   = a_PN + a_aug                  [saturated to 2g]
 *
 *  Attitude (split-channel):
 *      roll  = atan2(a_body_horiz_y, G)    horizontal PN only, fixed G denom
 *      pitch = −Kp·(los_el − fpa)          LOS elevation tracking, inertial FPA
 *      yaw   = atan2(vN_E, vN_N)           flight-path yaw
 *
 * ─────────────────────────────────────────────────────────────────────────────
 *  Topics
 * ─────────────────────────────────────────────────────────────────────────────
 *  SUB  /fmu/out/vehicle_local_position     px4_msgs/VehicleLocalPosition
 *  SUB  /fmu/out/vehicle_status_v1          px4_msgs/VehicleStatus
 *  SUB  /fmu/out/vehicle_attitude           px4_msgs/VehicleAttitude
 *  SUB  /fmu/out/airspeed_validated         px4_msgs/AirspeedValidated
 *  SUB  /fmu/out/vehicle_command_ack        px4_msgs/VehicleCommandAck
 *  PUB  /fmu/in/offboard_control_mode       px4_msgs/OffboardControlMode
 *  PUB  /fmu/in/vehicle_attitude_setpoint   px4_msgs/VehicleAttitudeSetpoint
 *  PUB  /fmu/in/trajectory_setpoint         px4_msgs/TrajectorySetpoint
 *  PUB  /fmu/in/vehicle_command             px4_msgs/VehicleCommand
 */

#include <rclcpp/rclcpp.hpp>
#include <rclcpp_action/rclcpp_action.hpp>
#include <px4_msgs/msg/vehicle_local_position.hpp>
#include <px4_msgs/msg/vehicle_status.hpp>
#include <px4_msgs/msg/vehicle_attitude.hpp>
#include <px4_msgs/msg/airspeed_validated.hpp>
#include <px4_msgs/msg/offboard_control_mode.hpp>
#include <px4_msgs/msg/vehicle_attitude_setpoint.hpp>
#include <px4_msgs/msg/trajectory_setpoint.hpp>
#include <px4_msgs/msg/vehicle_command.hpp>
#include <px4_msgs/msg/vehicle_command_ack.hpp>
#include <px4_ros2_striker/action/strike.hpp>
#include <px4_ros2_striker/action/recover.hpp>
#include <Eigen/Dense>
#include <Eigen/Geometry>
#include <cmath>
#include <limits>
#include <string>
#include <memory>
#include <mutex>

using namespace std::chrono_literals;

// ═══════════════════════════════════════════════════════════════════════════
//  Tuning Constants
// ═══════════════════════════════════════════════════════════════════════════

static constexpr double APN_N              = 4.5;
static constexpr double MAX_ROLL_RAD       = 60.0 * M_PI / 180.0;
static constexpr double MAX_PITCH_RAD      = 40.0 * M_PI / 180.0;
static constexpr double APN_THRUST         = 1.0;
static constexpr double HIT_RADIUS         = 5.0;    // [m]
static constexpr double TGO_LOOKAHEAD_S    = 0.15;   // [s]
static constexpr double MAX_LAT_ACCEL      = 2.0 * 9.81;
static constexpr double AIRSPEED_MIN       = 5.0;    // [m/s]
static constexpr double AIRSPEED_FALLBACK  = 15.0;   // [m/s]
static constexpr double G                  = 9.81;
static constexpr double R_EARTH            = 6371000.0;
static constexpr double KP_PITCH           = 1.5;
static constexpr double DEFAULT_ARRIVAL_R  = 10.0;   // [m] recovery arrival radius
static constexpr double RECOVERY_MAX_VZ    = 3.0;    // [m/s] gentle altitude change

// IIR LPF
static constexpr double ALPHA_TGT_VEL      = 0.25;
static constexpr double ALPHA_TGT_ACCEL    = 0.15;
static constexpr double TGT_VEL_ACCEL_GATE = 0.5;   // [m/s]

// Tail-chase recovery
static constexpr double VC_DIVERGE_THRESH  = -2.0;  // [m/s]

// OFFBOARD priming
static constexpr int    PRIME_COUNT        = 50;
static constexpr int    RETRY_INTERVAL     = 25;

// Feedback publish intervals
static constexpr int    STRIKE_FB_TICKS    = 50;     // every 1 s  at 50 Hz
static constexpr int    RECOVER_FB_TICKS   = 25;     // every 0.5 s

// PX4 mode codes
static constexpr float  PX4_MODE_CUSTOM          = 1.0f;
static constexpr float  PX4_CUSTOM_MAIN_AUTO      = 4.0f;
static constexpr float  PX4_SUB_LOITER            = 3.0f;
static constexpr float  PX4_SUB_RTL               = 5.0f;
static constexpr float  PX4_OFFBOARD_MODE         = 6.0f;

// ═══════════════════════════════════════════════════════════════════════════
//  Type aliases
// ═══════════════════════════════════════════════════════════════════════════

using StrikeAction  = px4_ros2_striker::action::Strike;
using RecoverAction = px4_ros2_striker::action::Recover;
using StrikeHandle  = rclcpp_action::ServerGoalHandle<StrikeAction>;
using RecoverHandle = rclcpp_action::ServerGoalHandle<RecoverAction>;

// ═══════════════════════════════════════════════════════════════════════════
//  Helpers
// ═══════════════════════════════════════════════════════════════════════════

inline Eigen::Vector3d lpf3(double alpha,
                            const Eigen::Vector3d& x,
                            const Eigen::Vector3d& x_prev)
{
    return alpha * x + (1.0 - alpha) * x_prev;
}

static constexpr float kNaN = std::numeric_limits<float>::quiet_NaN();

// ═══════════════════════════════════════════════════════════════════════════
//  APNFixedWingActionServer
// ═══════════════════════════════════════════════════════════════════════════

class APNFixedWingActionServer : public rclcpp::Node
{
public:

    // ── State machine ────────────────────────────────────────────────────────
    enum class State {
        IDLE,
        STRIKE,
        TERMINAL,   // absorbing — crashed
        RECOVERY,
        RECOVERED   // absorbing — final mode commanded
    };

    // ─────────────────────────────────────────────────────────────────────────
    APNFixedWingActionServer()
    : Node("apn_fw_action_server"),
      state_(State::IDLE),
      has_ref_(false),
      target_initialized_(false), target_time_valid_(false),
      nav_state_(0), setpoint_counter_(0),
      airspeed_(AIRSPEED_FALLBACK),
      drone_pos_ned_(Eigen::Vector3d::Zero()),
      drone_vel_ned_(Eigen::Vector3d::Zero()),
      att_q_(1.0, 0.0, 0.0, 0.0),
      target_pos_ned_(Eigen::Vector3d::Zero()),
      target_vel_ned_(Eigen::Vector3d::Zero()),
      target_accel_ned_(Eigen::Vector3d::Zero()),
      target_pos_prev_(Eigen::Vector3d::Zero()),
      target_vel_prev_(Eigen::Vector3d::Zero()),
      recovery_pos_ned_(Eigen::Vector3d::Zero()),
      strike_fb_tick_(0), recover_fb_tick_(0)
    {
        // ── QoS ──────────────────────────────────────────────────────────────
        auto sensor_qos = rclcpp::QoS(rclcpp::KeepLast(1))
                            .best_effort()
                            .durability_volatile();

        // ── FMU Subscriptions ─────────────────────────────────────────────────
        local_pos_sub_ = this->create_subscription<px4_msgs::msg::VehicleLocalPosition>(
            "/fmu/out/vehicle_local_position", sensor_qos,
            std::bind(&APNFixedWingActionServer::local_pos_cb, this, std::placeholders::_1));

        status_sub_ = this->create_subscription<px4_msgs::msg::VehicleStatus>(
            "/fmu/out/vehicle_status_v1", sensor_qos,
            std::bind(&APNFixedWingActionServer::status_cb, this, std::placeholders::_1));

        attitude_sub_ = this->create_subscription<px4_msgs::msg::VehicleAttitude>(
            "/fmu/out/vehicle_attitude", sensor_qos,
            std::bind(&APNFixedWingActionServer::attitude_cb, this, std::placeholders::_1));

        airspeed_sub_ = this->create_subscription<px4_msgs::msg::AirspeedValidated>(
            "/fmu/out/airspeed_validated", sensor_qos,
            std::bind(&APNFixedWingActionServer::airspeed_cb, this, std::placeholders::_1));

        ack_sub_ = this->create_subscription<px4_msgs::msg::VehicleCommandAck>(
            "/fmu/out/vehicle_command_ack", sensor_qos,
            std::bind(&APNFixedWingActionServer::ack_cb, this, std::placeholders::_1));

        // ── FMU Publishers ────────────────────────────────────────────────────
        cmd_pub_     = this->create_publisher<px4_msgs::msg::VehicleCommand>(
            "/fmu/in/vehicle_command", sensor_qos);
        ocm_pub_     = this->create_publisher<px4_msgs::msg::OffboardControlMode>(
            "/fmu/in/offboard_control_mode", sensor_qos);
        att_sp_pub_  = this->create_publisher<px4_msgs::msg::VehicleAttitudeSetpoint>(
            "/fmu/in/vehicle_attitude_setpoint", sensor_qos);
        traj_sp_pub_ = this->create_publisher<px4_msgs::msg::TrajectorySetpoint>(
            "/fmu/in/trajectory_setpoint", sensor_qos);

        // ── Action Servers ────────────────────────────────────────────────────
        // Use reentrant callback group so action callbacks don't block the timer.
        action_cbg_ = this->create_callback_group(
            rclcpp::CallbackGroupType::Reentrant);

        strike_server_ = rclcpp_action::create_server<StrikeAction>(
            this,
            "/strike_action",
            std::bind(&APNFixedWingActionServer::strike_goal_cb,   this,
                      std::placeholders::_1, std::placeholders::_2),
            std::bind(&APNFixedWingActionServer::strike_cancel_cb,  this,
                      std::placeholders::_1),
            std::bind(&APNFixedWingActionServer::strike_accepted_cb, this,
                      std::placeholders::_1),
            rcl_action_server_get_default_options(),
            action_cbg_);

        recover_server_ = rclcpp_action::create_server<RecoverAction>(
            this,
            "/recover_action",
            std::bind(&APNFixedWingActionServer::recover_goal_cb,   this,
                      std::placeholders::_1, std::placeholders::_2),
            std::bind(&APNFixedWingActionServer::recover_cancel_cb,  this,
                      std::placeholders::_1),
            std::bind(&APNFixedWingActionServer::recover_accepted_cb, this,
                      std::placeholders::_1),
            rcl_action_server_get_default_options(),
            action_cbg_);

        // ── 50 Hz guidance timer ──────────────────────────────────────────────
        timer_ = this->create_wall_timer(
            20ms, std::bind(&APNFixedWingActionServer::timer_cb, this));

        RCLCPP_INFO(this->get_logger(),
            "[APN-FW] Action server ready — /strike_action  /recover_action");
    }

private:
    // ── ROS handles ──────────────────────────────────────────────────────────
    rclcpp::CallbackGroup::SharedPtr action_cbg_;

    rclcpp::Subscription<px4_msgs::msg::VehicleLocalPosition>::SharedPtr  local_pos_sub_;
    rclcpp::Subscription<px4_msgs::msg::VehicleStatus>::SharedPtr         status_sub_;
    rclcpp::Subscription<px4_msgs::msg::VehicleAttitude>::SharedPtr       attitude_sub_;
    rclcpp::Subscription<px4_msgs::msg::AirspeedValidated>::SharedPtr     airspeed_sub_;
    rclcpp::Subscription<px4_msgs::msg::VehicleCommandAck>::SharedPtr     ack_sub_;

    rclcpp::Publisher<px4_msgs::msg::VehicleCommand>::SharedPtr           cmd_pub_;
    rclcpp::Publisher<px4_msgs::msg::OffboardControlMode>::SharedPtr      ocm_pub_;
    rclcpp::Publisher<px4_msgs::msg::VehicleAttitudeSetpoint>::SharedPtr  att_sp_pub_;
    rclcpp::Publisher<px4_msgs::msg::TrajectorySetpoint>::SharedPtr       traj_sp_pub_;

    rclcpp_action::Server<StrikeAction>::SharedPtr  strike_server_;
    rclcpp_action::Server<RecoverAction>::SharedPtr recover_server_;

    rclcpp::TimerBase::SharedPtr timer_;

    // ── Active goal handles (protected by goal_mutex_) ───────────────────────
    std::mutex                            goal_mutex_;
    std::shared_ptr<StrikeHandle>         active_strike_;
    std::shared_ptr<RecoverHandle>        active_recover_;

    // ── State ─────────────────────────────────────────────────────────────────
    State   state_;
    bool    has_ref_;
    bool    target_initialized_, target_time_valid_;
    uint8_t nav_state_;
    int     setpoint_counter_;
    double  airspeed_;

    double ref_lat_, ref_lon_;
    float  ref_alt_;

    Eigen::Vector3d drone_pos_ned_;
    Eigen::Vector3d drone_vel_ned_;
    Eigen::Vector4d att_q_;

    // Target (strike) state
    Eigen::Vector3d target_pos_ned_;
    Eigen::Vector3d target_vel_ned_;
    Eigen::Vector3d target_accel_ned_;
    Eigen::Vector3d target_pos_prev_;
    Eigen::Vector3d target_vel_prev_;
    rclcpp::Time    target_last_stamp_;
    double          recovery_arrival_radius_{DEFAULT_ARRIVAL_R};
    std::string     recovery_mode_{"hold"};

    // Recovery waypoint
    Eigen::Vector3d recovery_pos_ned_;

    // Feedback throttle counters
    int strike_fb_tick_;
    int recover_fb_tick_;

    // ─────────────────────────────────────────────────────────────────────────
    static const char* state_name(State s)
    {
        switch (s) {
            case State::IDLE:      return "IDLE";
            case State::STRIKE:    return "STRIKE";
            case State::TERMINAL:  return "TERMINAL";
            case State::RECOVERY:  return "RECOVERY";
            case State::RECOVERED: return "RECOVERED";
        }
        return "?";
    }

    // ═════════════════════════════════════════════════════════════════════════
    //  FMU Callbacks
    // ═════════════════════════════════════════════════════════════════════════

    void status_cb(const px4_msgs::msg::VehicleStatus::SharedPtr msg)
    {
        nav_state_ = msg->nav_state;
    }

    void attitude_cb(const px4_msgs::msg::VehicleAttitude::SharedPtr msg)
    {
        att_q_ << msg->q[0], msg->q[1], msg->q[2], msg->q[3];
    }

    void airspeed_cb(const px4_msgs::msg::AirspeedValidated::SharedPtr msg)
    {
        if (msg->true_airspeed_m_s > AIRSPEED_MIN) {
            airspeed_ = msg->true_airspeed_m_s;
        }
    }

    void ack_cb(const px4_msgs::msg::VehicleCommandAck::SharedPtr msg)
    {
        if (msg->command == px4_msgs::msg::VehicleCommand::VEHICLE_CMD_DO_SET_MODE) {
            if (msg->result == px4_msgs::msg::VehicleCommandAck::VEHICLE_CMD_RESULT_ACCEPTED) {
                RCLCPP_INFO_ONCE(this->get_logger(), "[APN-FW] Mode change accepted by PX4.");
            } else {
                RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 2000,
                    "[APN-FW] Mode change REJECTED — result=%d", msg->result);
            }
        }
    }

    void local_pos_cb(const px4_msgs::msg::VehicleLocalPosition::SharedPtr msg)
    {
        if ((msg->xy_global || msg->ref_timestamp > 0) && !has_ref_) {
            has_ref_ = true;
            ref_lat_ = msg->ref_lat;
            ref_lon_ = msg->ref_lon;
            ref_alt_ = msg->ref_alt;
            RCLCPP_INFO(this->get_logger(),
                "[APN-FW] GPS origin latched → Lat=%.6f  Lon=%.6f  Alt=%.2fm",
                ref_lat_, ref_lon_, ref_alt_);
        }
        drone_pos_ned_ << msg->x,  msg->y,  msg->z;
        drone_vel_ned_ << msg->vx, msg->vy, msg->vz;
    }

    // ═════════════════════════════════════════════════════════════════════════
    //  GPS → NED
    // ═════════════════════════════════════════════════════════════════════════
    Eigen::Vector3d gps_to_ned(double lat_deg, double lon_deg, double alt_m) const
    {
        const double lat_r     = lat_deg  * M_PI / 180.0;
        const double lon_r     = lon_deg  * M_PI / 180.0;
        const double ref_lat_r = ref_lat_ * M_PI / 180.0;
        const double ref_lon_r = ref_lon_ * M_PI / 180.0;

        Eigen::Vector3d ned;
        ned.x() =  R_EARTH * (lat_r - ref_lat_r);
        ned.y() =  R_EARTH * std::cos(ref_lat_r) * (lon_r - ref_lon_r);
        ned.z() = -(alt_m - static_cast<double>(ref_alt_));
        return ned;
    }

    // ═════════════════════════════════════════════════════════════════════════
    //  Strike action server callbacks
    // ═════════════════════════════════════════════════════════════════════════

    rclcpp_action::GoalResponse strike_goal_cb(
        const rclcpp_action::GoalUUID& /*uuid*/,
        std::shared_ptr<const StrikeAction::Goal> goal)
    {
        // Hard reject: no GPS, or node is crashed
        if (!has_ref_) {
            RCLCPP_WARN(this->get_logger(),
                "[STRIKE] REJECTED — GPS origin not ready.");
            return rclcpp_action::GoalResponse::REJECT;
        }
        if (state_ == State::TERMINAL) {
            RCLCPP_WARN(this->get_logger(),
                "[STRIKE] REJECTED — vehicle is in TERMINAL state (crashed).");
            return rclcpp_action::GoalResponse::REJECT;
        }

        RCLCPP_INFO(this->get_logger(),
            "[STRIKE] Goal accepted → GPS [%.6f, %.6f, %.2f]",
            goal->latitude, goal->longitude, goal->altitude);

        return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
    }

    rclcpp_action::CancelResponse strike_cancel_cb(
        const std::shared_ptr<StrikeHandle> /*handle*/)
    {
        RCLCPP_INFO(this->get_logger(), "[STRIKE] Cancel requested by client.");
        return rclcpp_action::CancelResponse::ACCEPT;
    }

    // Called after goal is accepted. Store handle and trigger state transition.
    void strike_accepted_cb(const std::shared_ptr<StrikeHandle> handle)
    {
        std::lock_guard<std::mutex> lk(goal_mutex_);

        // Preempt any existing strike goal
        if (active_strike_ && active_strike_->is_active()) {
            auto result        = std::make_shared<StrikeAction::Result>();
            result->hit        = false;
            result->miss_distance_m = static_cast<float>(
                (target_pos_ned_ - drone_pos_ned_).norm());
            result->termination = "preempted";
            active_strike_->abort(result);
            RCLCPP_WARN(this->get_logger(), "[STRIKE] Previous goal preempted.");
        }

        // Preempt active recovery (strike takes priority)
        abort_recovery_locked("preempted");

        // Initialise target state from goal GPS
        const auto& goal = handle->get_goal();
        init_target(gps_to_ned(goal->latitude, goal->longitude, goal->altitude));

        active_strike_    = handle;
        setpoint_counter_ = 0;   // Re-prime OFFBOARD
        strike_fb_tick_   = 0;
        state_            = State::STRIKE;

        RCLCPP_INFO(this->get_logger(),
            "[STRIKE] %s → STRIKE. Target NED=[%.1f, %.1f, %.1f]m",
            state_name(state_),
            target_pos_ned_.x(), target_pos_ned_.y(), target_pos_ned_.z());
    }

    // ═════════════════════════════════════════════════════════════════════════
    //  Recover action server callbacks
    // ═════════════════════════════════════════════════════════════════════════

    rclcpp_action::GoalResponse recover_goal_cb(
        const rclcpp_action::GoalUUID& /*uuid*/,
        std::shared_ptr<const RecoverAction::Goal> goal)
    {
        if (!has_ref_) {
            RCLCPP_WARN(this->get_logger(),
                "[RECOVER] REJECTED — GPS origin not ready.");
            return rclcpp_action::GoalResponse::REJECT;
        }
        if (state_ == State::TERMINAL) {
            RCLCPP_WARN(this->get_logger(),
                "[RECOVER] REJECTED — vehicle is in TERMINAL state (crashed).");
            return rclcpp_action::GoalResponse::REJECT;
        }

        const std::string mode = goal->final_mode.empty() ? "hold" : goal->final_mode;
        if (mode != "hold" && mode != "rtl") {
            RCLCPP_WARN(this->get_logger(),
                "[RECOVER] REJECTED — unknown final_mode '%s' (use 'hold' or 'rtl').",
                mode.c_str());
            return rclcpp_action::GoalResponse::REJECT;
        }

        RCLCPP_INFO(this->get_logger(),
            "[RECOVER] Goal accepted → GPS [%.6f, %.6f, %.2f]  mode=%s",
            goal->latitude, goal->longitude, goal->altitude, mode.c_str());

        return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
    }

    rclcpp_action::CancelResponse recover_cancel_cb(
        const std::shared_ptr<RecoverHandle> /*handle*/)
    {
        RCLCPP_INFO(this->get_logger(), "[RECOVER] Cancel requested by client.");
        return rclcpp_action::CancelResponse::ACCEPT;
    }

    void recover_accepted_cb(const std::shared_ptr<RecoverHandle> handle)
    {
        std::lock_guard<std::mutex> lk(goal_mutex_);

        // Preempt any existing recovery goal
        if (active_recover_ && active_recover_->is_active()) {
            auto result         = std::make_shared<RecoverAction::Result>();
            result->arrived     = false;
            result->termination = "preempted";
            active_recover_->abort(result);
            RCLCPP_WARN(this->get_logger(), "[RECOVER] Previous goal preempted.");
        }

        // Abort active strike — recovery takes priority
        abort_strike_locked("preempted");

        const auto& goal = handle->get_goal();

        recovery_pos_ned_       = gps_to_ned(goal->latitude, goal->longitude, goal->altitude);
        recovery_mode_          = goal->final_mode.empty() ? "hold" : goal->final_mode;
        recovery_arrival_radius_ = (goal->arrival_radius_m > 0.0)
                                   ? goal->arrival_radius_m
                                   : DEFAULT_ARRIVAL_R;

        active_recover_   = handle;
        setpoint_counter_ = 0;
        recover_fb_tick_  = 0;
        state_            = State::RECOVERY;

        RCLCPP_INFO(this->get_logger(),
            "[RECOVER] → RECOVERY. WP NED=[%.1f, %.1f, %.1f]m  mode=%s  r=%.1fm",
            recovery_pos_ned_.x(), recovery_pos_ned_.y(), recovery_pos_ned_.z(),
            recovery_mode_.c_str(), recovery_arrival_radius_);
    }

    // ═════════════════════════════════════════════════════════════════════════
    //  Helper: abort active goals (must be called with goal_mutex_ held)
    // ═════════════════════════════════════════════════════════════════════════

    void abort_strike_locked(const std::string& reason)
    {
        if (active_strike_ && active_strike_->is_active()) {
            auto result         = std::make_shared<StrikeAction::Result>();
            result->hit         = false;
            result->miss_distance_m = static_cast<float>(
                (target_pos_ned_ - drone_pos_ned_).norm());
            result->termination = reason;
            active_strike_->abort(result);
            active_strike_.reset();
        }
    }

    void abort_recovery_locked(const std::string& reason)
    {
        if (active_recover_ && active_recover_->is_active()) {
            auto result         = std::make_shared<RecoverAction::Result>();
            result->arrived     = false;
            result->termination = reason;
            active_recover_->abort(result);
            active_recover_.reset();
        }
    }

    // ═════════════════════════════════════════════════════════════════════════
    //  Target state initialisation (called when new strike goal arrives)
    // ═════════════════════════════════════════════════════════════════════════
    void init_target(const Eigen::Vector3d& pos)
    {
        target_pos_ned_     = pos;
        target_pos_prev_    = pos;
        target_vel_ned_     = Eigen::Vector3d::Zero();
        target_accel_ned_   = Eigen::Vector3d::Zero();
        target_vel_prev_    = Eigen::Vector3d::Zero();
        target_initialized_ = false;
        target_time_valid_  = false;
    }

    // Update target position + re-run kinematic estimator each guidance tick.
    // For a GPS-fixed target this is called ONCE in strike_accepted_cb.
    // If your architecture streams updated target GPS, call this each tick
    // with the latest position instead.
    void update_target(const Eigen::Vector3d& p_new)
    {
        const rclcpp::Time now = this->get_clock()->now();
        double tgt_dt = 0.02;
        if (target_time_valid_) {
            tgt_dt = std::clamp((now - target_last_stamp_).seconds(), 0.005, 0.5);
        }
        target_last_stamp_ = now;
        target_time_valid_ = true;

        if (target_initialized_) {
            const Eigen::Vector3d raw_vel = (p_new - target_pos_prev_) / tgt_dt;
            target_vel_ned_ = lpf3(ALPHA_TGT_VEL, raw_vel, target_vel_ned_);

            if (target_vel_ned_.norm() > TGT_VEL_ACCEL_GATE) {
                const Eigen::Vector3d raw_acc =
                    (target_vel_ned_ - target_vel_prev_) / tgt_dt;
                target_accel_ned_ = lpf3(ALPHA_TGT_ACCEL, raw_acc, target_accel_ned_);
            } else {
                target_accel_ned_ = lpf3(ALPHA_TGT_ACCEL,
                                         Eigen::Vector3d::Zero(), target_accel_ned_);
            }
            target_vel_prev_ = target_vel_ned_;
        } else {
            target_initialized_ = true;
        }
        target_pos_prev_ = p_new;
        target_pos_ned_  = p_new;
    }

    // ═════════════════════════════════════════════════════════════════════════
    //  APN Guidance
    // ═════════════════════════════════════════════════════════════════════════
    Eigen::Vector3d compute_apn_accel_ned()
    {
        const Eigen::Vector3d R_vec  = target_pos_ned_ - drone_pos_ned_;
        double                R_mag  = R_vec.norm();
        if (R_mag < 0.1) R_mag = 0.1;

        const Eigen::Vector3d lambda_hat = R_vec / R_mag;

        double V_mag = drone_vel_ned_.norm();
        if (V_mag < 1.0) V_mag = 1.0;
        const Eigen::Vector3d v_hat = drone_vel_ned_ / V_mag;

        const Eigen::Vector3d V_rel = target_vel_ned_ - drone_vel_ned_;
        const double          Vc    = -(V_rel.dot(lambda_hat));

        // Tail-chase recovery: if diverging AND behind target, pull toward LOS
        const double heading_error =
            std::acos(std::clamp(v_hat.dot(lambda_hat), -1.0, 1.0));
        if (Vc < VC_DIVERGE_THRESH && heading_error > M_PI / 2.0) {
            const Eigen::Vector3d pull =
                (lambda_hat - lambda_hat.dot(v_hat) * v_hat).normalized();
            RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 1000,
                "[STRIKE] Tail-chase (Vc=%.1f m/s, HE=%.1f°) — re-acquiring.",
                Vc, heading_error * 180.0 / M_PI);
            return pull * MAX_LAT_ACCEL;
        }

        // LOS angular rate
        const Eigen::Vector3d Omega = R_vec.cross(V_rel) / (R_mag * R_mag);

        // True PN: a = N · |V_drone| · (Ω × v̂)
        const Eigen::Vector3d a_PN = Omega.cross(v_hat) * (APN_N * V_mag);

        // Augmented term stripped along v̂ (TPN frame)
        const Eigen::Vector3d a_tgt_perp =
            target_accel_ned_ - target_accel_ned_.dot(v_hat) * v_hat;
        const Eigen::Vector3d a_aug = a_tgt_perp * (APN_N / 2.0);

        Eigen::Vector3d a_cmd = a_PN + a_aug;
        if (a_cmd.norm() > MAX_LAT_ACCEL) {
            a_cmd = a_cmd.normalized() * MAX_LAT_ACCEL;
        }
        return a_cmd;
    }

    // ═════════════════════════════════════════════════════════════════════════
    //  Attitude inversion (split-channel)
    // ═════════════════════════════════════════════════════════════════════════
    struct AttitudeCmd { double roll, pitch, yaw; };

    AttitudeCmd accel_ned_to_attitude(const Eigen::Vector3d& a_cmd) const
    {
        const Eigen::Vector3d R_vec = target_pos_ned_ - drone_pos_ned_;

        // ROLL — horizontal PN component only → body-lateral
        const Eigen::Vector3d    a_horiz{a_cmd(0), a_cmd(1), 0.0};
        const Eigen::Quaterniond q(att_q_(0), att_q_(1), att_q_(2), att_q_(3));
        const Eigen::Matrix3d    R_body_from_ned =
            q.normalized().toRotationMatrix().transpose();
        const double roll = std::clamp(
            std::atan2((R_body_from_ned * a_horiz)(1), G),
            -MAX_ROLL_RAD, MAX_ROLL_RAD);

        // PITCH — LOS elevation tracker using inertial FPA
        const double R_hz = std::max(
            std::sqrt(R_vec(0)*R_vec(0) + R_vec(1)*R_vec(1)), 0.1);
        const double los_el = std::atan2(R_vec(2), R_hz);

        // FPA from inertial ground speed (NOT scalar airspeed)
        const double v_horiz = std::max(std::sqrt(
            drone_vel_ned_(0)*drone_vel_ned_(0) +
            drone_vel_ned_(1)*drone_vel_ned_(1)), 1.0);
        const double fpa = std::atan2(drone_vel_ned_(2), v_horiz);

        const double pitch = std::clamp(
            -KP_PITCH * (los_el - fpa), -MAX_PITCH_RAD, MAX_PITCH_RAD);

        // YAW — flight-path direction (no sideslip), fallback to LOS
        const double v_gnd = std::sqrt(
            drone_vel_ned_(0)*drone_vel_ned_(0) +
            drone_vel_ned_(1)*drone_vel_ned_(1));
        const double yaw = (v_gnd > 1.0)
            ? std::atan2(drone_vel_ned_(1), drone_vel_ned_(0))
            : std::atan2(R_vec(1), R_vec(0));

        return {roll, pitch, yaw};
    }

    // ═════════════════════════════════════════════════════════════════════════
    //  50 Hz Timer — state machine dispatcher
    // ═════════════════════════════════════════════════════════════════════════
    void timer_cb()
    {
        if (!has_ref_) return;
        if (state_ == State::TERMINAL) return;  // absorbing, timer already cancelled

        const uint64_t ts = this->get_clock()->now().nanoseconds() / 1000;

        // OCM heartbeat — must arrive before guidance commands
        publish_ocm(ts);

        // OFFBOARD priming and retry (only when a mission is active)
        if (state_ == State::STRIKE || state_ == State::RECOVERY) {
            manage_offboard_mode();
        }

        const bool is_offboard =
            (nav_state_ == px4_msgs::msg::VehicleStatus::NAVIGATION_STATE_OFFBOARD);

        if (!is_offboard && state_ == State::STRIKE) return;
        if (!is_offboard && state_ == State::RECOVERY) return;

        switch (state_) {
            case State::STRIKE:   tick_strike(ts);   break;
            case State::RECOVERY: tick_recovery(ts); break;
            default: break;
        }
    }

    // ═════════════════════════════════════════════════════════════════════════
    //  STRIKE tick
    // ═════════════════════════════════════════════════════════════════════════
    void tick_strike(uint64_t ts)
    {
        std::shared_ptr<StrikeHandle> handle;
        {
            std::lock_guard<std::mutex> lk(goal_mutex_);
            handle = active_strike_;
        }
        if (!handle) return;

        // ── Cancellation check ─────────────────────────────────────────────
        if (handle->is_canceling()) {
            auto result         = std::make_shared<StrikeAction::Result>();
            result->hit         = false;
            result->miss_distance_m = static_cast<float>(
                (target_pos_ned_ - drone_pos_ned_).norm());
            result->termination = "cancelled";
            handle->canceled(result);
            {
                std::lock_guard<std::mutex> lk(goal_mutex_);
                active_strike_.reset();
            }
            state_ = State::IDLE;
            RCLCPP_INFO(this->get_logger(), "[STRIKE] Goal cancelled. → IDLE");
            return;
        }

        // ── Geometry ──────────────────────────────────────────────────────
        const Eigen::Vector3d R_vec      = target_pos_ned_ - drone_pos_ned_;
        const double          R_mag      = R_vec.norm();
        const Eigen::Vector3d lambda_hat = R_vec / std::max(R_mag, 0.1);
        const Eigen::Vector3d V_rel      = target_vel_ned_ - drone_vel_ned_;
        const double          Vc         = -(V_rel.dot(lambda_hat));
        const double          t_go       = (Vc > 1.0) ? (R_mag / Vc) : 999.0;

        // ── Terminal condition ─────────────────────────────────────────────
        if (R_mag < HIT_RADIUS || t_go < TGO_LOOKAHEAD_S) {
            auto result         = std::make_shared<StrikeAction::Result>();
            result->hit         = true;
            result->miss_distance_m = static_cast<float>(R_mag);
            result->termination = "hit";
            handle->succeed(result);
            {
                std::lock_guard<std::mutex> lk(goal_mutex_);
                active_strike_.reset();
            }
            enter_terminal();
            return;
        }

        // ── Guidance ──────────────────────────────────────────────────────
        const Eigen::Vector3d a_cmd = compute_apn_accel_ned();
        const AttitudeCmd     att   = accel_ned_to_attitude(a_cmd);
        publish_attitude_setpoint(att.roll, att.pitch, att.yaw, APN_THRUST, ts);

        // ── Feedback (throttled) ──────────────────────────────────────────
        if (++strike_fb_tick_ >= STRIKE_FB_TICKS) {
            strike_fb_tick_ = 0;

            const double Vtan = (V_rel - lambda_hat*V_rel.dot(lambda_hat)).norm();
            const double R_hz = std::max(
                std::sqrt(R_vec(0)*R_vec(0) + R_vec(1)*R_vec(1)), 0.1);
            const double v_horiz = std::max(std::sqrt(
                drone_vel_ned_(0)*drone_vel_ned_(0) +
                drone_vel_ned_(1)*drone_vel_ned_(1)), 1.0);

            auto fb          = std::make_shared<StrikeAction::Feedback>();
            fb->range_m      = static_cast<float>(R_mag);
            fb->vc_ms        = static_cast<float>(Vc);
            fb->vtan_ms      = static_cast<float>(Vtan);
            fb->los_el_deg   = static_cast<float>(
                std::atan2(R_vec(2), R_hz) * 180.0 / M_PI);
            fb->fpa_deg      = static_cast<float>(
                std::atan2(drone_vel_ned_(2), v_horiz) * 180.0 / M_PI);
            fb->roll_deg     = static_cast<float>(att.roll  * 180.0 / M_PI);
            fb->pitch_deg    = static_cast<float>(att.pitch * 180.0 / M_PI);
            fb->ias_ms       = static_cast<float>(airspeed_);
            fb->t_go_s       = static_cast<float>(t_go);
            handle->publish_feedback(fb);

            RCLCPP_INFO(this->get_logger(),
                "[STRIKE] r=%.1fm tgo=%.2fs | Vc=%.1f Vtan=%.1f m/s | "
                "LOS=%.1f° FPA=%.1f° | roll=%.1f° pitch=%.1f° | IAS=%.1fm/s",
                R_mag, t_go, Vc, Vtan,
                fb->los_el_deg, fb->fpa_deg,
                att.roll * 180.0/M_PI, att.pitch * 180.0/M_PI, airspeed_);
        }
    }

    // ═════════════════════════════════════════════════════════════════════════
    //  RECOVERY tick
    // ═════════════════════════════════════════════════════════════════════════
    void tick_recovery(uint64_t ts)
    {
        std::shared_ptr<RecoverHandle> handle;
        {
            std::lock_guard<std::mutex> lk(goal_mutex_);
            handle = active_recover_;
        }
        if (!handle) return;

        // ── Cancellation check ─────────────────────────────────────────────
        if (handle->is_canceling()) {
            auto result         = std::make_shared<RecoverAction::Result>();
            result->arrived     = false;
            result->termination = "cancelled";
            handle->canceled(result);
            {
                std::lock_guard<std::mutex> lk(goal_mutex_);
                active_recover_.reset();
            }
            state_ = State::IDLE;
            RCLCPP_INFO(this->get_logger(), "[RECOVER] Goal cancelled. → IDLE");
            return;
        }

        // ── Geometry ──────────────────────────────────────────────────────
        const Eigen::Vector3d err      = recovery_pos_ned_ - drone_pos_ned_;
        const double          dist     = err.norm();
        const double          dist_hz  = std::sqrt(err(0)*err(0) + err(1)*err(1));
        const double          alt_err  = std::abs(err(2));

        // ── Arrival check ─────────────────────────────────────────────────
        if (dist < recovery_arrival_radius_) {
            auto result         = std::make_shared<RecoverAction::Result>();
            result->arrived     = true;
            result->termination = "arrived";
            handle->succeed(result);
            {
                std::lock_guard<std::mutex> lk(goal_mutex_);
                active_recover_.reset();
            }
            enter_recovered();
            return;
        }

        // ── Position setpoint with gentle vertical speed cap ───────────────
        const double yaw_wp = std::atan2(err(1), err(0));
        publish_trajectory_setpoint(recovery_pos_ned_,
                                    std::clamp(err(2) * 0.5,
                                               -RECOVERY_MAX_VZ, RECOVERY_MAX_VZ),
                                    yaw_wp, ts);

        // ── Feedback (throttled) ──────────────────────────────────────────
        if (++recover_fb_tick_ >= RECOVER_FB_TICKS) {
            recover_fb_tick_ = 0;

            auto fb          = std::make_shared<RecoverAction::Feedback>();
            fb->dist_m       = static_cast<float>(dist);
            fb->dist_horiz_m = static_cast<float>(dist_hz);
            fb->alt_err_m    = static_cast<float>(alt_err);
            handle->publish_feedback(fb);

            RCLCPP_INFO(this->get_logger(),
                "[RECOVER] dist=%.1fm (hz=%.1f alt=%.1f) wp=[%.1f %.1f %.1f]",
                dist, dist_hz, alt_err,
                recovery_pos_ned_.x(), recovery_pos_ned_.y(), recovery_pos_ned_.z());
        }
    }

    // ═════════════════════════════════════════════════════════════════════════
    //  Terminal and Recovered entry
    // ═════════════════════════════════════════════════════════════════════════
    void enter_terminal()
    {
        state_ = State::TERMINAL;
        timer_->cancel();  // node goes fully silent — PX4 watchdog handles failsafe
        RCLCPP_INFO(this->get_logger(),
            "[APN-FW] ★ TARGET HIT — guidance stopped. Impact imminent.");
    }

    void enter_recovered()
    {
        state_ = State::RECOVERED;
        RCLCPP_INFO(this->get_logger(),
            "[APN-FW] ✓ Recovery waypoint reached — requesting %s.",
            recovery_mode_.c_str());
        request_final_mode();
        // Timer NOT cancelled — node stays alive for a potential new strike goal
    }

    void request_final_mode()
    {
        const float sub_mode = (recovery_mode_ == "rtl")
            ? PX4_SUB_RTL : PX4_SUB_LOITER;
        publish_vehicle_command(
            px4_msgs::msg::VehicleCommand::VEHICLE_CMD_DO_SET_MODE,
            PX4_MODE_CUSTOM, PX4_CUSTOM_MAIN_AUTO, sub_mode);
    }

    // ═════════════════════════════════════════════════════════════════════════
    //  OCM heartbeat — control layer depends on state
    // ═════════════════════════════════════════════════════════════════════════
    void publish_ocm(uint64_t ts)
    {
        const bool is_strike = (state_ == State::STRIKE);
        auto ocm         = px4_msgs::msg::OffboardControlMode();
        ocm.timestamp    = ts;
        ocm.position     = !is_strike;
        ocm.velocity     = false;
        ocm.acceleration = false;
        ocm.attitude     = is_strike;
        ocm.body_rate    = false;
        ocm_pub_->publish(ocm);
    }

    // ═════════════════════════════════════════════════════════════════════════
    //  OFFBOARD mode management
    // ═════════════════════════════════════════════════════════════════════════
    void manage_offboard_mode()
    {
        if (setpoint_counter_ < PRIME_COUNT) {
            ++setpoint_counter_;
            return;
        }

        const bool is_offboard =
            (nav_state_ == px4_msgs::msg::VehicleStatus::NAVIGATION_STATE_OFFBOARD);

        if (!is_offboard) {
            const int ticks = setpoint_counter_ - PRIME_COUNT;
            if (ticks % RETRY_INTERVAL == 0) {
                publish_vehicle_command(
                    px4_msgs::msg::VehicleCommand::VEHICLE_CMD_DO_SET_MODE,
                    PX4_MODE_CUSTOM, PX4_OFFBOARD_MODE);
                RCLCPP_INFO(this->get_logger(),
                    "[APN-FW] Requesting OFFBOARD (nav=%d, state=%s)...",
                    nav_state_, state_name(state_));
            }
        }
        ++setpoint_counter_;
    }

    // ═════════════════════════════════════════════════════════════════════════
    //  Publish helpers
    // ═════════════════════════════════════════════════════════════════════════
    void publish_attitude_setpoint(double roll, double pitch, double yaw,
                                   double thrust, uint64_t ts)
    {
        const Eigen::Quaterniond q =
            Eigen::AngleAxisd(yaw,   Eigen::Vector3d::UnitZ()) *
            Eigen::AngleAxisd(pitch, Eigen::Vector3d::UnitY()) *
            Eigen::AngleAxisd(roll,  Eigen::Vector3d::UnitX());

        auto sp               = px4_msgs::msg::VehicleAttitudeSetpoint();
        sp.timestamp          = ts;
        sp.q_d[0]             = static_cast<float>(q.w());
        sp.q_d[1]             = static_cast<float>(q.x());
        sp.q_d[2]             = static_cast<float>(q.y());
        sp.q_d[3]             = static_cast<float>(q.z());
        sp.thrust_body[0]     = static_cast<float>(thrust);
        sp.thrust_body[1]     = 0.0f;
        sp.thrust_body[2]     = 0.0f;
        sp.yaw_sp_move_rate   = 0.0f;
        att_sp_pub_->publish(sp);
    }

    void publish_trajectory_setpoint(const Eigen::Vector3d& pos_ned,
                                     double vz_cmd,
                                     double yaw, uint64_t ts)
    {
        auto sp            = px4_msgs::msg::TrajectorySetpoint();
        sp.timestamp       = ts;
        sp.position[0]     = static_cast<float>(pos_ned.x());
        sp.position[1]     = static_cast<float>(pos_ned.y());
        sp.position[2]     = static_cast<float>(pos_ned.z());
        sp.yaw             = static_cast<float>(yaw);
        sp.velocity[0]     = kNaN;
        sp.velocity[1]     = kNaN;
        sp.velocity[2]     = static_cast<float>(vz_cmd);  // gentle altitude control
        sp.acceleration[0] = kNaN;
        sp.acceleration[1] = kNaN;
        sp.acceleration[2] = kNaN;
        traj_sp_pub_->publish(sp);
    }

    void publish_vehicle_command(uint16_t command,
                                 float p1 = 0.f, float p2 = 0.f, float p3 = 0.f)
    {
        auto msg             = px4_msgs::msg::VehicleCommand();
        msg.timestamp        = this->get_clock()->now().nanoseconds() / 1000;
        msg.command          = command;
        msg.param1           = p1;
        msg.param2           = p2;
        msg.param3           = p3;
        msg.target_system    = 1;
        msg.target_component = 1;
        msg.source_system    = 1;
        msg.source_component = 1;
        msg.from_external    = true;
        cmd_pub_->publish(msg);
    }
};

// ═══════════════════════════════════════════════════════════════════════════
//  Entry point
//  MultiThreadedExecutor is required: action server callbacks (goal_cb,
//  cancel_cb, accepted_cb) run in the reentrant callback group while the
//  50 Hz timer and FMU topic callbacks run in the default mutually exclusive
//  group — ensuring they never interleave without the mutex.
// ═══════════════════════════════════════════════════════════════════════════
int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<APNFixedWingActionServer>();
    rclcpp::executors::MultiThreadedExecutor executor;
    executor.add_node(node);
    executor.spin();
    rclcpp::shutdown();
    return 0;
}