/**
 * @file apn_fw_striker_node.cpp
 * @brief Fixed-Wing APN Guidance Striker — ROS2 Offboard Node
 *
 * ─────────────────────────────────────────────────────────────────────────────
 *  STATE MACHINE
 * ─────────────────────────────────────────────────────────────────────────────
 *
 *   IDLE ─── has_target_ && is_offboard ──► STRIKE
 *   IDLE ─── has_recovery_ && is_offboard ──► RECOVERY  (no strike target)
 *
 *   STRIKE ─── R < HIT_RADIUS || t_go < 150ms ──► TERMINAL
 *   STRIKE ─── /recovery_point arrives ──► RECOVERY  (strike aborted)
 *
 *   TERMINAL  (absorbing) — timer cancelled, node silent. Vehicle crashes.
 *
 *   RECOVERY ─── dist < arrival_radius ──► RECOVERED
 *
 *   RECOVERED (absorbing) — final mode commanded (HOLD or RTL), timer cancelled.
 *
 *   NOTE: /recovery_point is ignored in TERMINAL and RECOVERED states.
 *
 * ─────────────────────────────────────────────────────────────────────────────
 *  GUIDANCE LAW  — True Proportional Navigation (TPN)
 * ─────────────────────────────────────────────────────────────────────────────
 *
 *  R       = p_target − p_drone                   LOS vector [NED, m]
 *  V_rel   = v_target − v_drone                   relative velocity [NED, m/s]
 *  λ̂       = R / |R|                              LOS unit vector
 *  v̂       = v_drone / |v_drone|                  flight-path unit vector
 *  Vc      = −(V_rel · λ̂)                         closing speed (+ve = closing)
 *  Ω       = (R × V_rel) / |R|²                   LOS angular rate [rad/s]
 *
 *  a_PN    = N · |V_drone| · (Ω × v̂)             True PN — always lateral to v̂
 *  a_aug   = (N/2) · [a_tgt − (a_tgt·v̂)v̂]        Augmented term stripped along v̂
 *  a_cmd   = a_PN + a_aug                          [saturated to 2g]
 *
 *  Attitude inversion (split-channel):
 *      roll    = atan2(a_body_horiz_y, G)          horizontal PN, fixed G denom
 *      pitch   = −Kp·(los_el − fpa)               LOS elevation tracking (TAS FPA)
 *      yaw     = atan2(v_drone_E, v_drone_N)       flight-path yaw (no sideslip)
 *
 * ─────────────────────────────────────────────────────────────────────────────
 *  ROS2 Parameters
 * ─────────────────────────────────────────────────────────────────────────────
 *  recovery_mode           string   "hold" | "rtl"    default: "hold"
 *  recovery_arrival_radius double   [m]               default: 10.0
 *
 * ─────────────────────────────────────────────────────────────────────────────
 *  Topics
 * ─────────────────────────────────────────────────────────────────────────────
 *  SUB  /strike_point                           sensor_msgs/NavSatFix
 *  SUB  /recovery_point                         sensor_msgs/NavSatFix
 *  SUB  /fmu/out/vehicle_local_position         px4_msgs/VehicleLocalPosition
 *  SUB  /fmu/out/vehicle_status                 px4_msgs/VehicleStatus
 *  SUB  /fmu/out/vehicle_attitude               px4_msgs/VehicleAttitude
 *  SUB  /fmu/out/airspeed_validated             px4_msgs/AirspeedValidated
 *  PUB  /fmu/in/offboard_control_mode           px4_msgs/OffboardControlMode
 *  PUB  /fmu/in/vehicle_attitude_setpoint       px4_msgs/VehicleAttitudeSetpoint
 *  PUB  /fmu/in/trajectory_setpoint             px4_msgs/TrajectorySetpoint
 *  PUB  /fmu/in/vehicle_command                 px4_msgs/VehicleCommand
 */

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/nav_sat_fix.hpp>
#include <px4_msgs/msg/vehicle_local_position.hpp>
#include <px4_msgs/msg/vehicle_status.hpp>
#include <px4_msgs/msg/vehicle_attitude.hpp>
#include <px4_msgs/msg/airspeed_validated.hpp>
#include <px4_msgs/msg/offboard_control_mode.hpp>
#include <px4_msgs/msg/vehicle_attitude_setpoint.hpp>
#include <px4_msgs/msg/trajectory_setpoint.hpp>
#include <px4_msgs/msg/vehicle_command.hpp>
#include <px4_msgs/msg/vehicle_command_ack.hpp>
#include <Eigen/Dense>
#include <Eigen/Geometry>
#include <cmath>
#include <limits>
#include <string>

using namespace std::chrono_literals;

// ═══════════════════════════════════════════════════════════════════════════
//  Tuning Constants
// ═══════════════════════════════════════════════════════════════════════════

static constexpr double APN_N               = 4.5;
static constexpr double MAX_ROLL_RAD        = 55.0 * M_PI / 180.0;
static constexpr double MAX_PITCH_RAD       = 40.0 * M_PI / 180.0;
static constexpr double APN_THRUST          = 1.0;    // Verify FW_THR_MAX not clamped
static constexpr double HIT_RADIUS          = 5.0;    // Proximity fuse [m]
static constexpr double TGO_LOOKAHEAD_S     = 0.15;   // t_go terminal threshold [s]
static constexpr double MAX_LAT_ACCEL       = 2.0 * 9.81;
static constexpr double AIRSPEED_MIN        = 5.0;
static constexpr double AIRSPEED_FALLBACK   = 15.0;
static constexpr double G                   = 9.81;
static constexpr double DT                  = 0.02;   // Guidance loop [s] — NOT used
                                                       // in target estimator (see FIX-1)
static constexpr double R_EARTH             = 6371000.0;
static constexpr double KP_PITCH            = 1.5;

// IIR LPF — target state estimator
static constexpr double ALPHA_TGT_VEL       = 0.25;
static constexpr double ALPHA_TGT_ACCEL     = 0.35;
static constexpr double TGT_VEL_ACCEL_GATE  = 0.5;   // [m/s] zero accel below this

// Tail-chase recovery
static constexpr double VC_DIVERGE_THRESH   = -2.0;  // [m/s]

// Offboard priming
static constexpr int    PRIME_COUNT         = 50;
static constexpr int    RETRY_INTERVAL      = 25;

// PX4 custom sub-mode codes used with VEHICLE_CMD_DO_SET_MODE
static constexpr float  PX4_CUSTOM_MAIN_MODE_AUTO        = 4.0f;
static constexpr float  PX4_CUSTOM_SUB_MODE_AUTO_LOITER  = 3.0f;
static constexpr float  PX4_CUSTOM_SUB_MODE_AUTO_RTL     = 5.0f;

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
//  APNFixedWingStrikerNode
// ═══════════════════════════════════════════════════════════════════════════
class APNFixedWingStrikerNode : public rclcpp::Node
{
public:

    // ── State machine ────────────────────────────────────────────────────────
    enum class State {
        IDLE,       // Priming OFFBOARD, waiting for a mission
        STRIKE,     // APN attitude guidance active
        TERMINAL,   // Hit detected — all output stopped, vehicle crashing
        RECOVERY,   // Flying to recovery waypoint on position setpoints
        RECOVERED   // Arrived — HOLD or RTL commanded, node dormant
    };

    APNFixedWingStrikerNode()
    : Node("apn_fw_striker_node"),
      state_(State::IDLE),
      has_ref_(false), has_target_(false), has_recovery_(false),
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
      recovery_pos_ned_(Eigen::Vector3d::Zero())
    {
        // ── ROS2 parameters ──────────────────────────────────────────────────
        this->declare_parameter<std::string>("recovery_mode",           "hold");
        this->declare_parameter<double>     ("recovery_arrival_radius", 10.0);

        recovery_mode_           = this->get_parameter("recovery_mode").as_string();
        recovery_arrival_radius_ = this->get_parameter("recovery_arrival_radius").as_double();

        if (recovery_mode_ != "hold" && recovery_mode_ != "rtl") {
            RCLCPP_WARN(this->get_logger(),
                "[APN-FW] Unknown recovery_mode '%s' — defaulting to 'hold'.",
                recovery_mode_.c_str());
            recovery_mode_ = "hold";
        }

        RCLCPP_INFO(this->get_logger(),
            "[APN-FW] recovery_mode=%s  arrival_radius=%.1fm",
            recovery_mode_.c_str(), recovery_arrival_radius_);

        // ── QoS ──────────────────────────────────────────────────────────────
        auto sensor_qos = rclcpp::QoS(rclcpp::KeepLast(1))
                            .best_effort()
                            .durability_volatile();

        // ── Subscriptions ─────────────────────────────────────────────────────
        target_sub_ = this->create_subscription<sensor_msgs::msg::NavSatFix>(
            "/strike_point", 10,
            std::bind(&APNFixedWingStrikerNode::target_cb, this, std::placeholders::_1));

        recovery_sub_ = this->create_subscription<sensor_msgs::msg::NavSatFix>(
            "/recovery_point", 10,
            std::bind(&APNFixedWingStrikerNode::recovery_cb, this, std::placeholders::_1));

        local_pos_sub_ = this->create_subscription<px4_msgs::msg::VehicleLocalPosition>(
            "/fmu/out/vehicle_local_position", sensor_qos,
            std::bind(&APNFixedWingStrikerNode::local_pos_cb, this, std::placeholders::_1));

        status_sub_ = this->create_subscription<px4_msgs::msg::VehicleStatus>(
            "/fmu/out/vehicle_status_v1", sensor_qos,
            std::bind(&APNFixedWingStrikerNode::status_cb, this, std::placeholders::_1));

        attitude_sub_ = this->create_subscription<px4_msgs::msg::VehicleAttitude>(
            "/fmu/out/vehicle_attitude", sensor_qos,
            std::bind(&APNFixedWingStrikerNode::attitude_cb, this, std::placeholders::_1));

        airspeed_sub_ = this->create_subscription<px4_msgs::msg::AirspeedValidated>(
            "/fmu/out/airspeed_validated", sensor_qos,
            std::bind(&APNFixedWingStrikerNode::airspeed_cb, this, std::placeholders::_1));

        ack_sub_ = this->create_subscription<px4_msgs::msg::VehicleCommandAck>(
            "/fmu/out/vehicle_command_ack", sensor_qos,
            std::bind(&APNFixedWingStrikerNode::ack_cb, this, std::placeholders::_1));

        // ── Publishers ────────────────────────────────────────────────────────
        cmd_pub_     = this->create_publisher<px4_msgs::msg::VehicleCommand>(
            "/fmu/in/vehicle_command", sensor_qos);
        ocm_pub_     = this->create_publisher<px4_msgs::msg::OffboardControlMode>(
            "/fmu/in/offboard_control_mode", sensor_qos);
        att_sp_pub_  = this->create_publisher<px4_msgs::msg::VehicleAttitudeSetpoint>(
            "/fmu/in/vehicle_attitude_setpoint", sensor_qos);
        traj_sp_pub_ = this->create_publisher<px4_msgs::msg::TrajectorySetpoint>(
            "/fmu/in/trajectory_setpoint", sensor_qos);

        // ── 50 Hz timer ───────────────────────────────────────────────────────
        timer_ = this->create_wall_timer(
            20ms, std::bind(&APNFixedWingStrikerNode::timer_cb, this));

        RCLCPP_INFO(this->get_logger(),
            "[APN-FW] Node ready. Waiting for GPS origin and strike/recovery point...");
    }

private:
    // ── ROS handles ──────────────────────────────────────────────────────────
    rclcpp::Subscription<sensor_msgs::msg::NavSatFix>::SharedPtr          target_sub_;
    rclcpp::Subscription<sensor_msgs::msg::NavSatFix>::SharedPtr          recovery_sub_;
    rclcpp::Subscription<px4_msgs::msg::VehicleLocalPosition>::SharedPtr  local_pos_sub_;
    rclcpp::Subscription<px4_msgs::msg::VehicleStatus>::SharedPtr         status_sub_;
    rclcpp::Subscription<px4_msgs::msg::VehicleAttitude>::SharedPtr       attitude_sub_;
    rclcpp::Subscription<px4_msgs::msg::AirspeedValidated>::SharedPtr     airspeed_sub_;
    rclcpp::Subscription<px4_msgs::msg::VehicleCommandAck>::SharedPtr     ack_sub_;

    rclcpp::Publisher<px4_msgs::msg::VehicleCommand>::SharedPtr           cmd_pub_;
    rclcpp::Publisher<px4_msgs::msg::OffboardControlMode>::SharedPtr      ocm_pub_;
    rclcpp::Publisher<px4_msgs::msg::VehicleAttitudeSetpoint>::SharedPtr  att_sp_pub_;
    rclcpp::Publisher<px4_msgs::msg::TrajectorySetpoint>::SharedPtr       traj_sp_pub_;

    rclcpp::TimerBase::SharedPtr timer_;

    // ── State ─────────────────────────────────────────────────────────────────
    State       state_;
    bool        has_ref_, has_target_, has_recovery_;
    bool        target_initialized_, target_time_valid_;
    uint8_t     nav_state_;
    int         setpoint_counter_;
    double      airspeed_;
    std::string recovery_mode_;
    double      recovery_arrival_radius_;

    double ref_lat_, ref_lon_;
    float  ref_alt_;

    Eigen::Vector3d drone_pos_ned_;
    Eigen::Vector3d drone_vel_ned_;
    Eigen::Vector4d att_q_;

    Eigen::Vector3d target_pos_ned_;
    Eigen::Vector3d target_vel_ned_;
    Eigen::Vector3d target_accel_ned_;
    Eigen::Vector3d target_pos_prev_;
    Eigen::Vector3d target_vel_prev_;
    rclcpp::Time    target_last_stamp_;

    Eigen::Vector3d recovery_pos_ned_;

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
        ned.z() = -(alt_m  - static_cast<double>(ref_alt_));
        return ned;
    }

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
    //  Callbacks
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
                    "[APN-FW] Mode change rejected — result=%d", msg->result);
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

    // ─────────────────────────────────────────────────────────────────────────
    //  target_cb — FIX-1: real inter-message dt for finite differences
    // ─────────────────────────────────────────────────────────────────────────
    void target_cb(const sensor_msgs::msg::NavSatFix::SharedPtr msg)
    {
        if (!has_ref_) {
            RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 2000,
                "[APN-FW] GPS origin not ready — ignoring strike point.");
            return;
        }

        const Eigen::Vector3d p_new =
            gps_to_ned(msg->latitude, msg->longitude, msg->altitude);

        // FIX-1: measure real elapsed time between consecutive messages
        const rclcpp::Time now = this->get_clock()->now();
        double tgt_dt = DT;
        if (target_time_valid_) {
            tgt_dt = std::clamp((now - target_last_stamp_).seconds(), 0.005, 0.5);
        }
        target_last_stamp_ = now;
        target_time_valid_ = true;

        if (target_initialized_) {
            const Eigen::Vector3d raw_vel = (p_new - target_pos_prev_) / tgt_dt;
            target_vel_ned_ = lpf3(ALPHA_TGT_VEL, raw_vel, target_vel_ned_);

            // FIX-6: noise gate — zero accel estimate when target is stationary
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
            target_vel_ned_     = Eigen::Vector3d::Zero();
            target_accel_ned_   = Eigen::Vector3d::Zero();
            target_vel_prev_    = Eigen::Vector3d::Zero();
            target_initialized_ = true;
        }

        target_pos_prev_ = p_new;
        target_pos_ned_  = p_new;

        RCLCPP_INFO(this->get_logger(),
            "[APN-FW] Strike point acquired/updated → NED [N=%.2f E=%.2f D=%.2f]m",
            target_pos_ned_.x(), target_pos_ned_.y(), target_pos_ned_.z());

        // Re-enable striking if we were idle, previously recovering, or recovered
        if (state_ == State::IDLE || state_ == State::RECOVERY || state_ == State::RECOVERED) {
            RCLCPP_WARN(this->get_logger(),
                "[APN-FW] %s → STRIKE.", state_name(state_));
            state_ = State::STRIKE;
            setpoint_counter_ = 0; // Reset offboard priming so it regains OFFBOARD
        }

        has_target_ = true;
    }

    // ─────────────────────────────────────────────────────────────────────────
    //  recovery_cb
    //
    //  Accepts a NavSatFix on /recovery_point and converts it to local NED.
    //  Transitions to RECOVERY from IDLE or STRIKE (aborting the strike).
    //  Silently ignored in TERMINAL (crashed) and RECOVERED (already done).
    //  If already in RECOVERY, the waypoint is simply updated in place —
    //  useful for an operator correcting the recovery point mid-flight.
    // ─────────────────────────────────────────────────────────────────────────
    void recovery_cb(const sensor_msgs::msg::NavSatFix::SharedPtr msg)
    {
        if (!has_ref_) {
            RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 2000,
                "[APN-FW] GPS origin not ready — ignoring recovery point.");
            return;
        }

        // Terminal is inherently absorbing — vehicle crashed. 
        if (state_ == State::TERMINAL) {
            RCLCPP_WARN_ONCE(this->get_logger(),
                "[APN-FW] Recovery point received in TERMINAL state — ignored.");
            return;
        }

        recovery_pos_ned_ = gps_to_ned(msg->latitude, msg->longitude, msg->altitude);
        has_recovery_     = true;

        RCLCPP_INFO(this->get_logger(),
            "[APN-FW] Recovery point set → NED [N=%.2f E=%.2f D=%.2f]m  dist=%.1fm",
            recovery_pos_ned_.x(), recovery_pos_ned_.y(), recovery_pos_ned_.z(),
            (recovery_pos_ned_ - drone_pos_ned_).norm());

        // Snap out of whatever we were doing (unless already recovering)
        if (state_ != State::RECOVERY) {
            RCLCPP_WARN(this->get_logger(),
                "[APN-FW] %s → RECOVERY (strike aborted).", state_name(state_));
            state_ = State::RECOVERY;
            setpoint_counter_ = 0; // Reset offboard priming
        }
    }

    // ═════════════════════════════════════════════════════════════════════════
    //  APN Guidance — NED acceleration command  (True PN + Augmented PN)
    // ═════════════════════════════════════════════════════════════════════════
    Eigen::Vector3d compute_apn_accel_ned()  // non-const: RCLCPP_WARN_THROTTLE needs mutable clock
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

        // FIX-3: tail-chase recovery
        const double heading_error =
            std::acos(std::clamp(v_hat.dot(lambda_hat), -1.0, 1.0));
        if (Vc < VC_DIVERGE_THRESH && heading_error > M_PI / 2.0) {
            const Eigen::Vector3d pull_dir =
                (lambda_hat - lambda_hat.dot(v_hat) * v_hat).normalized();
            RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 1000,
                "[APN-FW] Tail-chase detected (Vc=%.1f m/s, HE=%.1f°) — re-acquisition.",
                Vc, heading_error * 180.0 / M_PI);
            return pull_dir * MAX_LAT_ACCEL;
        }

        const Eigen::Vector3d Omega = R_vec.cross(V_rel) / (R_mag * R_mag);

        // True PN — lateral to flight path v̂
        const Eigen::Vector3d a_PN  = Omega.cross(v_hat) * (APN_N * V_mag);

        // FIX-2: augmented term strips along v̂ (TPN frame), not λ̂ (Pure PN frame)
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
    //  Fixed-Wing Attitude Command (split-channel)
    // ═════════════════════════════════════════════════════════════════════════
    struct AttitudeCmd { double roll, pitch, yaw; };

    AttitudeCmd accel_ned_to_attitude(const Eigen::Vector3d& a_cmd) const
    {
        const Eigen::Vector3d R_vec = target_pos_ned_ - drone_pos_ned_;

        // ROLL: horizontal PN → body-lateral
        const Eigen::Vector3d    a_horiz{a_cmd(0), a_cmd(1), 0.0};
        const Eigen::Quaterniond q(att_q_(0), att_q_(1), att_q_(2), att_q_(3));
        const Eigen::Matrix3d    R_body_from_ned =
            q.normalized().toRotationMatrix().transpose();
        const Eigen::Vector3d    a_body_horiz = R_body_from_ned * a_horiz;

        const double roll = std::clamp(
            std::atan2(a_body_horiz(1), G), -MAX_ROLL_RAD, MAX_ROLL_RAD);

        // PITCH: LOS elevation tracking — FIX-5: TAS for FPA
        const double R_horiz_mag = std::max(
            std::sqrt(R_vec(0)*R_vec(0) + R_vec(1)*R_vec(1)), 0.1);
        const double los_el  = std::atan2(R_vec(2), R_horiz_mag);
        const double tas_horiz = std::max(airspeed_, AIRSPEED_MIN);
        const double fpa       = std::atan2(drone_vel_ned_(2), tas_horiz);

        const double pitch = std::clamp(
            -KP_PITCH * (los_el - fpa), -MAX_PITCH_RAD, MAX_PITCH_RAD);

        // YAW: follow flight path — FIX-7: no sideslip in banked turns
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

        // TERMINAL is the only truly dead state (crashed)
        if (state_ == State::TERMINAL) return;

        const uint64_t ts = this->get_clock()->now().nanoseconds() / 1000;

        // OCM heartbeat
        publish_ocm(ts);

        // Offboard priming and retry (only actively push for offboard if guiding)
        if (state_ == State::STRIKE || state_ == State::RECOVERY) {
            manage_offboard_mode();
        }

        const bool is_offboard =
            (nav_state_ == px4_msgs::msg::VehicleStatus::NAVIGATION_STATE_OFFBOARD);
            
        // If we aren't offboard, don't run the guidance laws
        if (!is_offboard && state_ != State::IDLE && state_ != State::RECOVERED) {
            return;
        }

        // Dispatch
        switch (state_) {
            case State::IDLE:
                if (has_target_) {
                    RCLCPP_INFO(this->get_logger(),
                        "[APN-FW] IDLE → STRIKE");
                    state_ = State::STRIKE;
                } else if (has_recovery_) {
                    RCLCPP_INFO(this->get_logger(),
                        "[APN-FW] IDLE → RECOVERY (no strike target)");
                    state_ = State::RECOVERY;
                }
                break;

            case State::STRIKE:
                run_strike(ts);
                break;

            case State::RECOVERY:
                run_recovery(ts);
                break;

            default:
                break;
        }
    }

    // ═════════════════════════════════════════════════════════════════════════
    //  STRIKE
    // ═════════════════════════════════════════════════════════════════════════
    void run_strike(uint64_t ts)
    {
        const Eigen::Vector3d R_vec      = target_pos_ned_ - drone_pos_ned_;
        const double          R_mag      = R_vec.norm();
        const Eigen::Vector3d lambda_hat = R_vec / std::max(R_mag, 0.1);
        const Eigen::Vector3d V_rel      = target_vel_ned_ - drone_vel_ned_;
        const double          Vc         = -(V_rel.dot(lambda_hat));
        const double          t_go       = (Vc > 1.0) ? (R_mag / Vc) : 999.0;

        // FIX-8: proximity fuse + t_go lookahead
        if (R_mag < HIT_RADIUS || t_go < TGO_LOOKAHEAD_S) {
            enter_terminal();
            return;
        }

        const Eigen::Vector3d a_cmd = compute_apn_accel_ned();
        const AttitudeCmd     att   = accel_ned_to_attitude(a_cmd);

        publish_attitude_setpoint(att.roll, att.pitch, att.yaw, APN_THRUST, ts);
        log_strike_diagnostics(R_vec, R_mag, Vc, t_go, att);
    }

    // ─────────────────────────────────────────────────────────────────────────
    //  enter_terminal
    //
    //  Called once when the proximity fuse triggers.
    //
    //  Stops ALL setpoint output immediately — no pull-up, no thrust cut,
    //  no attitude command.  The vehicle is on a collision course; any
    //  command at this point would only disturb the impact geometry.
    //
    //  The PX4 OCM watchdog (~500ms timeout) will detect the loss of
    //  offboard heartbeat and trigger failsafe, but the impact occurs first.
    // ─────────────────────────────────────────────────────────────────────────
    void enter_terminal()
    {
        state_ = State::TERMINAL;
        timer_->cancel();

        RCLCPP_INFO(this->get_logger(),
            "[APN-FW] ★ TARGET HIT — guidance node stopped. Impact imminent.");
    }

    // ═════════════════════════════════════════════════════════════════════════
    //  RECOVERY
    //
    //  Publishes TrajectorySetpoint (position layer) to the recovery waypoint.
    //  PX4's position controller manages speed, path, and obstacle avoidance.
    //
    //  Yaw is commanded toward the waypoint throughout the approach.
    //  If the operator updates /recovery_point mid-flight, recovery_cb
    //  silently updates recovery_pos_ned_ and this function picks it up
    //  on the next tick — no special handling needed.
    // ═════════════════════════════════════════════════════════════════════════
    void run_recovery(uint64_t ts)
    {
        const Eigen::Vector3d err  = recovery_pos_ned_ - drone_pos_ned_;
        const double          dist = err.norm();

        if (dist < recovery_arrival_radius_) {
            enter_recovered();
            return;
        }

        const double yaw_to_wp = std::atan2(err(1), err(0));
        publish_trajectory_setpoint(recovery_pos_ned_, yaw_to_wp, ts);

        const double dist_hz = std::sqrt(err(0)*err(0) + err(1)*err(1));
        RCLCPP_INFO_THROTTLE(this->get_logger(), *this->get_clock(), 2000,
            "[RECOVERY] dist=%.1fm (hz=%.1f alt_err=%.1f) | wp=[%.1f %.1f %.1f]",
            dist, dist_hz, std::abs(err(2)),
            recovery_pos_ned_.x(), recovery_pos_ned_.y(), recovery_pos_ned_.z());
    }

    // ─────────────────────────────────────────────────────────────────────────
    //  enter_recovered
    //
    //  Called once on arrival at the recovery waypoint.
    //  Commands HOLD or RTL then cancels the timer.
    //  After this the node is fully dormant — PX4 owns the aircraft.
    // ─────────────────────────────────────────────────────────────────────────
    void enter_recovered()
    {
        state_ = State::RECOVERED;

        RCLCPP_INFO(this->get_logger(),
            "[APN-FW] ✓ Recovery waypoint reached — requesting %s.",
            recovery_mode_.c_str());

        request_final_mode();
        // Do NOT cancel the timer. The node stays perfectly alive in RECOVERED state, 
        // pumping harmless OCM heartbeats, ready to instantly snatch the drone 
        // back into OFFBOARD if a new /strike_point arrives!
    }

    // ─────────────────────────────────────────────────────────────────────────
    //  request_final_mode
    //
    //  PX4 DO_SET_MODE convention:
    //    param1 = 1   (custom mode flag)
    //    param2 = 4   (PX4_CUSTOM_MAIN_MODE_AUTO)
    //    param3 = 3   → AUTO_LOITER  (hold)
    //    param3 = 5   → AUTO_RTL
    // ─────────────────────────────────────────────────────────────────────────
    void request_final_mode()
    {
        const float sub_mode = (recovery_mode_ == "rtl")
            ? PX4_CUSTOM_SUB_MODE_AUTO_RTL
            : PX4_CUSTOM_SUB_MODE_AUTO_LOITER;

        publish_vehicle_command(
            px4_msgs::msg::VehicleCommand::VEHICLE_CMD_DO_SET_MODE,
            1.0f,                        // param1: custom mode flag
            PX4_CUSTOM_MAIN_MODE_AUTO,   // param2: AUTO main mode
            sub_mode);                   // param3: LOITER or RTL sub-mode
    }

    // ═════════════════════════════════════════════════════════════════════════
    //  OCM heartbeat — flags depend on state
    //
    //  STRIKE   → attitude = true    (attitude setpoints in use)
    //  RECOVERY → position = true    (TrajectorySetpoint in use)
    //  IDLE     → position = true    (safe default while priming)
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
        // thrust_body[0]: normalised [0,1] forward throttle.
        // Verify FW_THR_MAX in PX4 params is not clamped to a cruise value.
        sp.thrust_body[0]     = static_cast<float>(thrust);
        sp.thrust_body[1]     = 0.0f;
        sp.thrust_body[2]     = 0.0f;
        sp.yaw_sp_move_rate   = 0.0f;
        att_sp_pub_->publish(sp);
    }

    // Position [NED, m] + yaw [rad]. Velocity/accel = NaN → PX4 computes them.
    void publish_trajectory_setpoint(const Eigen::Vector3d& pos_ned,
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
        sp.velocity[2]     = kNaN;
        sp.acceleration[0] = kNaN;
        sp.acceleration[1] = kNaN;
        sp.acceleration[2] = kNaN;
        traj_sp_pub_->publish(sp);
    }

    // ── OFFBOARD mode management ──────────────────────────────────────────────
    void manage_offboard_mode()
    {
        // Don't prime or request OFFBOARD until we actually have a mission
        if (!has_target_ && !has_recovery_) return;

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
                    1.0f, 6.0f);
                RCLCPP_INFO(this->get_logger(),
                    "[APN-FW] Requesting OFFBOARD (nav=%d state=%s)...",
                    nav_state_, state_name(state_));
            }
        }
        ++setpoint_counter_;
    }

    void publish_vehicle_command(uint16_t command,
                                 float param1 = 0.f,
                                 float param2 = 0.f,
                                 float param3 = 0.f)
    {
        auto msg             = px4_msgs::msg::VehicleCommand();
        msg.timestamp        = this->get_clock()->now().nanoseconds() / 1000;
        msg.command          = command;
        msg.param1           = param1;
        msg.param2           = param2;
        msg.param3           = param3;
        msg.target_system    = 1;
        msg.target_component = 1;
        msg.source_system    = 1;
        msg.source_component = 1;
        msg.from_external    = true;
        cmd_pub_->publish(msg);
    }

    // ── Strike diagnostics ────────────────────────────────────────────────────
    void log_strike_diagnostics(const Eigen::Vector3d& R_vec, double R_mag,
                                double Vc, double t_go,
                                const AttitudeCmd& att)
    {
        const Eigen::Vector3d lambda_hat = R_vec / R_mag;
        const Eigen::Vector3d V_rel      = target_vel_ned_ - drone_vel_ned_;
        const double Vtan =
            (V_rel - lambda_hat * V_rel.dot(lambda_hat)).norm();

        const double R_horiz_mag = std::max(
            std::sqrt(R_vec(0)*R_vec(0) + R_vec(1)*R_vec(1)), 0.1);
        const double los_el_deg  = std::atan2(R_vec(2), R_horiz_mag) * 180.0 / M_PI;
        const double tas_horiz   = std::max(airspeed_, AIRSPEED_MIN);
        const double fpa_deg     =
            std::atan2(drone_vel_ned_(2), tas_horiz) * 180.0 / M_PI;

        RCLCPP_INFO_THROTTLE(this->get_logger(), *this->get_clock(), 1000,
            "[STRIKE] r=%.1fm tgo=%.2fs | Vc=%.1f Vtan=%.1f m/s | "
            "LOS_el=%.1f° FPA=%.1f° | roll=%.1f° pitch=%.1f° | IAS=%.1fm/s",
            R_mag, t_go, Vc, Vtan,
            los_el_deg, fpa_deg,
            att.roll  * 180.0 / M_PI,
            att.pitch * 180.0 / M_PI,
            airspeed_);
    }
};

// ═══════════════════════════════════════════════════════════════════════════
//  Entry point
// ═══════════════════════════════════════════════════════════════════════════
int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<APNFixedWingStrikerNode>());
    rclcpp::shutdown();
    return 0;
}