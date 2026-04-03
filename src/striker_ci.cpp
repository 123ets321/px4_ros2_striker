/**
 * @file apn_fw_action_node.cpp
 * @brief Main node — owns StrikerMode + RecoveryMode + two action servers
 *
 * API target: px4-ros2-interface-lib release/1.16
 *
 * ─────────────────────────────────────────────────────────────────────────────
 *  FIXES
 * ─────────────────────────────────────────────────────────────────────────────
 *
 *  FIX 1 — uint32_t for VehicleCommand::command (critical)
 *    VEHICLE_CMD_SET_NAV_STATE = 100001, which exceeds uint16_t max (65535).
 *    Using uint16_t silently truncated 100001 → 34465, causing the PX4
 *    commander to log "command 34465 unsupported" and drop every mode switch.
 *    publish_vehicle_command() now takes uint32_t, matching the px4_msgs
 *    field type exactly.
 *
 *  FIX 2 — Removed broken DO_SET_MODE fallback
 *    PX4_CUSTOM_MAIN_EXTERNAL = 8 maps to RATTITUDE_LEGACY in the commander's
 *    custom-mode table, not to the external-mode subspace.  The fallback
 *    caused a silent spurious mode switch on every action goal.  Removed.
 *    VEHICLE_CMD_SET_NAV_STATE (100001) is the only command needed.
 *
 *  FIX 3 — Recovery arrival radius clamped to MIN_FW_ARRIVAL_R (150m)
 *    Fixed-wing aircraft cannot reliably hit a small circle at cruise speed.
 *    Any caller-supplied arrival_radius_m below 150m is raised to 150m.
 */

#include <rclcpp/rclcpp.hpp>
#include <rclcpp_action/rclcpp_action.hpp>
#include <px4_msgs/msg/vehicle_local_position.hpp>
#include <px4_msgs/msg/vehicle_command.hpp>
#include <px4_ros2_striker/action/strike.hpp>
#include <px4_ros2_striker/action/recover.hpp>
#include <px4_ros2_striker/striker_mode.hpp>
#include <px4_ros2_striker/recovery_mode.hpp>
#include <Eigen/Dense>
#include <cmath>
#include <memory>
#include <string>

using namespace std::chrono_literals;

namespace apn_fw {

// ── PX4 / MAVLink mode constants ──────────────────────────────────────────────
static constexpr float  PX4_MODE_CUSTOM   = 1.0f;
static constexpr float  PX4_MAIN_AUTO     = 4.0f;
static constexpr float  PX4_SUB_LOITER    = 3.0f;
static constexpr float  PX4_SUB_RTL       = 5.0f;
static constexpr double R_EARTH           = 6371000.0;
static constexpr double HOME_ALT_OFFSET_M = 50.0;

using StrikeAction  = px4_ros2_striker::action::Strike;
using RecoverAction = px4_ros2_striker::action::Recover;
using StrikeHandle  = rclcpp_action::ServerGoalHandle<StrikeAction>;
using RecoverHandle = rclcpp_action::ServerGoalHandle<RecoverAction>;

// ═════════════════════════════════════════════════════════════════════════════
class APNActionNode : public rclcpp::Node
{
public:
    APNActionNode()
    : rclcpp::Node("apn_fw_action_server")
    , _striker_mode(*this)
    , _recovery_mode(*this)
    , _has_ref(false)
    , _terminal(false)
    , _final_mode_sent(false)
    , _recovery_was_active(false)
    , _striker_was_active(false)
    {
        auto sensor_qos = rclcpp::QoS(rclcpp::KeepLast(1))
                            .best_effort()
                            .durability_volatile();

        _cmd_pub = this->create_publisher<px4_msgs::msg::VehicleCommand>(
            "/fmu/in/vehicle_command", sensor_qos);

        // ── Register both modes with PX4 ─────────────────────────────────────
        if (!_striker_mode.doRegister()) {
            RCLCPP_FATAL(this->get_logger(),
                "[APNActionNode] Failed to register StrikerMode. "
                "Is the uXRCE-DDS agent running?");
            throw std::runtime_error("StrikerMode doRegister() failed");
        }
        RCLCPP_INFO(this->get_logger(),
            "[APNActionNode] StrikerMode registered, nav_state=%u",
            static_cast<unsigned>(_striker_mode.id()));

        if (!_recovery_mode.doRegister()) {
            RCLCPP_FATAL(this->get_logger(),
                "[APNActionNode] Failed to register RecoveryMode.");
            throw std::runtime_error("RecoveryMode doRegister() failed");
        }
        RCLCPP_INFO(this->get_logger(),
            "[APNActionNode] RecoveryMode registered, nav_state=%u",
            static_cast<unsigned>(_recovery_mode.id()));

        // ── StrikerMode activation callback → gate RecoveryMode ──────────────
        _striker_mode.setOnActivateCallback([this]() {
            RCLCPP_INFO(this->get_logger(),
                "[APNActionNode] StrikerMode active → enabling RecoveryMode.");
            update_recovery_availability(true);
        });

        // ── GPS origin subscription ───────────────────────────────────────────
        _ref_sub = this->create_subscription<px4_msgs::msg::VehicleLocalPosition>(
            "/fmu/out/vehicle_local_position", sensor_qos,
            [this](const px4_msgs::msg::VehicleLocalPosition::SharedPtr msg) {
                if (!_has_ref && (msg->xy_global || msg->ref_timestamp > 0)) {
                    _has_ref = true;
                    _ref_lat = msg->ref_lat;
                    _ref_lon = msg->ref_lon;
                    _ref_alt = msg->ref_alt;

                    const Eigen::Vector3d home_ned{0.0, 0.0, -HOME_ALT_OFFSET_M};
                    _recovery_mode.setDefaultWaypoint(home_ned);

                    RCLCPP_INFO(this->get_logger(),
                        "[APNActionNode] GPS origin latched → "
                        "Lat=%.6f Lon=%.6f Alt=%.2fm | "
                        "Default recovery WP = home+%.0fm",
                        _ref_lat, _ref_lon,
                        static_cast<double>(_ref_alt),
                        HOME_ALT_OFFSET_M);
                }
            });

        // ── Action servers ────────────────────────────────────────────────────
        _action_cbg = this->create_callback_group(
            rclcpp::CallbackGroupType::Reentrant);

        _strike_server = rclcpp_action::create_server<StrikeAction>(
            this, "/strike_action",
            std::bind(&APNActionNode::strike_goal_cb,     this,
                      std::placeholders::_1, std::placeholders::_2),
            std::bind(&APNActionNode::strike_cancel_cb,   this, std::placeholders::_1),
            std::bind(&APNActionNode::strike_accepted_cb, this, std::placeholders::_1),
            rcl_action_server_get_default_options(), _action_cbg);

        _recover_server = rclcpp_action::create_server<RecoverAction>(
            this, "/recover_action",
            std::bind(&APNActionNode::recover_goal_cb,     this,
                      std::placeholders::_1, std::placeholders::_2),
            std::bind(&APNActionNode::recover_cancel_cb,   this, std::placeholders::_1),
            std::bind(&APNActionNode::recover_accepted_cb, this, std::placeholders::_1),
            rcl_action_server_get_default_options(), _action_cbg);

        // ── Mode lifecycle monitor (200 ms) ───────────────────────────────────
        _mode_monitor = this->create_wall_timer(
            200ms, [this]() { check_mode_lifecycle(); });

        RCLCPP_INFO(this->get_logger(),
            "[APNActionNode] Ready — safe to connect QGC now.\n"
            "  Actions: /strike_action  /recover_action\n"
            "  'APN Strike'   nav_state=%u\n"
            "  'APN Recovery' nav_state=%u  (QGC direct → home +%.0fm)",
            static_cast<unsigned>(_striker_mode.id()),
            static_cast<unsigned>(_recovery_mode.id()),
            HOME_ALT_OFFSET_M);
    }

private:
    // ── Modes ─────────────────────────────────────────────────────────────────
    StrikerMode  _striker_mode;
    RecoveryMode _recovery_mode;

    // ── GPS reference ─────────────────────────────────────────────────────────
    bool   _has_ref;
    double _ref_lat{0.0}, _ref_lon{0.0};
    float  _ref_alt{0.0f};
    bool   _terminal;

    // ── Recovery completion state ─────────────────────────────────────────────
    bool _final_mode_sent;
    bool _recovery_was_active;
    bool _striker_was_active;

    // ── ROS handles ───────────────────────────────────────────────────────────
    rclcpp::CallbackGroup::SharedPtr _action_cbg;
    rclcpp::Subscription<px4_msgs::msg::VehicleLocalPosition>::SharedPtr _ref_sub;
    rclcpp::Publisher<px4_msgs::msg::VehicleCommand>::SharedPtr          _cmd_pub;
    rclcpp_action::Server<StrikeAction>::SharedPtr  _strike_server;
    rclcpp_action::Server<RecoverAction>::SharedPtr _recover_server;
    rclcpp::TimerBase::SharedPtr                    _mode_monitor;

    // ─────────────────────────────────────────────────────────────────────────
    //  GPS → NED
    // ─────────────────────────────────────────────────────────────────────────
    Eigen::Vector3d gps_to_ned(double lat, double lon, double alt) const
    {
        const double lr  = lat * M_PI / 180.0;
        const double lo  = lon * M_PI / 180.0;
        const double rlr = _ref_lat * M_PI / 180.0;
        const double rlo = _ref_lon * M_PI / 180.0;
        return {
             R_EARTH * (lr  - rlr),
             R_EARTH * std::cos(rlr) * (lo - rlo),
            -(alt - static_cast<double>(_ref_alt))
        };
    }

    // ─────────────────────────────────────────────────────────────────────────
    //  activate_mode()
    //
    //  Sends VEHICLE_CMD_SET_NAV_STATE (100001) — the only correct command for
    //  px4_ros2_cpp external modes on PX4 ≥ 1.14.
    //
    //  NOTE: No DO_SET_MODE fallback. There is no valid DO_SET_MODE encoding
    //  for external modes — custom_main_mode=8 maps to RATTITUDE_LEGACY and
    //  causes a silent spurious mode switch.
    // ─────────────────────────────────────────────────────────────────────────
    void activate_mode(px4_ros2::ModeBase& mode)
    {
        publish_vehicle_command(
            px4_msgs::msg::VehicleCommand::VEHICLE_CMD_SET_NAV_STATE,
            static_cast<float>(mode.id()));

        RCLCPP_INFO(this->get_logger(),
            "[APNActionNode] Mode switch requested → nav_state=%u",
            static_cast<unsigned>(mode.id()));
    }

    void update_recovery_availability(bool available)
    {
        _recovery_mode.setAvailable(available);
    }

    // ─────────────────────────────────────────────────────────────────────────
    //  STRIKE action server callbacks
    // ─────────────────────────────────────────────────────────────────────────
    rclcpp_action::GoalResponse strike_goal_cb(
        const rclcpp_action::GoalUUID&,
        std::shared_ptr<const StrikeAction::Goal> goal)
    {
        if (!_has_ref) {
            RCLCPP_WARN(this->get_logger(), "[STRIKE] REJECTED — GPS origin not ready.");
            return rclcpp_action::GoalResponse::REJECT;
        }
        if (_terminal) {
            RCLCPP_WARN(this->get_logger(), "[STRIKE] REJECTED — vehicle TERMINAL.");
            return rclcpp_action::GoalResponse::REJECT;
        }
        RCLCPP_INFO(this->get_logger(),
            "[STRIKE] Goal accepted → GPS [%.6f, %.6f, %.2f]",
            goal->latitude, goal->longitude, goal->altitude);
        return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
    }

    rclcpp_action::CancelResponse strike_cancel_cb(
        const std::shared_ptr<StrikeHandle>)
    {
        RCLCPP_INFO(this->get_logger(), "[STRIKE] Cancel requested.");
        return rclcpp_action::CancelResponse::ACCEPT;
    }

    void strike_accepted_cb(const std::shared_ptr<StrikeHandle> handle)
    {
        const auto& goal = handle->get_goal();
        const Eigen::Vector3d target = gps_to_ned(
            goal->latitude, goal->longitude, goal->altitude);

        // Commit goal state BEFORE mode switch — guard sees it on first tick.
        _striker_mode.setTarget(target);
        _striker_mode.setGoalHandle(handle);
        update_recovery_availability(true);
        activate_mode(_striker_mode);

        RCLCPP_INFO(this->get_logger(),
            "[STRIKE] Mode switch requested. Target NED=[%.1f, %.1f, %.1f]m",
            target.x(), target.y(), target.z());
    }

    // ─────────────────────────────────────────────────────────────────────────
    //  RECOVER action server callbacks
    // ─────────────────────────────────────────────────────────────────────────
    rclcpp_action::GoalResponse recover_goal_cb(
        const rclcpp_action::GoalUUID&,
        std::shared_ptr<const RecoverAction::Goal> goal)
    {
        if (!_has_ref) {
            RCLCPP_WARN(this->get_logger(), "[RECOVER] REJECTED — GPS origin not ready.");
            return rclcpp_action::GoalResponse::REJECT;
        }
        if (_terminal) {
            RCLCPP_WARN(this->get_logger(), "[RECOVER] REJECTED — vehicle TERMINAL.");
            return rclcpp_action::GoalResponse::REJECT;
        }
        const std::string mode = goal->final_mode.empty() ? "hold" : goal->final_mode;
        if (mode != "hold" && mode != "rtl") {
            RCLCPP_WARN(this->get_logger(),
                "[RECOVER] REJECTED — unknown final_mode '%s'.", mode.c_str());
            return rclcpp_action::GoalResponse::REJECT;
        }
        RCLCPP_INFO(this->get_logger(),
            "[RECOVER] Goal accepted → GPS [%.6f, %.6f, %.2f] mode=%s",
            goal->latitude, goal->longitude, goal->altitude, mode.c_str());
        return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
    }

    rclcpp_action::CancelResponse recover_cancel_cb(
        const std::shared_ptr<RecoverHandle>)
    {
        RCLCPP_INFO(this->get_logger(), "[RECOVER] Cancel requested.");
        return rclcpp_action::CancelResponse::ACCEPT;
    }

    void recover_accepted_cb(const std::shared_ptr<RecoverHandle> handle)
    {
        const auto& goal = handle->get_goal();
        const std::string mode = goal->final_mode.empty() ? "hold" : goal->final_mode;

        // Clamp arrival radius to MIN_FW_ARRIVAL_R (100 m).
        // We use our own distance check with attitude control — NAV_LOITER_RAD
        // is irrelevant here since PX4's navigator never runs.
        const double radius = std::max(
            (goal->arrival_radius_m > 0.0f)
                ? static_cast<double>(goal->arrival_radius_m)
                : DEFAULT_ARRIVAL_R,
            MIN_FW_ARRIVAL_R);

        // Default home substitution if goal carries no GPS coordinate.
        const bool goal_has_gps =
            (std::abs(goal->latitude)  > 1e-6 ||
             std::abs(goal->longitude) > 1e-6 ||
             std::abs(goal->altitude)  > 1e-3);

        Eigen::Vector3d wp;
        if (goal_has_gps) {
            wp = gps_to_ned(goal->latitude, goal->longitude, goal->altitude);
            RCLCPP_INFO(this->get_logger(),
                "[RECOVER] Using goal GPS → NED=[%.1f, %.1f, %.1f]m",
                wp.x(), wp.y(), wp.z());
        } else {
            wp = Eigen::Vector3d{0.0, 0.0, -HOME_ALT_OFFSET_M};
            RCLCPP_INFO(this->get_logger(),
                "[RECOVER] No GPS in goal → using default home NED=[%.1f, %.1f, %.1f]m",
                wp.x(), wp.y(), wp.z());
        }

        // Commit goal state BEFORE mode switch so that hasActiveGoal() returns
        // true on the very first updateSetpoint() tick.
        _recovery_mode.setWaypoint(wp, mode, radius);
        _recovery_mode.setGoalHandle(handle);

        // Ensure availability is set regardless of what the lifecycle monitor
        // may have done after the striker terminated.
        update_recovery_availability(true);

        _final_mode_sent     = false;
        _recovery_was_active = false;

        activate_mode(_recovery_mode);

        RCLCPP_INFO(this->get_logger(),
            "[RECOVER] Mode switch requested. WP=[%.1f %.1f %.1f]m mode=%s r=%.1fm",
            wp.x(), wp.y(), wp.z(), mode.c_str(), radius);
    }

    // ─────────────────────────────────────────────────────────────────────────
    //  Mode lifecycle monitor (200 ms timer)
    // ─────────────────────────────────────────────────────────────────────────
    void check_mode_lifecycle()
    {
        // ── (1) Striker lifecycle ─────────────────────────────────────────────
        const bool striker_active = _striker_mode.isActive();

        if (striker_active) {
            _striker_was_active = true;
        } else if (_striker_was_active) {
            if (!_recovery_mode.hasActiveGoal() && !_recovery_mode.isActive()) {
                _striker_was_active = false;
                update_recovery_availability(false);
                RCLCPP_INFO(this->get_logger(),
                    "[APNActionNode] StrikerMode deactivated → "
                    "RecoveryMode availability disabled.");
            }
        }

        // ── (2) Recovery lifecycle ────────────────────────────────────────────
        const bool recovery_active = _recovery_mode.isActive();

        if (recovery_active) {
            _recovery_was_active = true;
            return;
        }

        if (_recovery_was_active && !_final_mode_sent
            && !_recovery_mode.hasActiveGoal())
        {
            _final_mode_sent     = true;
            _recovery_was_active = false;
            request_final_mode(_recovery_mode.finalMode());
        }
    }

    // ─────────────────────────────────────────────────────────────────────────
    //  Final mode after recovery arrives
    // ─────────────────────────────────────────────────────────────────────────
    void request_final_mode(const std::string& mode)
    {
        // Switch PX4 into AUTO.LOITER or AUTO.RTL after successful recovery.
        // DO_SET_MODE is the correct command here (different from VEHICLE_CMD_SET_NAV_STATE
        // which is used to switch into registered external modes only).
        const float sub = (mode == "rtl") ? PX4_SUB_RTL : PX4_SUB_LOITER;
        publish_vehicle_command(
            px4_msgs::msg::VehicleCommand::VEHICLE_CMD_DO_SET_MODE,
            PX4_MODE_CUSTOM, PX4_MAIN_AUTO, sub);
        RCLCPP_INFO(this->get_logger(),
            "[APNActionNode] ✔ Recovery complete — switching to final mode: %s", mode.c_str());
    }

    // ─────────────────────────────────────────────────────────────────────────
    //  Low-level vehicle command publisher
    //
    //  FIX 1: cmd is uint32_t — VEHICLE_CMD_SET_NAV_STATE = 100001 exceeds
    //  uint16_t max (65535) and would silently truncate to 34465 otherwise.
    // ─────────────────────────────────────────────────────────────────────────
    void publish_vehicle_command(uint32_t cmd,   // ← uint32_t, NOT uint16_t
                                 float p1 = 0.f,
                                 float p2 = 0.f,
                                 float p3 = 0.f)
    {
        auto msg              = px4_msgs::msg::VehicleCommand();
        msg.timestamp         = this->get_clock()->now().nanoseconds() / 1000;
        msg.command           = cmd;
        msg.param1            = p1;
        msg.param2            = p2;
        msg.param3            = p3;
        msg.param4            = 0.f;
        msg.param5            = 0.0;
        msg.param6            = 0.0;
        msg.param7            = 0.f;
        msg.target_system     = 1;
        msg.target_component  = 1;
        msg.source_system     = 1;
        msg.source_component  = 1;
        msg.from_external     = true;
        _cmd_pub->publish(msg);
    }
};

}  // namespace apn_fw

// ═════════════════════════════════════════════════════════════════════════════
//  Entry point
// ═════════════════════════════════════════════════════════════════════════════
int main(int argc, char** argv)
{
    rclcpp::init(argc, argv);
    rclcpp::executors::MultiThreadedExecutor exec;
    auto node = std::make_shared<apn_fw::APNActionNode>();
    exec.add_node(node);
    exec.spin();
    rclcpp::shutdown();
    return 0;
}