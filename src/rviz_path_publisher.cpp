#include <rclcpp/rclcpp.hpp>
#include <px4_msgs/msg/vehicle_local_position.hpp>
#include <nav_msgs/msg/path.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>

class RvizPathPublisher : public rclcpp::Node
{
public:
    RvizPathPublisher() : Node("rviz_path_publisher")
    {
        // QoS for reading PX4 local position
        auto sensor_qos = rclcpp::QoS(rclcpp::KeepLast(1)).best_effort().durability_volatile();

        local_pos_sub_ = this->create_subscription<px4_msgs::msg::VehicleLocalPosition>(
            "/fmu/out/vehicle_local_position", sensor_qos,
            std::bind(&RvizPathPublisher::local_pos_cb, this, std::placeholders::_1));

        path_pub_ = this->create_publisher<nav_msgs::msg::Path>("/drone_path", 10);
        
        path_msg_.header.frame_id = "map"; // RViz fixed frame
        
        RCLCPP_INFO(this->get_logger(), "[RViz Path Publisher] Started. Forwarding /fmu/out/vehicle_local_position to /drone_path");
    }

private:
    void local_pos_cb(const px4_msgs::msg::VehicleLocalPosition::SharedPtr msg)
    {
        // Create new pose
        geometry_msgs::msg::PoseStamped pose;
        
        // Use system time for RViz synchronization
        pose.header.stamp = this->get_clock()->now();
        pose.header.frame_id = "map";

        // Convert PX4 NED (North, East, Down) to RViz2 ENU (East, North, Up)
        pose.pose.position.x = msg->y;   // East
        pose.pose.position.y = msg->x;   // North
        pose.pose.position.z = -msg->z;  // Up

        // Simple quaternion facing forward (just path points)
        pose.pose.orientation.w = 1.0;
        pose.pose.orientation.x = 0.0;
        pose.pose.orientation.y = 0.0;
        pose.pose.orientation.z = 0.0;

        path_msg_.poses.push_back(pose);
        path_msg_.header.stamp = pose.header.stamp;
        
        // Prevent memory overflow for long flights
        if(path_msg_.poses.size() > 5000) {
            path_msg_.poses.erase(path_msg_.poses.begin());
        }

        path_pub_->publish(path_msg_);
    }

    rclcpp::Subscription<px4_msgs::msg::VehicleLocalPosition>::SharedPtr local_pos_sub_;
    rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr path_pub_;
    nav_msgs::msg::Path path_msg_;
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<RvizPathPublisher>());
    rclcpp::shutdown();
    return 0;
}
