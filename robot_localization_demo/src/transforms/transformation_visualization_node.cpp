#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <tf2_ros/transform_listener.h>
#include <tf2_ros/buffer.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <turtlesim/srv/spawn.hpp>
#include <turtlesim/srv/set_pen.hpp>
#include <turtlesim/srv/teleport_absolute.hpp>
#include <tf2/utils.h>

class TransformVisualization : public rclcpp::Node {
public:
  TransformVisualization()
      : Node("transformation_visualization_node")
  {
    tf_buffer_ = std::make_unique<tf2_ros::Buffer>(this->get_clock());
    tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);
    
    spawn_client_ = this->create_client<turtlesim::srv::Spawn>("spawn");
    set_pen_client_ = this->create_client<turtlesim::srv::SetPen>("turtle_ekf/set_pen");
    teleport_client_ = this->create_client<turtlesim::srv::TeleportAbsolute>("turtle_ekf/teleport_absolute");

    spawnAndConfigureVisualizationTurtle();
    timer_ = this->create_wall_timer(
        std::chrono::milliseconds(100),  // Update every 100 ms
        std::bind(&TransformVisualization::updateVisualization, this));

    RCLCPP_INFO(this->get_logger(), "Transformation Visualization Node started.");
}

private:
  std::unique_ptr<tf2_ros::Buffer> tf_buffer_;
  std::shared_ptr<tf2_ros::TransformListener> tf_listener_;
  rclcpp::Client<turtlesim::srv::Spawn>::SharedPtr spawn_client_;
  rclcpp::Client<turtlesim::srv::SetPen>::SharedPtr set_pen_client_;
  rclcpp::Client<turtlesim::srv::TeleportAbsolute>::SharedPtr teleport_client_;
  rclcpp::TimerBase::SharedPtr timer_;
  bool turtle_spawned_ = false;
  std::string visualization_turtle_name_ = "turtle_ekf";

  void spawnAndConfigureVisualizationTurtle() {
    if (!spawn_client_->wait_for_service(std::chrono::seconds(5))) {
        RCLCPP_ERROR(this->get_logger(), "Spawn service not available for EKF visualization");
        return;
    }

    auto spawn_request = std::make_shared<turtlesim::srv::Spawn::Request>();
    spawn_request->x = 0.0;
    spawn_request->y = 0.0;
    spawn_request->theta = 0.0;
    spawn_request->name = "turtle_ekf";  

    // Optimal async pattern with callback and error handling
    spawn_client_->async_send_request(spawn_request,
        [this](rclcpp::Client<turtlesim::srv::Spawn>::SharedFuture future) {
            try {
                auto spawn_response = future.get();
                turtle_spawned_ = true;
                
                RCLCPP_INFO(this->get_logger(), 
                           "EKF visualization turtle '%s' spawned successfully", 
                           spawn_response->name.c_str());
                
                // Chain pen configuration after successful spawn
                configurePenForVisualizationTurtle();
                
            } catch (const std::exception& e) {
                RCLCPP_ERROR(this->get_logger(), 
                            "Failed to spawn EKF visualization turtle: %s", e.what());
            }
        });
  }

  void configurePenForVisualizationTurtle() {
        if (!set_pen_client_->wait_for_service(std::chrono::seconds(5))) {
            RCLCPP_ERROR(this->get_logger(), "Set pen service not available for EKF visualization");
            return;
        }

        auto set_pen_request = std::make_shared<turtlesim::srv::SetPen::Request>();
        set_pen_request->r = 0;
        set_pen_request->g = 255;
        set_pen_request->b = 0;  // Green color for EKF
        set_pen_request->width = 3;
        set_pen_request->off = false;

        set_pen_client_->async_send_request(set_pen_request,
            [this](rclcpp::Client<turtlesim::srv::SetPen>::SharedFuture future) {
                try {
                    future.get();
                    RCLCPP_INFO(this->get_logger(), 
                               "EKF visualization pen configured (green)");
                } catch (const std::exception& e) {
                    RCLCPP_ERROR(this->get_logger(), 
                                "Failed to configure pen for EKF visualization: %s", e.what());
                }
            });
    }

    void updateVisualization() {
        if (!turtle_spawned_) {
            RCLCPP_WARN(this->get_logger(), "Visualization turtle not spawned yet.");
            return;
        }

        try {
            geometry_msgs::msg::TransformStamped transform;
            transform = tf_buffer_->lookupTransform("map", "base_link", tf2::TimePointZero);

            geometry_msgs::msg::PoseStamped pose_base_link, pose_map;
            pose_base_link.header.stamp = this->get_clock()->now();
            pose_base_link.header.frame_id = "base_link";
            pose_base_link.pose.position.x = 0.0;
            pose_base_link.pose.position.y = 0.0;
            pose_base_link.pose.position.z = 0.0;
            pose_base_link.pose.orientation.x = 0.0;
            pose_base_link.pose.orientation.y = 0.0;
            pose_base_link.pose.orientation.z = 0.0;
            pose_base_link.pose.orientation.w = 1.0;

            tf2::doTransform(pose_base_link, pose_map, transform);

            auto teleport_request = std::make_shared<turtlesim::srv::TeleportAbsolute::Request>();
            teleport_request->x = pose_map.pose.position.x;
            teleport_request->y = pose_map.pose.position.y;

            tf2::Quaternion q(
                pose_map.pose.orientation.x,
                pose_map.pose.orientation.y,
                pose_map.pose.orientation.z,
                pose_map.pose.orientation.w
            );
            teleport_request->theta = tf2::getYaw(q);
            teleport_client_->async_send_request(teleport_request);
        } catch (const tf2::TransformException &ex) {
            RCLCPP_ERROR(this->get_logger(), "Transform error: %s", ex.what());
        }
    }
};

int main(int argc, char ** argv) {
    rclcpp::init(argc, argv);

    auto node = std::make_shared<TransformVisualization>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
