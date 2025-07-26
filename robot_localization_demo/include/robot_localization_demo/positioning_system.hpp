#ifndef ROBOT_LOCALIZATION_DEMO_POSITIONING_SYSTEM_HPP_
#define ROBOT_LOCALIZATION_DEMO_POSITIONING_SYSTEM_HPP_

#include <random>
#include <memory>
#include <string>

#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/pose_with_covariance_stamped.hpp>
#include <turtlesim/msg/pose.hpp>
#include <turtlesim/srv/spawn.hpp>
#include <turtlesim/srv/set_pen.hpp>
#include <turtlesim/srv/teleport_absolute.hpp>

namespace robot_localization_demo {
    class TurtlePositioningSystem : public rclcpp::Node {
        public:
            explicit TurtlePositioningSystem(
                double frequency = 1.0,
                double error_x_systematic = 0.0,
                double error_x_random = 0.2,
                double error_y_systematic = 0.0,
                double error_y_random = 0.2,
                double error_yaw_systematic = 0.0,
                double error_yaw_random = 0.2,
                bool visualize = false
            );
            ~TurtlePositioningSystem();

        private:
            // ROS2 node handle
            rclcpp::Subscription<turtlesim::msg::Pose>::SharedPtr turtle_pose_subscriber_;
            rclcpp::Publisher<geometry_msgs::msg::PoseWithCovarianceStamped>::SharedPtr turtle_pose_publisher_;
            rclcpp::TimerBase::SharedPtr timer_;

            // Service clients
            rclcpp::Client<turtlesim::srv::Spawn>::SharedPtr spawn_client_;
            rclcpp::Client<turtlesim::srv::SetPen>::SharedPtr set_pen_client_;
            rclcpp::Client<turtlesim::srv::TeleportAbsolute>::SharedPtr teleport_client_;

            // parameters
            double frequency_;
            double error_x_systematic_, error_y_systematic_, error_yaw_systematic_;  
            std::default_random_engine random_generator_;
            std::normal_distribution<double> random_distribution_x_;
            std::normal_distribution<double> random_distribution_y_;
            std::normal_distribution<double> random_distribution_yaw_;

            // state variables
            bool visualize_;
            std::string visualization_turtle_name_;
            uint32_t frame_sequence_;
            turtlesim::msg::Pose cached_pose_;
            rclcpp::Time cached_pose_timestamp_;
            
            // Callback methods
            void turtlePoseCallback(const turtlesim::msg::Pose::SharedPtr message);
            void publishMeasurement();

            // visualization helpers
            void configurePenForVisualizationTurtle();
            inline bool isVisualizationRequested() const { return visualize_; }
            inline bool isVisualizationTurtleAvailable() const {
                return !visualization_turtle_name_.empty();
            }

            void spawnAndConfigureVisualizationTurtle(const turtlesim::msg::Pose & initialPose);
            void moveVisualizationTurtle(const turtlesim::msg::Pose & measurement);
    };
}

#endif 
