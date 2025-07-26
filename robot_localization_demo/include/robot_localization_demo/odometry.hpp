#ifndef ROBOT_LOCALIZATION_DEMO_ODOMETRY_HPP_
#define ROBOT_LOCALIZATION_DEMO_ODOMETRY_HPP_

#include <random>
#include <memory>
#include <string>

#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/twist_with_covariance_stamped.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <turtlesim/msg/pose.hpp>
#include <turtlesim/srv/spawn.hpp>
#include <turtlesim/srv/set_pen.hpp>

namespace robot_localization_demo {
    class TurtleOdometry : public rclcpp::Node {
        public:
            explicit TurtleOdometry(
                double frequency = 10.0,
                double error_vx_systematic = 0.0,
                double error_vx_random = 0.05,
                double error_vyaw_systematic = 0.0,
                double error_vyaw_random = 0.02,
                bool visualize = false
            );

        private:
            // ROS2 components
            rclcpp::Subscription<turtlesim::msg::Pose>::SharedPtr pose_subscription_;
            rclcpp::Publisher<geometry_msgs::msg::TwistWithCovarianceStamped>::SharedPtr odometry_publisher_;
            rclcpp::TimerBase::SharedPtr timer_;

            // Visualization components
            rclcpp::Client<turtlesim::srv::Spawn>::SharedPtr spawn_client_;
            rclcpp::Client<turtlesim::srv::SetPen>::SharedPtr set_pen_client_;
            rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr turtle_cmd_publisher_;

            // Parameters
            double frequency_;
            bool visualize_;
            
            // Error modeling
            std::default_random_engine random_generator_;
            std::normal_distribution<double> random_distribution_vx_;
            std::normal_distribution<double> random_distribution_wz_;

            // State tracking
            turtlesim::msg::Pose current_pose_;
            turtlesim::msg::Pose previous_pose_;
            rclcpp::Time previous_time_;
            bool pose_received_;

            // Callback functions
            void poseCallback(const turtlesim::msg::Pose::SharedPtr msg);
            void publishOdometry();
            
            // Visualization functions
            void spawnAndConfigureVisualizationTurtle();
            void configurePenForVisualizationTurtle();
            void updateVisualization(double vx, double wz);
    };
}

#endif 
