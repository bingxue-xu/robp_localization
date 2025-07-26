#include "robot_localization_demo/odometry.hpp"
#include <cmath>
#include <tf2/utils.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>

namespace robot_localization_demo {

    TurtleOdometry::TurtleOdometry(
        double frequency,
        double error_vx_systematic, double error_vx_random, 
        double error_vyaw_systematic, double error_vyaw_random,
        bool visualize
    ):
        Node("turtle_odometry_system"),
        frequency_(frequency),
        visualize_(visualize),
        random_generator_{},
        random_distribution_vx_(error_vx_systematic, error_vx_random),
        random_distribution_wz_(error_vyaw_systematic, error_vyaw_random),
        current_pose_{},
        previous_pose_{},
        previous_time_(this->get_clock()->now()),
        pose_received_(false)
    {
        // Create subscription to turtle pose
        pose_subscription_ = this->create_subscription<turtlesim::msg::Pose>(
            "/turtle1/pose", 10,
            std::bind(&TurtleOdometry::poseCallback, this, std::placeholders::_1)
        );

        // Create publisher for odometry with covariance
        odometry_publisher_ = this->create_publisher<geometry_msgs::msg::TwistWithCovarianceStamped>(
            "/turtle1/sensors/twist", 10
        );

        // Set up timer for periodic publishing
        timer_ = this->create_wall_timer(
            std::chrono::milliseconds(static_cast<int>(1000.0 / frequency_)),
            std::bind(&TurtleOdometry::publishOdometry, this)
        );

        // Initialize visualization if requested
        if (visualize_) {
            spawnAndConfigureVisualizationTurtle();
        }

        RCLCPP_INFO(this->get_logger(), "TurtleOdometry initialized with frequency: %.2f Hz", frequency_);
    }

    void TurtleOdometry::poseCallback(const turtlesim::msg::Pose::SharedPtr msg) {
        current_pose_ = *msg;
        pose_received_ = true;
    }

    void TurtleOdometry::publishOdometry() {
        if (!pose_received_) {
            return; // No pose data yet
        }

        auto current_time = this->get_clock()->now();
        
        // Calculate time difference
        double dt = (current_time - previous_time_).seconds();
        
        if (dt <= 0.0) {
            return; // Avoid division by zero or negative time
        }

        // Calculate velocities
        double vx = (current_pose_.x - previous_pose_.x) / dt;
        double vy = (current_pose_.y - previous_pose_.y) / dt;
        
        // Calculate angular velocity (handle angle wrapping)
        double angular_diff = current_pose_.theta - previous_pose_.theta;
        
        // Normalize angle difference to [-pi, pi]
        while (angular_diff > M_PI) angular_diff -= 2.0 * M_PI;
        while (angular_diff < -M_PI) angular_diff += 2.0 * M_PI;
        
        double wz = angular_diff / dt;

        // Transform velocities to robot frame
        double cos_theta = std::cos(current_pose_.theta);
        double sin_theta = std::sin(current_pose_.theta);
        
        double vx_robot = cos_theta * vx + sin_theta * vy;
        // Note: vy_robot not needed for turtlesim (2D robot, no lateral movement)

        // Add systematic and random errors
        double vx_with_error = vx_robot + random_distribution_vx_(random_generator_);
        double wz_with_error = wz + random_distribution_wz_(random_generator_);

        // Create and publish odometry message
        auto odometry_msg = geometry_msgs::msg::TwistWithCovarianceStamped();
        odometry_msg.header.stamp = current_time;
        odometry_msg.header.frame_id = "base_link";
        
        // Set twist
        odometry_msg.twist.twist.linear.x = vx_with_error;
        odometry_msg.twist.twist.linear.y = 0.0; // Assume differential drive
        odometry_msg.twist.twist.angular.z = wz_with_error;

        // Set covariance (6x6 matrix, but we only care about x-linear and z-angular)
        std::fill(odometry_msg.twist.covariance.begin(), odometry_msg.twist.covariance.end(), 0.0);
        odometry_msg.twist.covariance[0] = 0.1;   // variance for vx
        odometry_msg.twist.covariance[35] = 0.05; // variance for wz

        odometry_publisher_->publish(odometry_msg);

        // Update visualization
        if (visualize_) {
            updateVisualization(vx_with_error, wz_with_error);
        }

        // Store current values for next iteration
        previous_time_ = current_time;
        previous_pose_ = current_pose_;
    }

    void TurtleOdometry::spawnAndConfigureVisualizationTurtle() {
        // Create service clients for turtle management
        spawn_client_ = this->create_client<turtlesim::srv::Spawn>("/spawn");
        set_pen_client_ = this->create_client<turtlesim::srv::SetPen>("/turtle_odometry/set_pen");
        
        // Create publisher for turtle commands
        turtle_cmd_publisher_ = this->create_publisher<geometry_msgs::msg::Twist>(
            "/turtle_odometry/cmd_vel", 10
        );

        // Wait for spawn service and use optimal async pattern
        if (!spawn_client_->wait_for_service(std::chrono::seconds(5))) {
            RCLCPP_ERROR(this->get_logger(), "Spawn service not available for odometry visualization");
            return;
        }

        auto spawn_request = std::make_shared<turtlesim::srv::Spawn::Request>();
        spawn_request->x = 5.5;
        spawn_request->y = 5.5;
        spawn_request->theta = 0.0;
        spawn_request->name = "turtle_odometry";  // Explicit descriptive name

        // Optimal async pattern with callback and error handling
        spawn_client_->async_send_request(spawn_request,
            [this](rclcpp::Client<turtlesim::srv::Spawn>::SharedFuture future) {
                try {
                    auto spawn_response = future.get();
                    RCLCPP_INFO(this->get_logger(), 
                               "Odometry visualization turtle '%s' spawned successfully", 
                               spawn_response->name.c_str());
                    
                    // Chain pen configuration after successful spawn
                    configurePenForVisualizationTurtle();
                    
                } catch (const std::exception& e) {
                    RCLCPP_ERROR(this->get_logger(), 
                                "Failed to spawn odometry visualization turtle: %s", e.what());
                }
            });
    }

    void TurtleOdometry::configurePenForVisualizationTurtle() {
        if (!set_pen_client_->wait_for_service(std::chrono::seconds(2))) {
            RCLCPP_ERROR(this->get_logger(), "Set pen service not available for odometry visualization");
            return;
        }

        auto pen_request = std::make_shared<turtlesim::srv::SetPen::Request>();
        pen_request->r = 200;     // Red color for odometry
        pen_request->g = 0;    
        pen_request->b = 0; 
        pen_request->width = 2;
        pen_request->off = false;

        set_pen_client_->async_send_request(pen_request,
            [this](rclcpp::Client<turtlesim::srv::SetPen>::SharedFuture future) {
                try {
                    future.get();
                    RCLCPP_INFO(this->get_logger(), 
                               "Odometry visualization pen configured (red)");
                } catch (const std::exception& e) {
                    RCLCPP_ERROR(this->get_logger(), 
                                "Failed to configure pen for odometry visualization: %s", e.what());
                }
            });
    }

    void TurtleOdometry::updateVisualization(double vx, double wz) {
        if (!turtle_cmd_publisher_) {
            return;
        }

        auto cmd_msg = geometry_msgs::msg::Twist();
        cmd_msg.linear.x = vx;
        cmd_msg.angular.z = wz;
        
        turtle_cmd_publisher_->publish(cmd_msg);
    }

} 
