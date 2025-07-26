#include "robot_localization_demo/positioning_system.hpp"
#include <tf2/utils.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>

namespace robot_localization_demo {
    TurtlePositioningSystem::TurtlePositioningSystem(
        double frequency,
        double error_x_systematic, double error_x_random,
        double error_y_systematic, double error_y_random,
        double error_yaw_systematic, double error_yaw_random,
        bool visualize
    ):
        Node("turtle_positioning_system"),
        frequency_(frequency),
        error_x_systematic_(error_x_systematic), error_y_systematic_(error_y_systematic), error_yaw_systematic_(error_yaw_systematic),  
        random_generator_{},
        random_distribution_x_(0.0, error_x_random),
        random_distribution_y_(0.0, error_y_random),
        random_distribution_yaw_(0.0, error_yaw_random),
        visualize_(visualize),
        visualization_turtle_name_(""),
        frame_sequence_(0),
        cached_pose_timestamp_(0, 0, RCL_ROS_TIME)
        {
            turtle_pose_subscriber_ = this->create_subscription<turtlesim::msg::Pose>(
                "turtle1/pose", 16,
                std::bind(&TurtlePositioningSystem::turtlePoseCallback, this, std::placeholders::_1));

            turtle_pose_publisher_ = this ->create_publisher<geometry_msgs::msg::PoseWithCovarianceStamped>(
                "turtle1/sensors/pose", 16);

            // Set up timer for periodic publishing
            timer_ = this->create_wall_timer(
                std::chrono::milliseconds(static_cast<int>(1000.0 / frequency_)),
                std::bind(&TurtlePositioningSystem::publishMeasurement, this)
            );

            if (visualize_) {
                spawn_client_ = this->create_client<turtlesim::srv::Spawn>("spawn");
            }

            RCLCPP_INFO(this->get_logger(), "TurtlePositioningSystem initialized with frequency: %.2f Hz", frequency_);
        }
    TurtlePositioningSystem::~TurtlePositioningSystem() = default;

    void TurtlePositioningSystem::publishMeasurement() {
        if (cached_pose_timestamp_.seconds() == 0) {
            return; // No pose data yet
        }

        auto measurement = cached_pose_;
        measurement.x += error_x_systematic_ + random_distribution_x_(random_generator_);   
        measurement.y += error_y_systematic_ + random_distribution_y_(random_generator_);   
        measurement.theta += error_yaw_systematic_ + random_distribution_yaw_(random_generator_);   

        geometry_msgs::msg::PoseWithCovarianceStamped current_pose;
        current_pose.header.stamp = this->now();
        current_pose.header.frame_id = "map";
        current_pose.pose.pose.position.x = measurement.x;
        current_pose.pose.pose.position.y = measurement.y;
        current_pose.pose.pose.position.z = 0.0;

        tf2::Quaternion q;
        q.setRPY(0.0, 0.0, measurement.theta);
        current_pose.pose.pose.orientation = tf2::toMsg(q);

        // Set covariance matrix (uncertainty information) 
        double x_variance = std::pow(random_distribution_x_.stddev(), 2);
        double y_variance = std::pow(random_distribution_y_.stddev(), 2);  
        double yaw_variance = std::pow(random_distribution_yaw_.stddev(), 2);
        
        current_pose.pose.covariance = {
            x_variance, 0., 0., 0., 0., 0.,
            0., y_variance, 0., 0., 0., 0.,
            0., 0., 0., 0., 0., 0.,
            0., 0., 0., 0., 0., 0.,
            0., 0., 0., 0., 0., 0.,
            0., 0., 0., 0., 0., yaw_variance
        };

        turtle_pose_publisher_->publish(current_pose);

        if(isVisualizationRequested() && isVisualizationTurtleAvailable()) {
            moveVisualizationTurtle(measurement);
        }
    }

    void TurtlePositioningSystem::turtlePoseCallback(const turtlesim::msg::Pose::SharedPtr message) {
        cached_pose_timestamp_ = this->now();
        cached_pose_ = *message;

        if (isVisualizationRequested() && !isVisualizationTurtleAvailable()) {
            spawnAndConfigureVisualizationTurtle(*message);
        }
    }

    void TurtlePositioningSystem::spawnAndConfigureVisualizationTurtle(const turtlesim::msg::Pose & initialPose) {
        if (isVisualizationRequested() && !isVisualizationTurtleAvailable()) {
            if (!spawn_client_->wait_for_service(std::chrono::seconds(5))){
                RCLCPP_ERROR(this->get_logger(), "Spawn service not available");
                return;
            }

            auto spawnRequest = std::make_shared<turtlesim::srv::Spawn::Request>();
            spawnRequest->x = initialPose.x;
            spawnRequest->y = initialPose.y;
            spawnRequest->theta = initialPose.theta;
            spawnRequest->name = "turtle_position";  

            // Use async callback to avoid executor deadlock
            auto spawnFuture = spawn_client_->async_send_request(spawnRequest,
                [this](rclcpp::Client<turtlesim::srv::Spawn>::SharedFuture future) {
                    try {
                        auto spawnResponse = future.get();
                        visualization_turtle_name_ = spawnResponse->name;
                        configurePenForVisualizationTurtle();
                        
                        RCLCPP_INFO(this->get_logger(), 
                                   "Position sensor visualization turtle '%s' spawned successfully (blue pen)", 
                                   visualization_turtle_name_.c_str());
                    } catch (const std::exception& e) {
                        RCLCPP_ERROR(this->get_logger(), "Failed to spawn visualization turtle: %s", e.what());
                    }
                });
        }
    }

    void TurtlePositioningSystem::configurePenForVisualizationTurtle() {
        if (!isVisualizationTurtleAvailable()) return;

        std::string penService = visualization_turtle_name_ + "/set_pen";
        set_pen_client_ = this->create_client<turtlesim::srv::SetPen>(penService);

        if (!set_pen_client_->wait_for_service(std::chrono::seconds(5))) {
            RCLCPP_ERROR(this->get_logger(), "Set pen service not available");
            return;
        }

        auto penRequest = std::make_shared<turtlesim::srv::SetPen::Request>();
        penRequest->r = 0;
        penRequest->g = 0;
        penRequest->b = 255;  
        penRequest->width = 2;
        penRequest->off = false;

        set_pen_client_->async_send_request(penRequest);
    }

    void TurtlePositioningSystem::moveVisualizationTurtle(const turtlesim::msg::Pose & measurement) {
        if (isVisualizationTurtleAvailable() && isVisualizationRequested()) {
            std::string teleportService = visualization_turtle_name_ + "/teleport_absolute";
            teleport_client_ = this->create_client<turtlesim::srv::TeleportAbsolute>(teleportService);

            auto teleportRequest = std::make_shared<turtlesim::srv::TeleportAbsolute::Request>();
            teleportRequest->x = measurement.x;
            teleportRequest->y = measurement.y;
            teleportRequest->theta = measurement.theta;

            teleport_client_->async_send_request(teleportRequest);
        }
        
    }
}
