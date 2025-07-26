#include <rclcpp/rclcpp.hpp>  
#include <iostream>
#include <robot_localization_demo/odometry.hpp>

int main(int argc, char ** argv) {
    rclcpp::init(argc, argv); // Initialize the ROS 2 system, sets up ROS2 infrastructure, connects to DDS network, prepares logging, etc

    // Default sensor parameters
    double frequency = 10.0;
    double error_vx_systematic = 0.0;
    double error_vx_random = 0.05;
    double error_vyaw_systematic = 0.0;
    double error_vyaw_random = 0.02;
    bool visualize = false;

    for (int i = 1; i < argc; i++) {
        std::string arg = argv[i];
        
        if ((arg == "-f" || arg == "--frequency") && i + 1 < argc) {
            frequency = std::stod(argv[++i]);
        } else if (arg == "-v" || arg == "--visualize") {
            visualize = true;
        } else if ((arg == "-X" || arg == "--error-vx-systematic") && i + 1 < argc) {
            error_vx_systematic = std::stod(argv[++i]);
        } else if ((arg == "-x" || arg == "--error-vx-random") && i + 1 < argc) {
            error_vx_random = std::stod(argv[++i]);
        } else if ((arg == "-t" || arg == "--error-vyaw-systematic") && i + 1 < argc) {
            error_vyaw_systematic = std::stod(argv[++i]);
        } else if ((arg == "-T" || arg == "--error-vyaw-random") && i + 1 < argc) {
            error_vyaw_random = std::stod(argv[++i]);
        } else if (arg == "--help" || arg == "-h") {
            std::cout << "Usage: " << argv[0] << " [options]\n";
            std::cout << "Options:\n";
            std::cout << "  -f, --frequency <Hz>            Set measurement frequency\n";
            std::cout << "  -v, --visualize                 Enable visualization\n";
            std::cout << "  -X, --error-vx-systematic <val> Set systematic error on X velocity\n";
            std::cout << "  -x, --error-vx-random <val>     Set random error on X velocity\n";
            std::cout << "  -t, --error-vyaw-systematic <val> Set systematic error on angular velocity\n";
            std::cout << "  -T, --error-vyaw-random <val>   Set random error on angular velocity\n";
            std::cout << "  --help, -h                      Show this help message\n";
            return EXIT_SUCCESS;
        }
    }

    auto odometry_node = std::make_shared<robot_localization_demo::TurtleOdometry>(
        frequency, error_vx_systematic, error_vx_random,
        error_vyaw_systematic, error_vyaw_random, visualize
    );

    RCLCPP_INFO(odometry_node->get_logger(), "Starting Turtle Odometry Node...");
    RCLCPP_INFO(odometry_node->get_logger(), "Parameters: "
        "Frequency: %.2f Hz, "
        "Error VX Systematic: %.2f, "
        "Error VX Random: %.2f, "
        "Error VYaw Systematic: %.2f, "
        "Error VYaw Random: %.2f, "
        "Visualize: %s",
        frequency,
        error_vx_systematic,
        error_vx_random,
        error_vyaw_systematic,
        error_vyaw_random,
        visualize ? "true" : "false"
    );

    rclcpp::spin(odometry_node);  // Run the node until shutdown
    rclcpp::shutdown();           // Cleanup and shutdown ROS 2
    return EXIT_SUCCESS;
}
