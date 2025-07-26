#include <rclcpp/rclcpp.hpp>                      
#include <iostream>                               
#include "robot_localization_demo/positioning_system.hpp"   

int main(int argc, char ** argv) {
    rclcpp::init(argc, argv);

    double frequency = 1.0;
    double error_x_systematic = 0.0;
    double error_x_random = 0.2;
    double error_y_systematic = 0.0;
    double error_y_random = 0.2;
    double error_yaw_systematic = 0.0;
    double error_yaw_random = 0.2;
    bool visualize = false;

    for (int i = 1; i < argc; i++) {
        std::string arg = argv[i];

        if ((arg == "-f" || arg == "--frequency") && i + 1 < argc) {
            frequency = std::stod(argv[++i]);
        } else if (arg == "-v" || arg == "--visualize") {
            visualize = true;
        } else if ((arg == "-x" || arg == "--error-x-random") && i + 1 < argc) {
            error_x_random = std::stod(argv[++i]);
        } else if ((arg == "-y" || arg == "--error-y-random") && i + 1 < argc) {
            error_y_random = std::stod(argv[++i]);
        } else if ((arg == "-t" || arg == "--error-yaw-random") && i + 1 < argc) {
            error_yaw_random = std::stod(argv[++i]);
        } else if (arg == "--error-x-systematic" && i + 1 < argc) {
            error_x_systematic = std::stod(argv[++i]);
        } else if (arg == "--error-y-systematic" && i + 1 < argc) {
            error_y_systematic = std::stod(argv[++i]);
        } else if (arg == "--error-yaw-systematic" && i + 1 < argc) {
            error_yaw_systematic = std::stod(argv[++i]);
        } else if (arg == "--help" || arg == "-h") {
            std::cout << "Usage: " << argv[0] << " [options]\n";
            std::cout << "Options:\n";
            std::cout << "  -f, --frequency <Hz>            Set measurement frequency\n";
            std::cout << "  -x, --error-x-random <value>    Set random error on X position\n";
            std::cout << "  -y, --error-y-random <value>    Set random error on Y position\n";
            std::cout << "  -t, --error-yaw-random <value>  Set random error on yaw\n";
            std::cout << "  -v, --visualize                 Enable visualization\n";
            std::cout << "  --error-x-systematic <value>    Set systematic error on X position\n";
            std::cout << "  --error-y-systematic <value>    Set systematic error on Y position\n";
            std::cout << "  --error-yaw-systematic <value>  Set systematic error on yaw\n";
            return EXIT_SUCCESS;
        }
    }

    auto positioning_node = std::make_shared<robot_localization_demo::TurtlePositioningSystem>(
        frequency, error_x_systematic, error_x_random,
        error_y_systematic, error_y_random,
        error_yaw_systematic, error_yaw_random, visualize
    );

    RCLCPP_INFO(positioning_node->get_logger(), "Starting Turtle Positioning System Node...");
    RCLCPP_INFO(positioning_node->get_logger(), "Parameters: "
        "Frequency: %.2f Hz, "
        "Error X Systematic: %.2f, "
        "Error X Random: %.2f, "
        "Error Y Systematic: %.2f, "
        "Error Y Random: %.2f, "
        "Error Yaw Systematic: %.2f, "
        "Error Yaw Random: %.2f\n",
        frequency, error_x_systematic, error_x_random,
        error_y_systematic, error_y_random,
        error_yaw_systematic, error_yaw_random
    );

    rclcpp::spin(positioning_node);
    rclcpp::shutdown();
    return EXIT_SUCCESS;
}
