#include <exception>
#include <memory>

#include "fss_manager/manager_node.hpp"

int main(int argc, char* argv[]) {
    rclcpp::init(argc, argv);
    try {
        rclcpp::spin(std::make_shared<fss_manager::ManagerNode>());
    } catch (const std::exception& error) {
        RCLCPP_FATAL(rclcpp::get_logger("manager_node"), "%s", error.what());
        rclcpp::shutdown();
        return 1;
    }
    rclcpp::shutdown();
    return 0;
}
