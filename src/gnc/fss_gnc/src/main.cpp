#include <exception>
#include <rclcpp/rclcpp.hpp>

#include "fss_gnc/gnc_node.hpp"

int main(int argc, char** argv) {
    rclcpp::init(argc, argv);
    int status = 0;
    try {
        rclcpp::spin(fss::gnc::make_gnc_node());
    } catch (const std::exception& error) {
        RCLCPP_FATAL(rclcpp::get_logger("gnc_node"), "GNC startup/runtime failure: %s", error.what());
        status = 1;
    }
    rclcpp::shutdown();
    return status;
}
