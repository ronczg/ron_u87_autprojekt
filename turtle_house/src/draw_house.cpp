#include <chrono>
#include <memory>
#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include <cmath> 

using namespace std::chrono_literals;

class DrawHouse : public rclcpp::Node
{
public:
    DrawHouse() : Node("draw_house")
    {
        publisher_ = this->create_publisher<geometry_msgs::msg::Twist>("turtle1/cmd_vel", 10);
        timer_ = this->create_wall_timer(500ms, std::bind(&DrawHouse::draw_house, this));
        cmd_ = geometry_msgs::msg::Twist();
        step_ = 0;
    }

private:
    void move_forward(double distance, double time)
    {
        geometry_msgs::msg::Twist msg;
        msg.linear.x = distance;
        msg.angular.z = 0.0;
        publisher_->publish(msg);

        auto sleep_duration = std::chrono::duration_cast<std::chrono::nanoseconds>(std::chrono::duration<double>(time));
        rclcpp::sleep_for(sleep_duration);


        msg.linear.x = 0.0;
        publisher_->publish(msg);
    }

    void rotate(double angular_distance_degrees, double time)
    {
      
        double angular_distance_radians = angular_distance_degrees * (M_PI / 180.0);

        geometry_msgs::msg::Twist msg;
        msg.linear.x = 0.0;
        msg.angular.z = angular_distance_radians;
        publisher_->publish(msg);

        auto sleep_duration = std::chrono::duration_cast<std::chrono::nanoseconds>(std::chrono::duration<double>(time));
        rclcpp::sleep_for(sleep_duration);

        msg.angular.z = 0.0;
        publisher_->publish(msg);
    }

    void draw_house()
    {
        switch (step_)
        {
        case 0: move_forward(1.0, 2.0); break;     // Move forward 2 seconds
        case 1: rotate(90 , 1.0); break;          // Rotate 90 degrees (90  rad)
        case 2: move_forward(1.0, 2.0); break;     // Move forward 2 seconds
        case 3: rotate(90 , 1.0); break;          // Rotate 90 degrees
        case 4: move_forward(1.0, 2.0); break;     // Move forward 2 seconds
        case 5: rotate(90 , 1.0); break;          // Rotate 90 degrees
        case 6: move_forward(1.0, 2.0); break;     // Move forward 2 seconds
        case 7: rotate(90 , 1.0); break;          // Rotate 90 degrees
        case 8: move_forward(1.0, 2.0); break;     // Complete square base
        case 9: rotate(90 , 1.0); break;          // Rotate to start triangle
        case 10: move_forward(1.0, 2.0); break;    // First side of triangle
        case 11: rotate(45, 1.0); break;         // Rotate 45 degrees
        case 12: move_forward(1.0, 1.8); break;    // Second side of triangle
        case 13: rotate(120, 1.0); break;         // Rotate 120 degrees
        case 14: move_forward(1.0, 1.8); break;    // Complete triangle roof
        default:
            cmd_.linear.x = 0.0;
            cmd_.angular.z = 0.0;
           rclcpp::Rate rate(1);  // 10 Hz-es frekvencia
            while (rclcpp::ok()) {
            publisher_->publish(cmd_);
            rate.sleep();
}

            return;  // Stop movement
        }
        step_++;
    }

    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr publisher_;
    rclcpp::TimerBase::SharedPtr timer_;
    geometry_msgs::msg::Twist cmd_;
    int step_;
};

int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<DrawHouse>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
