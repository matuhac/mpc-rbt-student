#include <fcntl.h>
#include <unistd.h>

#include <chrono>
#include <functional>

#include "KeyboardControl.hpp"

using namespace std::chrono_literals;

KeyboardControlNode::KeyboardControlNode(): rclcpp::Node("keyboard_control_node") 
{

//Parametry
this->declare_parameter("ForwardSpeed", 0.5);
this->declare_parameter("BackwardSpeed", -0.5);
this->declare_parameter("LeftRotation", -0.5);
this->declare_parameter("RightRotation", 0.5);

// Set terminal settings to non-blocking
tcgetattr(STDIN_FILENO, &old_termios_);
struct termios new_termios = old_termios_;
new_termios.c_lflag &= ~(ICANON | ECHO);
tcsetattr(STDIN_FILENO, TCSANOW, &new_termios);

fcntl(STDIN_FILENO, F_SETFL, O_NONBLOCK);

//vytvoreni casovace pro callback
timer_ = this->create_wall_timer(10ms, std::bind(&KeyboardControlNode::timerCallback, this));
//vytvorit publisher?? neni to ten twist v callbacku?
twist_publisher_ = this->create_publisher<geometry_msgs::msg::Twist>("cmd_vel", 10);
								//    topic name, queue size

RCLCPP_INFO(this->get_logger(), "Use Arrow Keys to control the robot. Press 'ctrl+c' to quit.");

}

KeyboardControlNode::~KeyboardControlNode() {
    tcsetattr(STDIN_FILENO, TCSANOW, &old_termios_);
}

void KeyboardControlNode::timerCallback() {
    //RCLCPP_INFO(this->get_logger(), "Callback Called");

    geometry_msgs::msg::Twist twist{}; //publisher??
    char c;

    fd_set readfds;
    struct timeval timeout;
    FD_ZERO(&readfds);
    FD_SET(STDIN_FILENO, &readfds);

    timeout.tv_sec = 0;
    timeout.tv_usec = 0;

    int retval = select(STDIN_FILENO + 1, &readfds, nullptr, nullptr, &timeout);

    if (retval > 0 && FD_ISSET(STDIN_FILENO, &readfds)) {
        if (read(STDIN_FILENO, &c, 1) == 1) {
            if (c == '\033') { // ESC sequence (arrow keys)
                char seq[2];
                if (read(STDIN_FILENO, &seq, 2) != 2)
                    return;

                if (seq[0] == '[') {
                    switch (seq[1]) {
                        case 'A':
                            twist.linear.x = this->get_parameter("ForwardSpeed").as_double();  // up arrow
                            //RCLCPP_INFO(this->get_logger(), "Up arrow");
                            break;
                        case 'B':
                            twist.linear.x = this->get_parameter("BackwardSpeed").as_double(); // down arrow
                            //RCLCPP_INFO(this->get_logger(), "Down arrow");
                            break;
                        case 'C':
                            twist.angular.z = this->get_parameter("LeftRotation").as_double(); // right arrow
                            //RCLCPP_INFO(this->get_logger(), "Right arrow");
                            break;
                        case 'D':
                            twist.angular.z = this->get_parameter("RightRotation").as_double();  // left arrow
                            //RCLCPP_INFO(this->get_logger(), "Left arrow");
                            break;
                    }
                }
            }

            twist_publisher_->publish(twist);
        }
    }
    // else no data was available, do nothing
}
