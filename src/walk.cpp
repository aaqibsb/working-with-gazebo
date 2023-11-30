// Copyright(c) 2022 Aaqib Barodawala
//
// Permission is hereby granted,
// free of charge, to any person obtaining a copy of this software and
// associated documentation files(the "Software"), to deal in the Software
// without restriction, including without limitation the rights to use, copy,
// modify, merge, publish, distribute, sublicense, and / or sell copies of the
// Software, and to permit persons to whom the Software is furnished to do so,
// subject to the following conditions :
//
// The above copyright notice and this permission notice shall be included in
// all copies or substantial portions of the Software.
//
// THE SOFTWARE IS PROVIDED "AS IS",
// WITHOUT WARRANTY OF ANY KIND, EXPRESS OR IMPLIED, INCLUDING BUT NOT LIMITED
// TO THE WARRANTIES OF MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE AND
// NONINFRINGEMENT.IN NO EVENT SHALL THE AUTHORS OR COPYRIGHT HOLDERS BE LIABLE
// FOR ANY CLAIM, DAMAGES OR OTHER LIABILITY, WHETHER IN AN ACTION OF CONTRACT,
// TORT OR OTHERWISE, ARISING FROM, OUT OF OR IN CONNECTION WITH THE SOFTWARE OR
// THE USE OR OTHER DEALINGS IN THE SOFTWARE.

/**
 * @file walk.cpp
 * @author Aaqib Barodawala (aaqib.barodawala@gmail.com)
 * @brief Simple Walker algorithm for TurtleBot3
 * @version 0.1
 * @date 2022-29-06
 *
 * @copyright Copyright (c) 2022
 *
 */
#include <geometry_msgs/msg/twist.hpp>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/laser_scan.hpp>

using std::placeholders::_1;
using namespace std::chrono_literals;

using LASER = sensor_msgs::msg::LaserScan;
using TWIST = geometry_msgs::msg::Twist;

/**
 * @brief A simple class to implement walker algorithm for turtlebot3.
 * It uses the laser scan to get depth and move the robot accordingly.
 *
 */
class Walk : public rclcpp::Node {
 public:
  /**
   * @brief Construct a Walk class instance
   *
   */
  Walk() : Node("walk") {
    auto cb = std::bind(&Walk::laserData_cb, this, _1);
    laser_data_sub = this->create_subscription<LASER>("scan", 10, cb);
    pub_vel = this->create_publisher<TWIST>("cmd_vel", 10);
  }

 private:
  /**
   * @brief Get the Laser Scan data
   *
   * @param scanData
   */
  void laserData_cb(const LASER& scanData) {
    if (scanData.header.stamp.sec == 0) {
      return;
    }

    auto laser_scan_data = scanData.ranges;
    for (int i = 330; i < 330 + 60; i++) {
      if (laser_scan_data[i % 360] < 0.8) {
        move(0.0, 0.1);
      } else {
        move(0.1, 0.0);
      }
    }
  }

  /**
   * @brief Publishes velocity commands as per the distance (0.8)
   * read by the laser scan data
   * @param x_vel
   * @param z_vel
   */
  void move(float x_vel, float z_vel) {
    auto vel_msg = TWIST();
    vel_msg.linear.x = x_vel;
    vel_msg.angular.z = -z_vel;
    pub_vel->publish(vel_msg);
  }

  // Declaring private variables
  rclcpp::Subscription<LASER>::SharedPtr laser_data_sub;
  rclcpp::Publisher<TWIST>::SharedPtr pub_vel;
  rclcpp::TimerBase::SharedPtr timer;
  LASER last_scan_data;
};

int main(int argc, char** argv) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<Walk>());
  rclcpp::shutdown();
  return 0;
}
