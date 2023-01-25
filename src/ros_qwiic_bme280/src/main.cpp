/*
MIT License

Copyright (c) 2022 Lou Amadio

Permission is hereby granted, free of charge, to any person obtaining a copy
of this software and associated documentation files (the "Software"), to deal
in the Software without restriction, including without limitation the rights
to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
copies of the Software, and to permit persons to whom the Software is
furnished to do so, subject to the following conditions:

The above copyright notice and this permission notice shall be included in all
copies or substantial portions of the Software.

THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
SOFTWARE.
*/

#include <memory>
#include <chrono>
#include <math.h>
#include "rclcpp/rclcpp.hpp"

#include "Arduino.h"
#include "Wire.h"
#include "SparkFunBME280.h"
#include "sensor_msgs/msg/temperature.hpp"
#include "sensor_msgs/msg/relative_humidity.hpp"

using namespace std::chrono_literals;
using std::placeholders::_1;

class I2CPublisher : public rclcpp::Node
{
  BME280 mySensor;

public:
  I2CPublisher() : Node("bme280")
  {
  }

  void initialize()
  {
    get_parameter_or<uint8_t>("id", _id, 0x77);
    get_parameter_or<double>("poll", _poll, 500.0);
    get_parameter_or<std::string>("frame_id", _frameId, "temp");

    Wire.begin();
    if (mySensor.beginI2C() == false)
    {
      RCLCPP_ERROR(rclcpp::get_logger("bme280"), "Could not initialize bme280 on 0x%x", _id);
      return;
    }
    // Create polling timer
    _timer = this->create_wall_timer(std::chrono::duration<double, std::milli>(_poll), std::bind(&I2CPublisher::timer_callback, this));
    // Create publishers
    _tempPub = this->create_publisher<sensor_msgs::msg::Temperature>("temp", 10);
    _humPub = this->create_publisher<sensor_msgs::msg::RelativeHumidity>("hum", 10);
  }

private:
  void timer_callback()
  {
    // Poll sensor for new data
    sensor_msgs::msg::Temperature tempMsg;
    sensor_msgs::msg::RelativeHumidity humMsg;

    float c = mySensor.readTempC();

    tempMsg.header.frame_id = _frameId;
    tempMsg.header.stamp = rclcpp::Clock().now();
    tempMsg.temperature = c;

    float h = mySensor.readFloatHumidity();

    humMsg.header.frame_id = _frameId;
    humMsg.header.stamp = rclcpp::Clock().now();
    humMsg.relative_humidity = h;

    _tempPub->publish(tempMsg);
    _humPub->publish(humMsg);
  }
  rclcpp::TimerBase::SharedPtr _timer;

  rclcpp::Publisher<sensor_msgs::msg::Temperature>::SharedPtr _tempPub;
  rclcpp::Publisher<sensor_msgs::msg::RelativeHumidity>::SharedPtr _humPub;
  uint8_t _id;
  double _poll;
  std::string _frameId;
};

int main(int argc, char *argv[])
{
  rclcpp::init(argc, argv);

  auto node = std::make_shared<I2CPublisher>();
  node->declare_parameter("id", rclcpp::PARAMETER_INTEGER);
  node->declare_parameter("poll", rclcpp::PARAMETER_DOUBLE);
  node->declare_parameter("frame_id", rclcpp::PARAMETER_STRING);

  node->initialize();

  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}