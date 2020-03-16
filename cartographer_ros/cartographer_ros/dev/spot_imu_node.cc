// /*
//  * Copyright 2016 The Cartographer Authors
//  *
//  * Licensed under the Apache License, Version 2.0 (the "License");
//  * you may not use this file except in compliance with the License.
//  * You may obtain a copy of the License at
//  *
//  *      http://www.apache.org/licenses/LICENSE-2.0
//  *
//  * Unless required by applicable law or agreed to in writing, software
//  * distributed under the License is distributed on an "AS IS" BASIS,
//  * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
//  * See the License for the specific language governing permissions and
//  * limitations under the License.
//  */

#include "ros/ros.h"
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include "sensor_msgs/Imu.h"
#include "nav_msgs/Odometry.h"
#include "gflags/gflags.h"
#include "glog/logging.h"
#include <cmath>

DEFINE_string(input, "", "Robot name");

constexpr int kSubscriberQueueSize = 150;
constexpr char kWOInTopic[] = "/visual_odom";
constexpr char kImuOutTopic[] = "imu";

class FakeImu {
    public:
    std::string ImuOutTopic; 
    std::string WioInTopic;

    ::ros::NodeHandle node_handle;
    ::ros::Publisher ImuPub;
    ::ros::Subscriber WioSub;

    nav_msgs::Odometry wo_;
    sensor_msgs::Imu imu_;

    double roll;
    double pitch;
    double yaw;
    std::string robot;
    ros::Time last_time;

    FakeImu(const std::string& robot_name) {
        ImuOutTopic = kImuOutTopic;
        WioInTopic = robot_name + kWOInTopic;

        robot = robot_name;
        ImuPub = node_handle.advertise<sensor_msgs::Imu>(ImuOutTopic, 10);
        WioSub = node_handle.subscribe(WioInTopic, kSubscriberQueueSize, &FakeImu::WheelOdometryCallback, this);
        last_time = ros::Time(0.0);
    }
    void WheelOdometryCallback(const nav_msgs::Odometry::ConstPtr& wo_in) {
        wo_ = *wo_in;

        // Get the Euler angles
        tf2::Quaternion quat;
        tf2::convert(wo_.pose.pose.orientation , quat);
        double roll_, pitch_, yaw_;
        tf2::Matrix3x3 orTmp(quat);
        orTmp.getRPY(roll_, pitch_, yaw_);
        // Fill frame
        imu_.header.frame_id = robot+"/base_link";
        // If wo can be used
        if (last_time.toSec() > 0) {
            double delta_t = (wo_.header.stamp - last_time).toSec();
            imu_.angular_velocity.x = (roll_ - roll)/delta_t;
            imu_.angular_velocity.y = (pitch_ - pitch)/delta_t;
            imu_.angular_velocity.z = (yaw_ - yaw)/delta_t;
        }
        // Fill orientation
        imu_.orientation.x = wo_.pose.pose.orientation.x;
        imu_.orientation.y = wo_.pose.pose.orientation.y;
        imu_.orientation.z = wo_.pose.pose.orientation.z;
        imu_.orientation.w = wo_.pose.pose.orientation.w;

        // Fill accelerations
        imu_.linear_acceleration.x = 9.8 * sin(pitch_);
        imu_.linear_acceleration.y = 9.8 * cos(pitch_) * sin(roll_);
        imu_.linear_acceleration.z = 9.8 * cos(pitch_) * cos(roll_);
        
        last_time = wo_.header.stamp;
        roll = roll_;
        pitch = pitch_;
        yaw = yaw_;

        ImuPub.publish(imu_);
    }
};

int main(int argc, char** argv) {
  ::ros::init(argc, argv, "spot_imu_node");
  google::InitGoogleLogging(argv[0]);
  google::SetUsageMessage(
      "\n\n"
      "Modify Imu topic to extract pose from VIO/WIO and velocities from "
      "Imu topic."
      );
  google::ParseCommandLineFlags(&argc, &argv, true);
  CHECK(!FLAGS_input.empty()) << "-input Robot name.";

  FakeImu imu(FLAGS_input);

  ::ros::spin();
  ::ros::shutdown();
}
