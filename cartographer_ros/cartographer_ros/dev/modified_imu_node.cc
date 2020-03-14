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
#include "sensor_msgs/Imu.h"
#include "nav_msgs/Odometry.h"
#include "gflags/gflags.h"
#include "glog/logging.h"

DEFINE_string(input, "", "Robot name");

constexpr int kSubscriberQueueSize = 150;
constexpr char kImuInTopic[] = "/vn100/imu_wori_wcov";
constexpr char kWOInTopic[] = "/hero/wio_ekf/odom";
constexpr char kImuOutTopic[] = "imu";

class FakeImu {
    public:
    std::string ImuInTopic; 
    std::string ImuOutTopic; 
    std::string WioInTopic;

    ::ros::NodeHandle node_handle;
    ::ros::Publisher ImuPub;
    ::ros::Subscriber ImuSub;
    ::ros::Subscriber WioSub;

    nav_msgs::Odometry wo_;
    sensor_msgs::Imu imu_;

    FakeImu(const std::string& robot_name) {
        ImuInTopic = robot_name + kImuInTopic;
        ImuOutTopic = kImuOutTopic;
        WioInTopic = robot_name + kWOInTopic;

        ImuPub = node_handle.advertise<sensor_msgs::Imu>(ImuOutTopic, 10);
        WioSub = node_handle.subscribe(WioInTopic, kSubscriberQueueSize, &FakeImu::WheelOdometryCallback, this);
        ImuSub = node_handle.subscribe(ImuInTopic, kSubscriberQueueSize, &FakeImu::ImuCallback, this);
    }
    void WheelOdometryCallback(const nav_msgs::Odometry::ConstPtr& wo_in) {
        wo_ = *wo_in;
    }
    void ImuCallback(const sensor_msgs::Imu::ConstPtr& imu_in) {
        imu_ = *imu_in;
        if (wo_.header.stamp >= imu_in->header.stamp) {
            // If wo can be used
            if (wo_.header.seq > 0) {
                imu_.orientation.x = wo_.pose.pose.orientation.x;
                imu_.orientation.y = wo_.pose.pose.orientation.y;
                imu_.orientation.z = wo_.pose.pose.orientation.z;
                imu_.orientation.w = wo_.pose.pose.orientation.w;
                // Wo used then disable so that not used again
                wo_.header.seq = 0;
            }
        }
        ImuPub.publish(imu_);
    }
};

int main(int argc, char** argv) {
  ::ros::init(argc, argv, "modified_imu_node");
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
