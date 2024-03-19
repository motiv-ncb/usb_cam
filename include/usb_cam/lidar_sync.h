#pragma once
#include <thread>
#include <chrono>

#include "ros/ros.h"
#include "sensor_msgs/PointCloud2.h"

namespace usb_cam {
    class LidarSync {
    public: 
        LidarSync(const std::string lidarTopic) : initialTime_(0.0), delay_(0.0)  { 
            lidarTopic_ = lidarTopic;

            times_.reserve(100);
        }

        void CaptureData(std::size_t messages_count) {
            ros::Rate r(1000);

            ros::Subscriber sub  = n_.subscribe(lidarTopic_, messages_count, &LidarSync::SubscriberCallback, this);
            while (times_.size() <= messages_count)
            {
                ros::spinOnce();
                r.sleep();
            }

            std::sort(times_.begin(), times_.end());

            initialTime_ = times_[0];

            delay_ = 0.0;
            for (size_t i = 1; i < times_.size(); i++)
            {
                delay_ += times_[i] - times_[i-1];
            }   
            delay_ /= times_.size() - 1;
            ROS_INFO("estimated delay: %.4f\n",delay_);
        }

        void SubscriberCallback(const sensor_msgs::PointCloud2& pcd_msg) {
            double pcd_time = pcd_msg.header.stamp.toSec();
            times_.push_back(pcd_time);
            ROS_INFO("Lidar time: %.4f, seq: %u", pcd_time, pcd_msg.header.seq);
        }

        double getSleepTime(double currentTime) {
            double framesFromInitial = (currentTime - initialTime_) / delay_;

            double intpart;
            double fractTime = modf(framesFromInitial, &intpart);
            
            return (1 - fractTime) * delay_;
        }

        void Test() {
            ROS_INFO("delay: %.4f\n",delay_);
            // ROS_INFO("fromInitialTime: %.4f\n", fromInitialTime);
            // ROS_INFO("framesFromInitial: %.4f\n", framesFromInitial);
            // ROS_INFO("fractTime: %.4f\n", fractTime);
            // ROS_INFO("sleepTime: %.4f\n", sleepTime);
            
            // ros::Duration(sleepTime).sleep();

            ros::Rate r(30); 
            for (size_t i = 0; i < 30; i++)
            { 
                double currentTime = ros::Time::now().toSec();
                if (i==0) {
                    double sleepTime = getSleepTime(currentTime);
                    ros::Duration(sleepTime).sleep();
                    // std::this_thread::sleep_for(std::chrono::milliseconds(static_cast<int>(sleepTime*1000)));
                }

                ROS_INFO("time: %.4f, times from initial: %.4f\n", currentTime, (currentTime - initialTime_)/delay_); 
                r.sleep();
            } 
        }
        

    protected:
        std::string lidarTopic_;
        std::vector<double> times_;
        ros::NodeHandle n_;
        double initialTime_;
        double delay_;
    };
}