#include <iostream>
#include <unistd.h>

#include <ros/ros.h>
#include "std_msgs/Float64.h"

#include <algorithm>    // std::sort
#include <deque>

using namespace std;

deque<double> cap_memory;

int filter_median_size = 5;
int filter_mean_width = 3;
bool new_data = false;

void cap_callback(const std_msgs::Float64::ConstPtr& msg)
{
    cap_memory.push_front(msg->data);
    if(cap_memory.size()>filter_median_size)
    {
        cap_memory.pop_back();
    }
    new_data = true;
}

int main(int argc, char *argv[]){
    ros::init(argc, argv, "cap_filter_node");
    ros::NodeHandle n;

    // Parameters
    ros::NodeHandle n_private("~");
    const double frequency = n_private.param<double>("frequency", 2000);

    filter_median_size = n_private.param<int>("filter_median_size", 5);
    filter_mean_width = n_private.param<int>("filter_mean_width", 3);

    // Subscriber
    ros::Subscriber cap_sub = n.subscribe("/capRaw", 10, cap_callback);

    // Publisher
    ros::Publisher cap_pub = n.advertise<std_msgs::Float64>("capFiltered", 1);

    // Loop variables
    std_msgs::Float64 msg;

    ROS_INFO("[Filtre cap] Start Ok");
    ros::Rate loop_rate(frequency);
    while (ros::ok())
    {
        ros::spinOnce();
        if(!cap_memory.empty() && new_data)
        {
            /// ************** Compute cap ************** //
            /// MEDIAN + MEAN FILTER
            double cap;
            deque<double> cap_memory_tmp(cap_memory); // Make a copy
            sort(cap_memory_tmp.begin(), cap_memory_tmp.end()); // Sort to take median

            // Compute the mean with value centered to the median
            int n_mid = round(cap_memory_tmp.size()/2.0);
            double cap_mean = 0.0;
            int k=0;
            for(size_t i = max(n_mid-filter_mean_width, 0); i<min(n_mid+filter_mean_width, (int)cap_memory_tmp.size()); i++)
            {
                cap_mean += cap_memory_tmp[i];
                k++;
            }
            cap_mean /= max(k, 1);
            cap = cap_mean;
            msg.data = cap;
            
            // Publish
            cap_pub.publish(msg);
            new_data = false;
        }
        loop_rate.sleep();
    }
    return 0;
}
