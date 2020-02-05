#include <iostream>
#include <unistd.h>

#include <ros/ros.h>
#include <gpsd_client/GPSFix.h>
#include "std_msgs/Float64MultiArray.h"
#include <cmath>
#include <vector>
#include <proj_api.h>

using namespace std;
double latitude, longitude, track;
bool new_data = false;
bool data_valid = false;
//double east0 = 253575.276988;
//double north0 = 6805716.83588;
double east0 = 253495.324541;
double north0 = 6805738.35646;

void navFix_callback(const gpsd_client::GPSFix::ConstPtr& msg)
{
  latitude =  msg->latitude;
  longitude =  msg->longitude;
  track = msg->track;
  data_valid = (msg->status>=msg->STATUS_MODE_2D);
  new_data = true;
}

int main(int argc, char *argv[])
{
  ros::init(argc, argv, "lambert_node");
  ros::NodeHandle n;

  // Parameters
  ros::NodeHandle n_private("~");
  double frequency = n_private.param<double>("frequency", 1.0);

  // Init proj
  projPJ pj_lambert, pj_latlong;
  if (!(pj_lambert = pj_init_plus("+init=epsg:2154")))
  {
    ROS_WARN("[Lambert_node] Error Lambert \n");
    exit(1);
  }

  if (!(pj_latlong = pj_init_plus("+init=epsg:4326")))
  {
    ROS_WARN("[Lambert_node] Error LatLong \n");
    exit(1);
  }

  // Topics
  ros::Subscriber navFix_sub = n.subscribe("fix", 1, navFix_callback);
  ros::Publisher pose_pub = n.advertise<std_msgs::Float64MultiArray>("poseRaw", 1);

  ROS_INFO("[FUSION lambert] Start Ok");
  ros::Rate loop_rate(frequency);
  while (ros::ok())
  {
    ros::spinOnce();
    if(new_data)
    {
      if(longitude != 0. && latitude != 0. && data_valid)
      {
        std_msgs::Float64MultiArray msg_pose;
        msg_pose.data.clear();
        double east = longitude*M_PI/180.0; // Longitude
        double north = latitude*M_PI/180.0; // Latitude
        pj_transform(pj_latlong, pj_lambert, 1, 1, &east, &north, nullptr);

        std::vector<double> poseLamb = {north - north0, east - east0, track};
        msg_pose.data.insert(msg_pose.data.end(), poseLamb.begin(), poseLamb.end());
        /*message : data(0) = north;
                    data(1) = east; 
            */
        pose_pub.publish(msg_pose);
      }
      new_data = false;
    }
    loop_rate.sleep();
  }
  pj_free(pj_latlong);  
  pj_free(pj_lambert);
  return 0;
}
