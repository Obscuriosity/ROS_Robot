// http://wiki.ros.org/navigation/Tutorials/RobotSetup/Sensors

#include <ros/ros.h>
#include <sensor_msgs/PointCloud.h>
#include <sensor_msgs/Range.h>

void snr_1_CB(const sensor_msgs::Range::ConstPtr& msg)
{
  cloud.points[0].x = msg.range;
  cloud.points[0].y = 0;
  cloud.points[0].z = 0;
  cloud.channels[0].values[0] = ;
}

int main(int argc, char** argv){
  ros::init(argc, argv, "point_cloud_publisher");

  ros::NodeHandle n;
  ros::Publisher cloud_pub = n.advertise<sensor_msgs::PointCloud>("cloud", 10);
  ros::Subscriber snr1sub = n.subscribe("snr_1", 10, snr_1_CB)

  unsigned int num_points = 8;

  int count = 0;
  ros::Rate r(10.0);
  while(n.ok()){
    sensor_msgs::PointCloud cloud;
    cloud.header.stamp = ros::Time::now();
    cloud.header.frame_id = "sonar_frame";

    cloud.points.resize(num_points);

    //we'll also add an intensity channel to the cloud - Why?
    cloud.channels.resize(1);
    cloud.channels[0].name = "intensities";
    cloud.channels[0].values.resize(num_points);

    //generate some fake data for our point cloud
    for(unsigned int i = 0; i < num_points; ++i){
      cloud.points[i].x = 1 + count;
      cloud.points[i].y = 2 + count;
      cloud.points[i].z = 3 + count;
      cloud.channels[0].values[i] = 100 + count;
    }

    cloud_pub.publish(cloud);
    ++count;
    r.sleep();
  }
}
