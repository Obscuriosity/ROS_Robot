// http://wiki.ros.org/navigation/Tutorials/RobotSetup/Sensors

#include <ros/ros.h>
#include <sensor_msgs/PointCloud.h>
#include <sensor_msgs/Range.h>

void snr_1_CB(const sensor_msgs::Range::ConstPtr& msg)
{
  cloud.points[0].x = msg.range;
  cloud.points[0].y = 0;
  cloud.points[0].z = 0;
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

    }

    cloud_pub.publish(cloud);
    r.sleep();
  }
}
