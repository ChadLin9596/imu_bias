#include <ros/ros.h>
#include <std_msgs/String.h>
#include <sstream>
#include <math.h>
#include <geometry_msgs/Vector3.h>
#include <geometry_msgs/Quaternion.h>
#include <sensor_msgs/MagneticField.h>
#include <sensor_msgs/Imu.h>
#include <tf/transform_datatypes.h>
#include <tf/transform_broadcaster.h>

// declare global Variables
sensor_msgs::Imu imu_data;
sensor_msgs::Imu imu_pub;
sensor_msgs::MagneticField mag_data;
geometry_msgs::Quaternion quat;
double roll, pitch, yaw;
double mag_max_x, mag_max_y, mag_max_z;
double mag_min_x, mag_min_y, mag_min_z;
// declare ros publisher
ros::Publisher imu_cal_pub;
ros::Publisher mag_cal_pub;

void imu_cb(const sensor_msgs::Imu::ConstPtr& msgs)
{
  imu_data = *msgs;
  imu_pub = *msgs;
}

void mag_cb(const sensor_msgs::MagneticField::ConstPtr& msgs)
{
  mag_data = *msgs;

  // =============computing bias============
  if(mag_data.magnetic_field.x >= mag_max_x || mag_data.magnetic_field.x <= mag_min_x){
    if(mag_data.magnetic_field.x >= mag_max_x)
      mag_max_x = mag_data.magnetic_field.x;
    else
      mag_min_x = mag_data.magnetic_field.x;
  }

  if(mag_data.magnetic_field.y >= mag_max_y || mag_data.magnetic_field.y <= mag_min_y){
    if(mag_data.magnetic_field.y >= mag_max_y)
      mag_max_y = mag_data.magnetic_field.y;
    else
      mag_min_y = mag_data.magnetic_field.y;
  }

  if(mag_data.magnetic_field.z >= mag_max_z || mag_data.magnetic_field.z <= mag_min_z){
    if(mag_data.magnetic_field.z >= mag_max_z)
      mag_max_z = mag_data.magnetic_field.z;
    else
      mag_min_z = mag_data.magnetic_field.z;
  }
  // =============computing bias============

  double mag_int_x = (mag_max_x - mag_min_x)/2;
  double mag_int_y = (mag_max_y - mag_min_y)/2;
  double mag_int_z = (mag_max_z - mag_min_z)/2;
  double mag_mid_x = (mag_max_x + mag_min_x)/2;
  double mag_mid_y = (mag_max_y + mag_min_y)/2;
  double mag_mid_z = (mag_max_z + mag_min_z)/2;

  //printf("x : %f \t y : %f \t z : %f\n",,,);

  double mag_int_avg = (mag_int_x + mag_int_y + mag_int_z)/3;
  double scale_x = mag_int_avg/mag_int_x;
  double scale_y = mag_int_avg/mag_int_y;
  double scale_z = mag_int_avg/mag_int_z;

  // let the data project to mean = 0 range = 0.5~-0.5
  mag_data.magnetic_field.x = (mag_data.magnetic_field.x - mag_mid_x) * scale_x;
  mag_data.magnetic_field.y = (mag_data.magnetic_field.y - mag_mid_y) * scale_y;
  mag_data.magnetic_field.z = (mag_data.magnetic_field.z - mag_mid_z) * scale_z;

  mag_cal_pub.publish(mag_data);
}


int main (int argc , char **argv)
{
  ros::init(argc,argv,"imucalibrate");
  ros::NodeHandle n;
  // declare subscriber
  ros::Subscriber imu_sub = n.subscribe<sensor_msgs::Imu> ("/mavros/imu/data",1,imu_cb);
  ros::Subscriber mag_sub = n.subscribe<sensor_msgs::MagneticField> ("/mavros/imu/mag",1,mag_cb);
  // declare publisher
  imu_cal_pub = n.advertise<sensor_msgs::Imu> ("/mavros/imu/calib",1);
  mag_cal_pub = n.advertise<sensor_msgs::MagneticField> ("/mavros/imu/magcal",1);
  // initial setting
  mag_max_x = 0; mag_max_y = 0; mag_max_z = 0;
  mag_min_x = 0; mag_min_y = 0; mag_min_z = 0;


  ros::Rate loop_rate(50);

  while(ros::ok())
  {
    // compute roll pitch yaw M_PI
    tf::Quaternion orientation;
    tf::quaternionMsgToTF(imu_data.orientation, orientation);
    tf::Matrix3x3(orientation).getRPY(roll, pitch, yaw);
    printf("before roll : %f \t pitch : %f \t yaw : %f\n",roll,pitch,yaw);

    // declare magnetic_field
    float mag_x = mag_data.magnetic_field.x;
    float mag_y = mag_data.magnetic_field.y;
    float mag_z = mag_data.magnetic_field.z;

    // caculate yaw from magnetic_field
    double xh = mag_x * cos(pitch) + mag_y * sin(roll) * sin(pitch) - mag_z * cos(roll) * sin(pitch);
    double yh = mag_y * cos(roll) + mag_z * sin(roll);
    yaw = -std::atan2(yh,xh);
    printf("after roll : %f \t pitch : %f \t yaw : %f\n",roll,pitch,yaw);

    // transfer euler angle to quaternion
    tf::Quaternion q = tf::createQuaternionFromRPY(roll, pitch, yaw);
    tf::quaternionTFToMsg(q,quat);

    imu_pub.orientation = quat;

    imu_cal_pub.publish(imu_pub);

    ros::spinOnce();
    loop_rate.sleep();
  }
}
