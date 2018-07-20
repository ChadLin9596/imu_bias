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
geometry_msgs::Vector3 euler_angle;
double roll, pitch, yaw;
double mag_max_x, mag_max_y, mag_max_z;
double mag_min_x, mag_min_y, mag_min_z;
// declare ros publisher
ros::Publisher imu_cal_pub;
ros::Publisher mag_cal_pub;
ros::Publisher rpy_cal_pub;
// declare imu calibrate
bool state = true;
float bias_x,bias_y,bias_z;
int n = 801;
float sum_x_imu = 0, sum_y_imu = 0, sum_z_imu = 0;
float avg_x_imu = 0, avg_y_imu = 0, avg_z_imu = 0;
int count_imu = 0;

void imu_cb(const sensor_msgs::Imu::ConstPtr& msgs)
{
  imu_data = *msgs;
  imu_pub = *msgs;
  
  if (state){
    if (imu_data.linear_acceleration.x != 0 && imu_data.linear_acceleration.y != 0){

      sum_x_imu += imu_data.linear_acceleration.x;
      sum_y_imu += imu_data.linear_acceleration.y;
      sum_z_imu += imu_data.linear_acceleration.z;

      count_imu += 1;

      avg_x_imu = sum_x_imu / count_imu;
      avg_y_imu = sum_y_imu / count_imu;
      avg_z_imu = sum_z_imu / count_imu;
      ROS_INFO("computing bias ... %d",count_imu);
    }
    if(count_imu == n-1){
      ROS_INFO("---Start calculate---");
      bias_x = sum_x_imu / (count_imu);
      bias_y = sum_y_imu / (count_imu);
      bias_z = (sum_z_imu / (count_imu))-9.81;
      ROS_INFO("bias_x = %f, bias_y = %f, bias_z = %f", bias_x, bias_y, bias_z);
      count_imu += 1;
      ROS_INFO("---Finish---");
      state = false;
    }
  }

  if (!state){
    imu_pub.linear_acceleration.x = imu_data.linear_acceleration.x - bias_x;
    imu_pub.linear_acceleration.y = imu_data.linear_acceleration.y - bias_y;
    imu_pub.linear_acceleration.z = imu_data.linear_acceleration.z - bias_z;
  }
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
  /*
  mag_data.magnetic_field.x = (mag_data.magnetic_field.x - mag_mid_x) * scale_x;
  mag_data.magnetic_field.y = (mag_data.magnetic_field.y - mag_mid_y) * scale_y;
  mag_data.magnetic_field.z = (mag_data.magnetic_field.z - mag_mid_z) * scale_z;

  printf("mag_mid x : %f \t y : %f \t z : %f \t \n",mag_mid_x,mag_mid_y,mag_mid_z);
  printf("scale : x : %f \t y : %f \t z : %f \t \n",scale_x,scale_y,scale_z);
  */
  mag_mid_x = -0.000014; mag_mid_y = -0.000014; mag_mid_z = -0.000014;
  scale_x   = 1.259550 ; scale_y   = 0.831721 ; scale_z   = 0.996274 ;
  mag_data.magnetic_field.x = (mag_data.magnetic_field.x - mag_mid_x) * scale_x;
  mag_data.magnetic_field.y = (mag_data.magnetic_field.y - mag_mid_y) * scale_y;
  mag_data.magnetic_field.z = (mag_data.magnetic_field.z - mag_mid_z) * scale_z;

  mag_cal_pub.publish(mag_data);
}


int main (int argc , char **argv)
{
  ros::init(argc,argv,"imucalibrate");
  ros::NodeHandle nh;
  // declare subscriber
  ros::Subscriber imu_sub = nh.subscribe<sensor_msgs::Imu> ("/mavros/imu/data",1,imu_cb);
  ros::Subscriber mag_sub = nh.subscribe<sensor_msgs::MagneticField> ("/mavros/imu/mag",1,mag_cb);
  // declare publisher
  imu_cal_pub = nh.advertise<sensor_msgs::Imu> ("/mavros/imu/calib",1);
  mag_cal_pub = nh.advertise<sensor_msgs::MagneticField> ("/mavros/imu/magcal",1);
  rpy_cal_pub = nh.advertise<geometry_msgs::Vector3> ("mavros/imu/rpy",1);
  // initial setting
  mag_max_x = 0; mag_max_y = 0; mag_max_z = 0;
  mag_min_x = 0; mag_min_y = 0; mag_min_z = 0;


  ros::Rate loop_rate(50);

  while(ros::ok())
  {
    if(!state){
      // compute roll pitch yaw M_PI
      tf::Quaternion orientation;
      tf::quaternionMsgToTF(imu_data.orientation, orientation);
      tf::Matrix3x3(orientation).getRPY(roll, pitch, yaw);
      printf("before roll : %f \t pitch : %f \t yaw : %f\n",roll*180/M_PI,pitch*180/M_PI,yaw*180/M_PI);

      // declare magnetic_field
      float mag_x = mag_data.magnetic_field.x;
      float mag_y = mag_data.magnetic_field.y;
      float mag_z = mag_data.magnetic_field.z;

      // caculate yaw from magnetic_field
      double xh = mag_x * cos(pitch) + mag_y * sin(roll) * sin(pitch) - mag_z * cos(roll) * sin(pitch);
      double yh = mag_y * cos(roll) + mag_z * sin(roll);
      yaw = -std::atan2(yh,xh);
      printf("after roll : %f \t pitch : %f \t yaw : %f\n",roll*180/M_PI,pitch*180/M_PI,yaw*180/M_PI);
      euler_angle.x = roll;
      euler_angle.y = pitch;
      euler_angle.z = yaw;
      // transfer euler angle to quaternion
      tf::Quaternion q = tf::createQuaternionFromRPY(roll, pitch, yaw);
      tf::quaternionTFToMsg(q,quat);
      imu_pub.orientation = quat;

      rpy_cal_pub.publish(euler_angle);
      imu_cal_pub.publish(imu_pub);
    }
    ros::spinOnce();
    loop_rate.sleep();
  }
}
