#include <ros/ros.h>
#include <ncrl_sensor_cal/sensor_common.h>

// declare global Variables
sensor_msgs::Imu imu_data;
sensor_msgs::Imu imu_pub;
sensor_msgs::MagneticField mag_data;
geometry_msgs::Quaternion quat;
geometry_msgs::Vector3 euler_angle;
double roll, pitch, yaw;
// declare global variables
double mag_max_x = 0, mag_max_y = 0, mag_max_z = 0;
double mag_min_x = 0, mag_min_y = 0, mag_min_z = 0;
double imu_max_x = 0, imu_max_y = 0, imu_max_z = 0;
double imu_min_x = 0, imu_min_y = 0, imu_min_z = 0;
double imu_sum_x = 0, imu_sum_y = 0, imu_sum_z = 0;
double imu_avg_x = 0, imu_avg_y = 0, imu_avg_z = 0;
double gyo_sum_x = 0, gyo_sum_y = 0, gyo_sum_z = 0;
double gyo_avg_x = 0, gyo_avg_y = 0, gyo_avg_z = 0;
// declare ros publisher
ros::Publisher imu_cal_pub;
ros::Publisher mag_cal_pub;
ros::Publisher rpy_cal_pub;
// declare imu calibrate
bool state = true;
int count_imu = 0;
int key = 0;

using namespace std;

void keyboard_control()
{
  int c = getch();
  //ROS_INFO("C: %d",c);
  if (c != EOF) {
    switch (c){
    case 117: //u
      key = 1;
      break;
    case 100: //d
      key = 2;
      break;
    case 110: //n
      key = 3;
      break;
    case 101: //e
      key = 4;
      break;
    case 119: //w
      key = 5;
      break;
    case 115: // s
      key = 6;
      break;
    case 116: // t
      key = 7;
      break;
    }
  }
}


void imu_cb(const sensor_msgs::Imu::ConstPtr& msgs)
{
  imu_data = *msgs;
  imu_pub = *msgs;

  if (key != 0 && key != 7){
    switch(key){
    case 1:
      if (imu_data.linear_acceleration.z > imu_max_z || imu_data.linear_acceleration.z < imu_min_z){
        if (imu_data.linear_acceleration.z > imu_max_z)
          imu_max_z = imu_data.linear_acceleration.z;
        else
          imu_min_z = imu_data.linear_acceleration.z;
      }
      break;
    case 2:
      if (imu_data.linear_acceleration.z > imu_max_z || imu_data.linear_acceleration.z < imu_min_z){
        if (imu_data.linear_acceleration.z > imu_max_z)
          imu_max_z = imu_data.linear_acceleration.z;
        else
          imu_min_z = imu_data.linear_acceleration.z;
      }
      break;
    case 3:
      if (imu_data.linear_acceleration.x > imu_max_x || imu_data.linear_acceleration.x < imu_min_x){
        if (imu_data.linear_acceleration.x > imu_max_x)
          imu_max_x = imu_data.linear_acceleration.x;
        else
          imu_min_x = imu_data.linear_acceleration.x;
      }
      break;
    case 4:
      if (imu_data.linear_acceleration.y > imu_max_y || imu_data.linear_acceleration.y < imu_min_y){
        if (imu_data.linear_acceleration.y > imu_max_y)
          imu_max_y = imu_data.linear_acceleration.y;
        else
          imu_min_y = imu_data.linear_acceleration.y;
      }
      break;
    case 5:
      if (imu_data.linear_acceleration.y > imu_max_y || imu_data.linear_acceleration.y < imu_min_y){
        if (imu_data.linear_acceleration.y > imu_max_y)
          imu_max_y = imu_data.linear_acceleration.y;
        else
          imu_min_y = imu_data.linear_acceleration.y;
      }
      break;
    case 6:
      if (imu_data.linear_acceleration.x > imu_max_x || imu_data.linear_acceleration.x < imu_min_x){
        if (imu_data.linear_acceleration.x > imu_max_x)
          imu_max_x = imu_data.linear_acceleration.x;
        else
          imu_min_x = imu_data.linear_acceleration.x;
      }
      break;
    }
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

  mag_data.magnetic_field.x = (mag_data.magnetic_field.x - mag_mid_x) * scale_x;
  mag_data.magnetic_field.y = (mag_data.magnetic_field.y - mag_mid_y) * scale_y;
  mag_data.magnetic_field.z = (mag_data.magnetic_field.z - mag_mid_z) * scale_z;

  printf("mag_mid x : %f \t y : %f \t z : %f \t \n",mag_mid_x,mag_mid_y,mag_mid_z);
  printf("scale : x : %f \t y : %f \t z : %f \t \n",scale_x,scale_y,scale_z);

  //mag_mid_x = -0.000014; mag_mid_y = -0.000014; mag_mid_z = -0.000014;
  //scale_x   = 1.259550 ; scale_y   = 0.831721 ; scale_z   = 0.996274 ;
  mag_data.magnetic_field.x = (mag_data.magnetic_field.x - mag_mid_x) * scale_x;
  mag_data.magnetic_field.y = (mag_data.magnetic_field.y - mag_mid_y) * scale_y;
  mag_data.magnetic_field.z = (mag_data.magnetic_field.z - mag_mid_z) * scale_z;

  mag_cal_pub.publish(mag_data);
}


int main (int argc , char **argv)
{
  ros::init(argc,argv,"compute_imu_bias");
  ros::NodeHandle nh;

  int n1 = 0;
  int n2 = 0;
  string topic_sensor1;
  string topic_sensor2;
  ros::param::get("~n1", n1);
  ros::param::get("~n2", n2);
  ros::param::get("~topic_sensor1", topic_sensor1);
  ros::param::get("~topic_sensor2", topic_sensor2);
  // declare subscriber
  ros::Subscriber imu_sub = nh.subscribe<sensor_msgs::Imu> ("topic_sensor1",1,imu_cb);
  ros::Subscriber mag_sub = nh.subscribe<sensor_msgs::MagneticField> ("topic_sensor2",1,mag_cb);
  // declare publisher
  imu_cal_pub = nh.advertise<sensor_msgs::Imu> ("/mavros/imu/calib",1);           // publish imu calib
  mag_cal_pub = nh.advertise<sensor_msgs::MagneticField> ("/mavros/imu/magcal",1);// publish mag calib
  rpy_cal_pub = nh.advertise<geometry_msgs::Vector3> ("/mavros/imu/rpy",1);        // publish rpy result

  ros::Rate loop_rate(50);

  while(ros::ok())
  {
    if(!state){
      // compute roll pitch yaw M_PI
      //tf::Quaternion orientation;
      //tf::quaternionMsgToTF(imu_data.orientation, orientation);
      //tf::Matrix3x3(orientation).getRPY(roll, pitch, yaw);
      //printf("before roll : %f \t pitch : %f \t yaw : %f\n",roll*180/M_PI,pitch*180/M_PI,yaw*180/M_PI);

      // declare magnetic_field
      float mag_x = mag_data.magnetic_field.x;
      float mag_y = mag_data.magnetic_field.y;
      float mag_z = mag_data.magnetic_field.z;

      // caculate yaw from magnetic_field
      double xh = mag_x * cos(pitch) + mag_y * sin(roll) * sin(pitch) - mag_z * cos(roll) * sin(pitch);
      double yh = mag_y * cos(roll) + mag_z * sin(roll);
      yaw = -std::atan2(yh,xh);
      printf("after roll : %f \t pitch : %f \t yaw : %f\n",roll*180/M_PI,pitch*180/M_PI,yaw*180/M_PI);
      euler_angle.x = roll*180/M_PI;
      euler_angle.y = pitch*180/M_PI;
      euler_angle.z = yaw*180/M_PI;
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
