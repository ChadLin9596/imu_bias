#include <ros/ros.h>
#include <sensor_msgs/Imu.h>
#include <eigen3/Eigen/Dense>
#include <string>
#include <geometry_msgs/PoseStamped.h>

using namespace std;

sensor_msgs::Imu imu_data;

void gyo_cb(const sensor_msgs::Imu::ConstPtr &msg){
  imu_data = *msg;
}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "compute_gyo_bias");
  ros::NodeHandle nh;

  int n = 0;
  string gyo_topic;// topic_mocap, topic_force;
  ros::param::get("~n", n);
  ros::param::get("~topic_sensor", gyo_topic);

  ros::Subscriber imu_sub = nh.subscribe<sensor_msgs::Imu>(gyo_topic, 2, gyo_cb);

  ros::Rate rate(50);
  //imu
  float sum_x_imu = 0,sum_y_imu = 0, sum_z_imu = 0, avg_x_imu = 0, avg_y_imu = 0, avg_z_imu = 0;
  int count = 0, i = 0 ;
  std::vector<float> x_imu, y_imu, z_imu;
  float square_x_imu = 0, square_y_imu = 0, square_z_imu = 0;
  float avg_sigma_x_imu = 0, avg_sigma_y_imu = 0, avg_sigma_z_imu = 0;
  float sigma_x_imu = 0, sigma_y_imu = 0, sigma_z_imu = 0;
  //Eigen::VectorXd linear_x;
  while(ros::ok()){

  if (imu_data.linear_acceleration.x !=0 && imu_data.linear_acceleration.y !=0){

    // x_imu is a vector from( std::vector<float> )
    x_imu.push_back(imu_data.linear_acceleration.x);
    y_imu.push_back(imu_data.linear_acceleration.y);
    z_imu.push_back(imu_data.linear_acceleration.z);

    i += 1;

    sum_x_imu += imu_data.linear_acceleration.x;
    sum_y_imu += imu_data.linear_acceleration.y;
    sum_z_imu += imu_data.linear_acceleration.z;

    count += 1;

    avg_x_imu = sum_x_imu / count;
    avg_y_imu = sum_y_imu / count;
    avg_z_imu = sum_z_imu / count;

  }
  // if n is 800 then caculate it
  if(i == n-1){
    ROS_INFO("---Start calculate---");
    ROS_INFO("avg_x_imu = %f, avg_y_imu = %f, avg_z_imu = %f", avg_x_imu, avg_y_imu, avg_z_imu);
    for(int k = 0 ; k < n ; k++){
      square_x_imu = pow((x_imu[k] - avg_x_imu),2);
      square_y_imu = pow((y_imu[k] - avg_y_imu),2);
      square_z_imu = pow((z_imu[k] - avg_z_imu),2);

      avg_sigma_x_imu += square_x_imu;
      avg_sigma_y_imu += square_y_imu;
      avg_sigma_z_imu += square_z_imu;
    }

    sigma_x_imu = sqrt(avg_sigma_x_imu / n);
    sigma_y_imu = sqrt(avg_sigma_y_imu / n);
    sigma_z_imu = sqrt(avg_sigma_z_imu / n);

    ROS_INFO("sigma_x_imu = %f, sigma_y_imu = %f, sigma_z_imu = %f", sigma_x_imu, sigma_y_imu, sigma_z_imu);

    i += 1;
    ROS_INFO("---Finish---");
    ros::shutdown();
  }
  ROS_INFO("Bias: ax = %f, ay = %f, az = %f", avg_x_imu, avg_y_imu, avg_z_imu);

  ros::spinOnce();
  rate.sleep();
  }


}
