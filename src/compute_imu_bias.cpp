#include <ros/ros.h>
#include <sensor_msgs/Imu.h>
#include <eigen3/Eigen/Dense>
#include <string>
#include <geometry_msgs/PoseStamped.h>
//#include <UKF/output.h>
using namespace std;
sensor_msgs::Imu imu_data;
//geometry_msgs::PoseStamped mocap_data;
// UKF::output force_data;
void imu_cb(const sensor_msgs::Imu::ConstPtr &msg){
  imu_data = *msg;
}
/*
void mocap_cb(const geometry_msgs::PoseStamped::ConstPtr &msg){
  mocap_data = *msg;
}
*/
/*
void force_cb(const UKF::output::ConstPtr &msg){
  force_data = *msg;
}
*/
int main(int argc, char **argv)
{
  ros::init(argc, argv, "compute_imu_bias");
  ros::NodeHandle nh;
  int n = 0;
  string topic_imu, topic_mocap, topic_force;
  ros::param::get("~n", n);
  ros::param::get("~topic_imu", topic_imu);
  //ros::param::get("~topic_mocap", topic_mocap);
  //ros::param::get("~topic_force", topic_force);
  ros::Subscriber imu_sub = nh.subscribe<sensor_msgs::Imu>(topic_imu, 2, imu_cb);
  //ros::Subscriber mocap_sub = nh.subscribe<geometry_msgs::PoseStamped>(topic_mocap, 2, mocap_cb);
  //ros::Subscriber force_sub = nh.subscribe<UKF::output>(topic_force, 2, force_cb);
  ros::Rate rate(50);
  //imu
  float sum_x_imu = 0,sum_y_imu = 0, sum_z_imu = 0, avg_x_imu = 0, avg_y_imu = 0, avg_z_imu = 0;
  int count = 0, i = 0 ;
  std::vector<float> x_imu, y_imu, z_imu;
  float square_x_imu = 0, square_y_imu = 0, square_z_imu = 0;
  float avg_sigma_x_imu = 0, avg_sigma_y_imu = 0, avg_sigma_z_imu = 0;
  float sigma_x_imu = 0, sigma_y_imu = 0, sigma_z_imu = 0;
  //mocap
  /*
  float sum_x_mocap = 0,sum_y_mocap = 0, sum_z_mocap = 0, avg_x_mocap = 0, avg_y_mocap = 0, avg_z_mocap = 0;
  std::vector< float > x_mocap, y_mocap, z_mocap;
  float square_x_mocap = 0, square_y_mocap = 0, square_z_mocap = 0;
  float avg_sigma_x_mocap = 0, avg_sigma_y_mocap = 0, avg_sigma_z_mocap = 0;
  float sigma_x_mocap = 0, sigma_y_mocap = 0, sigma_z_mocap = 0;
  //force
  float sum_x_force = 0,sum_y_force = 0, sum_z_force = 0, avg_x_force = 0, avg_y_force = 0, avg_z_force = 0;
  std::vector< float > x_force, y_force, z_force;
  float square_x_force = 0, square_y_force = 0, square_z_force = 0;
  float avg_sigma_x_force = 0, avg_sigma_y_force = 0, avg_sigma_z_force = 0;
  float sigma_x_force = 0, sigma_y_force = 0, sigma_z_force = 0;
  */
  //Eigen::VectorXd linear_x;
  while(ros::ok()){

  if (imu_data.linear_acceleration.x !=0 && imu_data.linear_acceleration.y !=0){

    // x_imu is a vector from( std::vector<float> )
    x_imu.push_back(imu_data.linear_acceleration.x);
    y_imu.push_back(imu_data.linear_acceleration.y);
    z_imu.push_back(imu_data.linear_acceleration.z);
    /*
    x_mocap.push_back(mocap_data.pose.position.x);
    y_mocap.push_back(mocap_data.pose.position.y);
    z_mocap.push_back(mocap_data.pose.position.z);

    x_force.push_back(force_data.force.x);
    y_force.push_back(force_data.force.y);
    z_force.push_back(force_data.force.z);
    */
    i += 1;
    sum_x_imu += imu_data.linear_acceleration.x;
    sum_y_imu += imu_data.linear_acceleration.y;
    sum_z_imu += imu_data.linear_acceleration.z;
    /*
    sum_x_mocap += mocap_data.pose.position.x;
    sum_y_mocap += mocap_data.pose.position.y;
    sum_z_mocap += mocap_data.pose.position.z;

    sum_x_force += force_data.force.x;
    sum_y_force += force_data.force.y;
    sum_z_force += force_data.force.z;
    */
    count += 1;
    avg_x_imu = sum_x_imu / count;
    avg_y_imu = sum_y_imu / count;
    avg_z_imu = sum_z_imu / count;
    /*
    avg_x_mocap = sum_x_mocap / count;
    avg_y_mocap = sum_y_mocap / count;
    avg_z_mocap = sum_z_mocap / count;

    avg_x_force = sum_x_force / count;
    avg_y_force = sum_y_force / count;
    avg_z_force = sum_z_force / count;
    */
  }
  if(i == n-1){
    ROS_INFO("---Start calculate---");
    ROS_INFO("avg_x_imu = %f, avg_y_imu = %f, avg_z_imu = %f", avg_x_imu, avg_y_imu, avg_z_imu);
    //ROS_INFO("avg_x_mocap = %f, avg_y_mocap = %f, avg_z_mocap = %f", avg_x_mocap, avg_y_mocap, avg_z_mocap);
    //ROS_INFO("avg_x_force = %f, avg_y_force = %f, avg_z_force = %f", avg_x_force, avg_y_force, avg_z_force);
    for(int k = 0 ; k < n ; k++){
      square_x_imu = pow((x_imu[k] - avg_x_imu),2);
      square_y_imu = pow((y_imu[k] - avg_y_imu),2);
      square_z_imu = pow((z_imu[k] - avg_z_imu),2);
      /*
      square_x_mocap = pow((x_mocap[k] - avg_x_mocap),2);
      square_y_mocap = pow((y_mocap[k] - avg_y_mocap),2);
      square_z_mocap = pow((z_mocap[k] - avg_z_mocap),2);

      square_x_force = pow((x_force[k] - avg_x_force),2);
      square_y_force = pow((y_force[k] - avg_y_force),2);
      square_z_force = pow((z_force[k] - avg_z_force),2);
      */
      avg_sigma_x_imu += square_x_imu;
      avg_sigma_y_imu += square_y_imu;
      avg_sigma_z_imu += square_z_imu;
      /*
      avg_sigma_x_mocap += square_x_mocap;
      avg_sigma_y_mocap += square_y_mocap;
      avg_sigma_z_mocap += square_z_mocap;

      avg_sigma_x_force += square_x_force;
      avg_sigma_y_force += square_y_force;
      avg_sigma_z_force += square_z_force;
      */
    }

    sigma_x_imu = sqrt(avg_sigma_x_imu / n);
    sigma_y_imu = sqrt(avg_sigma_y_imu / n);
    sigma_z_imu = sqrt(avg_sigma_z_imu / n);
    /*
    sigma_x_mocap = sqrt(avg_sigma_x_mocap / n);
    sigma_y_mocap = sqrt(avg_sigma_y_mocap / n);
    sigma_z_mocap = sqrt(avg_sigma_z_mocap / n);

    sigma_x_force = sqrt(avg_sigma_x_force / n);
    sigma_y_force = sqrt(avg_sigma_y_force / n);
    sigma_z_force = sqrt(avg_sigma_z_force / n);
    */
    ROS_INFO("sigma_x_imu = %f, sigma_y_imu = %f, sigma_z_imu = %f", sigma_x_imu, sigma_y_imu, sigma_z_imu);
    //ROS_INFO("sigma_x_mocap = %f, sigma_y_mocap = %f, sigma_z_mocap = %f", sigma_x_mocap, sigma_y_mocap, sigma_z_mocap);
    //ROS_INFO("sigma_x_force = %f, sigma_y_force = %f, sigma_z_force = %f", sigma_x_force, sigma_y_force, sigma_z_force);
    i += 1;
    ROS_INFO("---Finish---");
    ros::shutdown();
  }

  //ROS_INFO("Bias: ax = %f, ay = %f, az = %f", avg_x_imu, avg_y_imu, avg_z_imu);
/*
    if(abs(imu_data.linear_acceleration.x) > 1 ){
      ROS_INFO("a_x > 1");
      count += 1;
      ROS_INFO("Numbers = %d", count);
    }
    */
  ros::spinOnce();
  rate.sleep();
  }


}
