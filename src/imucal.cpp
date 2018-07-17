#include "ros/ros.h"
#include "std_msgs/String.h"
#include <sstream>
#include <geometry_msgs/Vector3.h>
#include <geometry_msgs/Quaternion.h>
#include <sensor_msgs/MagneticField.h>
#include <sensor_msgs/Imu.h>
#include "math.h"
//http://www.euclideanspace.com/maths/geometry/rotations/conversions/quaternionToEuler/
float buffer[3][100];
float diff_avg;
float a,b,r;
int j=0;
float roll=0;
float pitch=0;
float yaw=0;
float xyaw;
float yyaw;
float yaw_mag;
float t0,t1,t2,t3,t4;
float yaw_init=0;
float yaw_diff=0;
float yaw_dif=0;
float tep=1.3439;
bool set=false;
float head;

sensor_msgs::MagneticField imu;
geometry_msgs::Vector3 max;
geometry_msgs::Vector3 min;
geometry_msgs::Vector3 bias;
geometry_msgs::Vector3 diff;
geometry_msgs::Vector3 scale;
geometry_msgs::Vector3 temp;
void calib(const sensor_msgs::MagneticField::ConstPtr& msg)
{
	imu = *msg;
	//printf("bias.x = %f \n",imu.magnetic_field.x);
	/*buffer[0][j]=imu.magnetic_field.x;
	buffer[1][j]=imu.magnetic_field.y;
	buffer[2][j]=imu.magnetic_field.z;
	j++;*/
	temp.x=imu.magnetic_field.x;
	temp.y=imu.magnetic_field.y;
	temp.z=imu.magnetic_field.z;
	if(temp.x>max.x){
	max.x=temp.x;
	}
	if(temp.y>max.y){
	max.y=temp.y;
	}
	if(temp.z>max.z){
	max.z=temp.z;
	}
	if(temp.x<min.x){
	min.x=temp.x;
	}
	if(temp.y<min.y){
	min.y=temp.y;
	}
	if(temp.z<min.z){
	min.z=temp.z;
	}
	bias.x=(max.x+min.x)/2;
	bias.y=(max.y+min.y)/2;
	bias.z=(max.z+min.z)/2;
	//printf("bias.x = %f ,bias.y = %f ,bias.z = %f\n",bias.x,bias.y,bias.z);
	diff.x=(max.x-min.x)/2;
	diff.y=(max.y-min.y)/2;
	diff.z=(max.z-min.z)/2;
	diff_avg=(diff.x+diff.y+diff.z)/3;
	scale.x=diff_avg/diff.x;
	scale.y=diff_avg/diff.y;
	scale.z=diff_avg/diff.z;
	//printf("scale.x = %f ,scale.y = %f ,scale.z = %f \n",scale.x,scale.y,scale.z);
	//20180424:-0.000001,0.000005,-0.000008,1.060807,0.955803.0.989041
	
}
sensor_msgs::Imu pose_qua;
geometry_msgs::Vector3 total;
void quaternion(const sensor_msgs::Imu::ConstPtr& msg)
{
	pose_qua = *msg;
	
	t0 = +2.0*(pose_qua.orientation.w*pose_qua.orientation.x+pose_qua.orientation.y*pose_qua.orientation.z);
	t1 = +1.0-2.0*(pose_qua.orientation.x*pose_qua.orientation.x+pose_qua.orientation.y*pose_qua.orientation.y);
	roll = std::atan2(t0,t1);
	//roll = (roll/3.1415926*180);
        
	t2 = +2.0 * (pose_qua.orientation.w*pose_qua.orientation.y-pose_qua.orientation.x * pose_qua.orientation.z);
   	pitch = std::asin(t2);
	//pitch = (pitch/3.1415926*180);

	t3 = +2.0*(pose_qua.orientation.w*pose_qua.orientation.z+pose_qua.orientation.x*pose_qua.orientation.y);
	t4 = +1.0-2.0*(pose_qua.orientation.y*pose_qua.orientation.y+pose_qua.orientation.z*pose_qua.orientation.z);

	yaw = std::atan2(t3,t4);
	//yaw = (yaw/3.1415926*180);
  printf("before roll = %f , pitch = %f , yaw = %f \n",roll,pitch,yaw);
	temp.x=pose_qua.angular_velocity.x;
	temp.y=pose_qua.angular_velocity.y;
	temp.z=pose_qua.angular_velocity.z;
	total.x+=pose_qua.angular_velocity.x;
	total.y+=pose_qua.angular_velocity.y;
	total.z+=pose_qua.angular_velocity.z;
	//printf("head = %f,head = %f,head = %f \n",temp.x,temp.y,temp.z);
	if(set == false){
	head=pose_qua.header.seq;
	set = true;
	}
	//printf("head = %f \n",head);
	if(temp.x>max.x){
	max.x=temp.x;
	}
	if(temp.y>max.y){
	max.y=temp.y;
	}
	if(temp.z>max.z){
	max.z=temp.z;
	}
	if(temp.x<min.x){
	min.x=temp.x;
	}
	if(temp.y<min.y){
	min.y=temp.y;
	}
	if(temp.z<min.z){
	min.z=temp.z;
	}
	bias.x=total.x/(pose_qua.header.seq-head+1);
	bias.y=total.y/(pose_qua.header.seq-head+1);
	bias.z=total.z/(pose_qua.header.seq-head+1);
	printf("bias.x = %f ,bias.y = %f ,bias.z = %f\n",bias.x,bias.y,bias.z);
	diff.x=(max.x-min.x)/2;
	diff.y=(max.y-min.y)/2;
	diff.z=(max.z-min.z)/2;
	scale.x=1/diff.x;
	scale.y=1/diff.y;
	scale.z=1/diff.z;
	//printf("scale.x = %f ,scale.y = %f ,scale.z = %f \n",scale.x,scale.y,scale.z);
}

/*void qua2eul(sensor_msgs::MagneticField& imu)
{
	//printf("bias.x = %f ,bias.x = %f ,bias.x = %f \n",buffer[0][j],buffer[1][j],buffer[2][j]);
	//j++;
	int i;
	temp1[0][0]=10;
	temp1[0][1]=-10;
	temp2[0][0]=10;
	temp2[0][1]=-10;
	temp3[0][0]=10;
	temp3[0][1]=-10;
	for(i=0; i<100; i++){
	
		//printf("bias.x = %f \n",bufferx[i]);
		if(buffer[0][i]>temp1[0][0]){
		max.x = buffer[0][i];
		temp1[0][0] = max.x;
		}
		if(buffer[0][i]<temp1[0][1]){
		min.x = buffer[0][i];
		temp1[0][1] = min.x;
		}
		if(buffer[1][i]>temp2[0][0]){
		max.y = buffer[1][i];
		temp2[0][0] = max.y;
		}
		if(buffer[1][i]<temp2[0][1]){
		min.y = buffer[1][i];
		temp2[0][1] = min.y;
		}
		if(buffer[2][i]>temp3[0][0]){
		max.z = buffer[2][i];
		temp3[0][0] = max.z;
		}
		if(buffer[2][i]<temp3[0][1]){
		min.z = buffer[2][i];
		temp3[0][1] = min.z;
		}
	}
	bias.x=(max.x+min.x)/2;
	bias.y=(max.y+min.y)/2;
	bias.z=(max.z+min.z)/2;
	printf("bias.x = %f ,bias.x = %f ,bias.x = %f\n",bias.x,bias.y,bias.z);
	diff.x=(max.x-min.x)/2;
	diff.y=(max.y-min.y)/2;
	diff.z=(max.z-min.z)/2;
	diff_avg=(diff.x+diff.y+diff.z)/3;
	scale.x=diff_avg/diff.x;
	scale.y=diff_avg/diff.y;
	scale.z=diff_avg/diff.z;
	printf("scale.x = %f ,scale.x = %f ,scale.x = %f\n",scale.x,scale.y,scale.z);
}*/
geometry_msgs::Quaternion q;
void result(sensor_msgs::MagneticField& imu, float roll,float pitch)
{
	roll = std::atan2(t0,t1);
   	pitch = std::asin(t2);
	/*roll = (roll/3.1415926*180);
	pitch = (pitch/3.1415926*180);
	yaw = (yaw/3.1415926*180);
	printf("roll = %f , pitch = %f , yaw = %f \n",roll,pitch,yaw);*/
	imu.magnetic_field.x=(imu.magnetic_field.x+0.000001)*1.060807;
	imu.magnetic_field.y=(imu.magnetic_field.y-0.000005)*0.955803;
	xyaw=imu.magnetic_field.x*cos(pitch)+imu.magnetic_field.y*sin(roll)*sin(pitch)-imu.magnetic_field.z*cos(roll)*sin(pitch);
	yyaw=imu.magnetic_field.y*cos(roll)+imu.magnetic_field.z*sin(roll);

	yaw_mag=-std::atan2(yyaw,xyaw);
	//yaw_mag = (yaw_mag/3.1415926*180);
  printf("yaw_mag = %f \n",yaw_mag);
	/*yaw_dif = yaw_mag-tep;
	tep = yaw_mag;
	
	if(set == false){
		if(yaw_dif<-0.52359 || yaw_dif>0.52359){
		yaw_init = yaw_mag;
		set=true;
		}
	}
	yaw_diff = yaw_mag-yaw_init;
	
	if(yaw_diff>3.1415926){
	yaw_diff = yaw_diff-6.2831852;
	}
	else if(yaw_diff<-3.1415926){
	yaw_diff = yaw_diff+6.2831852;
	}*/
	//printf("yaw_init = %f \n",yaw_init);
	double cy = cos(yaw_mag * 0.5);
	double sy = sin(yaw_mag * 0.5);
	double cr = cos(roll * 0.5);
	double sr = sin(roll * 0.5);
	double cp = cos(pitch * 0.5);
	double sp = sin(pitch * 0.5);

	q.w = cy * cr * cp + sy * sr * sp;
	q.x = cy * sr * cp - sy * cr * sp;
	q.y = cy * cr * sp + sy * sr * cp;
	q.z = sy * cr * cp - cy * sr * sp;
	//printf("q.w = %f,q.x = %f ,q.y = %f, q.z = %f \n",q.w,q.x,q.y,q.z);
	/*float roll2;
	float k0 = 2.0*(q.w*q.x+q.y*q.z);
	float k1 = 1.0-2.0*(q.x*q.x+q.y*q.y);
	roll2 = std::atan2(k0,k1);
	roll2 = (roll2/3.1415926*180);
        float pitch2;
	float k2 = 2.0 * (q.w*q.y-q.x * q.z);
   	pitch2 = std::asin(k2);
	pitch2 = (pitch2/3.1415926*180);
	float yaw2;
	float k3 = 2.0*(q.w*q.z+q.x*q.y);
	float k4 = 1.0-2.0*(q.y*q.y+q.z*q.z);
	yaw2 = std::atan2(k3,k4);
	yaw2 = (yaw2/3.1415926*180);*/
	float roll2=0;
	roll2 = (roll/3.1415926*180);
  float pitch2=0;
	pitch2 = (pitch/3.1415926*180);
	float yaw2=0;
	yaw2 = (yaw_mag/3.1415926*180);
	//printf("roll = %f,pitch = %f ,yaw= %f \n",roll2,pitch2,yaw2);

}

int main(int argc , char **argv)
{

	ros::init(argc , argv , "imucal");	
	ros::NodeHandle n;
  ros::Subscriber mag_sub = n.subscribe<sensor_msgs::MagneticField> ("/mavros/imu/mag",1,calib);
  ros::Subscriber qua_sub = n.subscribe<sensor_msgs::Imu> ("/mavros/imu/data",1,quaternion);
  ros::Publisher cal_pub = n.advertise<sensor_msgs::Imu>("/mavros/imu/calib", 1);
	ros::Rate loop_rate(60);

	max.x=0;
	max.y=0;
	max.z=0;
	min.x=0;
	min.y=0;
	min.z=0;

	while (ros::ok())
	{
	result(imu,roll,pitch);
	//qua2eul(imu);	
	pose_qua.orientation.w=q.w;
	pose_qua.orientation.x=q.x;
	pose_qua.orientation.y=q.y;
	pose_qua.orientation.z=q.z;

  cal_pub.publish(pose_qua);
	ros::spinOnce();
	loop_rate.sleep();
	}
	
        return 0;
}

