#include <ros/ros.h>
#include <ros/package.h>
#include <stdio.h>
#include <string>
#include <pthread.h>
#include <nav_msgs/Odometry.h>
#include <geometry_msgs/Point32.h>
#include <geometry_msgs/Twist.h>
#include "sensor_msgs/LaserScan.h"
#include <leg_msgs/PositionMeasurement.h>
#include <leg_msgs/Legs_Position.h>
#include <cir_msgs/Device_Ultra_Msg.h>
#include "vfh_algorithm.h"

struct track_human
{
	std::string				id_;
	geometry_msgs::Point			pos_;
	double					dir_;
};

ros::Publisher 			robot_vel_pub;
ros::Subscriber			legs_pos_sub_, ultra_sensor_sub_, laser_sub_, odom_sub_;
double					max_spd_;
track_human				track_human_;
bool laser_set = false, odom_set = false, robot_flag = true;
int* ultra_sensor;

static pthread_t p_thread;
static bool thread_flag = true;
static int thread_id;

double laser_ranges[361][2];
double odom_pose[3];
double odom_vel[3];
int odom_stall;

VFH_Algorithm *vfh_Algorithm;
// Control velocity
double con_vel[3];
bool synchronous_mode;
int speed, turnrate;
float dist;
double angdiff;
double dist_eps;
double ang_eps;
struct timeval startescape, curr;
bool escaping;
double timediff;
int escape_turnrate_deg;

// bookkeeping to implement hysteresis when rotating at the goal
int rotatedir;

// bookkeeping to implement smarter escape policy
int escapedir;

// how fast and how long to back up to escape from a stall
double escape_speed;
double escape_max_turnspeed;
double escape_time;


void OD_Callback(const nav_msgs::Odometry::ConstPtr& pose_msg);
void LS_Callback(const sensor_msgs::LaserScan::ConstPtr& msg);
void DoOneUpdate(int goal_x, int goal_y);

ros::Publisher chatter_pub;
void OD_Callback(const nav_msgs::Odometry::ConstPtr& pose_msg)
{
	odom_pose[0] = pose_msg->pose.pose.position.x * 1e2;
	odom_pose[1] = pose_msg->pose.pose.position.y * 1e2;
	odom_pose[2] = atan2(pose_msg->pose.pose.orientation.z, pose_msg->pose.pose.orientation.w)*180/M_PI*2;

	float init_x, init_y, init_angle;
	std::vector<float> robot_init;

	ros::param::get("/ROBOT_INIT", robot_init);
	//printf("TEST (%f %f %f), [%f, %f, %f]\n", odom_pose[0], odom_pose[1], odom_pose[2],robot_init[0], robot_init[1], robot_init[2]);
//	ros::param::set("/ROBOT_POSE_X", robot_init[0]-odom_pose[1]);
//	ros::param::set("/ROBOT_POSE_Y", robot_init[1]+odom_pose[0]);
//	ros::param::set("/ROBOT_ANGLE", robot_init[2]+odom_pose[2]);

	geometry_msgs::Point32 start;
	
	double dx = (robot_init[1]-odom_pose[0]);
	double dz = (robot_init[0]-odom_pose[1]);
	double dist = sqrt(pow(odom_pose[0],2)+pow(odom_pose[1],2));
	double dangle = (robot_init[2]-odom_pose[2]) *M_PI/180;
	start.x = robot_init[1]+odom_pose[0];
	start.z = robot_init[0]-odom_pose[1];

//	start.x = odom_pose[0]*sin(dangle)+odom_pose[1]*cos(dangle)+robot_init[1];
//	start.z = odom_pose[0]*cos(dangle)-odom_pose[1]*sin(dangle)+robot_init[0];

//	start.x = odom_pose[0]*sin(dangle)+odom_pose[1]*cos(dangle);
//	start.z = odom_pose[0]*cos(dangle)-odom_pose[1]*sin(dangle);


//	start.x = dx*sin(dangle)+dz*cos(dangle);
//	start.z = dx*cos(dangle)-dz*sin(dangle);


	//printf("%f, %f\n", start.x, start.z);

//	printf("%f %f\n", dx, dz, odom_pose[0]*sin(dangle));
	//printf("%f %f %f %f %f %f %f\n",robot_init[2]-odom_pose[2],robot_init[2], odom_pose[2],  robot_init[1], odom_pose[0],robot_init[0], odom_pose[1]); 

	start.x = -start.x;
	//start.z = dz;


	//start.x = -start.x;
	
	ros::param::set("/ROBOT_POSE_X", start.z);
	ros::param::set("/ROBOT_POSE_Y", -start.x);
	ros::param::set("/ROBOT_ANGLE", robot_init[2]+odom_pose[2]);



	//start.x = -(robot_init[1]+odom_pose[0])/1000;
	//start.z = (robot_init[0]-odom_pose[1])/1000;
	//start.y = robot_init[2]+odom_pose[2];
	odom_vel[0] = pose_msg->twist.twist.linear.x * 1e2;
	odom_vel[1] = pose_msg->twist.twist.linear.y * 1e2;
	odom_vel[2] = (pose_msg->twist.twist.angular.z) * 180 / M_PI;
	chatter_pub.publish(start);
	odom_set = true;
}

void Robot_Init()
{
	speed = 0;
	turnrate = 0;

	// initialize some navigation state
	rotatedir = 1;
	escapedir = 1;
	escaping = 0;

	// take the bigger of the two dimensions and halve to get a radius
	float robot_radius = 0.5;//static_cast<float> (MAX(geom->size.sl,geom->size.sw));
	robot_radius /= 2.0;

	vfh_Algorithm->SetRobotRadius( robot_radius * 1e3 );

}


int main(int argc, char **argv)
{
	ros::init(argc, argv,"leg_follow");
	ros::NodeHandle nh;
	odom_sub_ = nh.subscribe("/odom", 1, OD_Callback);
	chatter_pub = nh.advertise<geometry_msgs::Point32>("poo_2d", 3);
	ros::Rate r(10);

	while(ros::ok())
	{
		ros::spinOnce();
		r.sleep();
	}
	return 0;
}
