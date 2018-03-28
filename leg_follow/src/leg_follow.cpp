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
//void LS_Callback(const sensor_msgs::LaserScan::ConstPtr& msg);
void DoOneUpdate(int goal_x, int goal_y);

void
PutCommand(double v, double w)
{
	geometry_msgs::Twist cmd;

	con_vel[0] = v;
	con_vel[1] = 0;
	con_vel[2] = w;

	if(fabs(con_vel[2]) > (double)vfh_Algorithm->GetMaxTurnrate((int)con_vel[0]))
	{
		con_vel[2] = (double)vfh_Algorithm->GetMaxTurnrate((int)con_vel[0]);
	}
	cmd.linear.x = con_vel[0] / 1e3;
	cmd.angular.z = (con_vel[2])*M_PI/180;

	//ROS_ERROR("v : %.2f, w : %.2f", v, w);
	robot_vel_pub.publish(cmd);
}


double
angle_diff(double a, double b)
{
	double ra, rb;
	double d1, d2;

	ra = a*M_PI/180;
	if(ra<0)
		ra = 2*M_PI - ra;
	rb = b*M_PI/180;
	if(rb<0)
		rb = 2*M_PI - rb;


	d1 = ra-rb;
	d2 = 2*M_PI - fabs(d1);
	if(d1 > 0)
		d2 *= -1.0;

	if(fabs(d1) < fabs(d2))
		return((d1)*180/M_PI);
	else
		return((d2)*180/M_PI);
}

void LD_Callback(const leg_msgs::Legs_Position::ConstPtr& leg_pos_msg)
{
	double distance=5;
	ROS_ERROR("size : %d", leg_pos_msg->legs.size());
	if(leg_pos_msg->legs.size()>0 &&odom_set)
	{
		for(int iter=0; iter<leg_pos_msg->legs.size(); iter++)
		{
			ROS_INFO("%.2f, %.2f, id : %s", leg_pos_msg->legs[iter].cen.x * 1e3, leg_pos_msg->legs[iter].cen.z * 1e3, leg_pos_msg->legs[iter].object_id.c_str());
			double dist_to_leg = static_cast<float> (sqrt(pow((leg_pos_msg->legs[iter].cen.x - odom_pose[1]/1e3),2) + pow((leg_pos_msg->legs[iter].cen.z - odom_pose[0]/1e3),2)));
			ROS_ERROR("iter ");
			distance = std::min(distance, dist_to_leg);
			if(distance == dist_to_leg)
			{
				track_human_.id_ = leg_pos_msg->legs[iter].object_id;
				track_human_.pos_.x = leg_pos_msg->legs[iter].cen.x * 1e3;
				track_human_.pos_.z = leg_pos_msg->legs[iter].cen.z * 1e3;
				track_human_.dir_ = atan2(leg_pos_msg->legs[iter].cen.z, leg_pos_msg->legs[iter].cen.x) * 180/ 3.14 -90;
			}
		}
	}
	else
	{
		ROS_ERROR("no id");
		track_human_.id_ = "-1";
	}

	if(laser_set && odom_set && robot_flag)
	{
		ROS_INFO("%.2f, %.2f, id : %s", track_human_.pos_.x, track_human_.pos_.z, track_human_.id_.c_str());
		if(track_human_.id_ == "0")
			DoOneUpdate(track_human_.pos_.x, track_human_.pos_.z);
		else
			PutCommand(0, 0);	
	}
}

/*
void US_Callback(const cir_msgs::Device_Ultra_Msg::ConstPtr& ultra_sensor_msg)
{
	ultra_sensor[0] = ultra_sensor_msg->param[6];
	ultra_sensor[1] = ultra_sensor_msg->param[7];
	ultra_sensor[2] = ultra_sensor_msg->param[0];
	ultra_sensor[3] = ultra_sensor_msg->param[1];
	ultra_sensor[4] = ultra_sensor_msg->param[2];
}
*/
void LS_Callback(const sensor_msgs::LaserScan::ConstPtr& laser_sensor_msg)
{
	int i;
	double r;

	int laser_count = 361;
	for(i = 0; i < laser_count; i++)
		laser_ranges[i][0] = -1 ;

	for(i = 0; i < 181; i++)
	{
		if(isinf(laser_sensor_msg->ranges[(int)rint(84+(i*513/180))]))
			laser_ranges[i*2][0] = 3000;
		else if(laser_sensor_msg->ranges[(int)rint(84+(i*513/180))] * 1e3 > 10) 
			laser_ranges[i*2][0] = laser_sensor_msg->ranges[(int)rint(84+(i*513/180))] * 1e3;
		else
			laser_ranges[i*2][0] = 3000;
	}
	r = 3000.0;

	for (i = 0; i < laser_count; i++)
	{
		if (laser_ranges[i][0] != -1) {
			r = laser_ranges[i][0];
		}
		else {
			laser_ranges[i][0] = r;
		}
	}
	laser_set = true;
}

void OD_Callback(const nav_msgs::Odometry::ConstPtr& pose_msg)
{
	odom_pose[0] = pose_msg->pose.pose.position.x * 1e2;
	odom_pose[1] = pose_msg->pose.pose.position.y * 1e2;
	odom_pose[2] = atan2(pose_msg->pose.pose.orientation.z, pose_msg->pose.pose.orientation.w)*180/M_PI*2;

	odom_vel[0] = pose_msg->twist.twist.linear.x * 1e2;
	odom_vel[1] = pose_msg->twist.twist.linear.y * 1e2;
	odom_vel[2] = (pose_msg->twist.twist.angular.z) * 180 / M_PI;

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

void DoOneUpdate(int goal_x, int goal_z)
{
	// Figure how far, in distance and orientation, we are from the goal
	dist = static_cast<float> (sqrt(pow((goal_x - odom_pose[1]),2) + pow((goal_z - odom_pose[0]),2)));

	angdiff = angle_diff(0,odom_pose[2]);
	ROS_ERROR("%.2f, %.2f", goal_x, goal_z);
	ROS_ERROR("dist : %.2f, ang : %.2f, odom : %.2f", dist, angdiff, odom_pose[2]);

	// If we're currently escaping after a stall, check whether we've done
	// so for long enough.


	if(escaping)
	{
		gettimeofday(&curr,NULL);
		timediff = (curr.tv_sec + curr.tv_usec/1e6) -
				(startescape.tv_sec + startescape.tv_usec/1e6);
		if(timediff > escape_time)
		{
			// if we're still stalled, try escaping the other direction
			if(odom_stall)
				escapedir *= -1;
			escaping = false;
		}
	//	ROS_INFO("case 0");
	}

	if(escaping ||
			(escape_speed && escape_time && odom_stall))
	{
		if(!escaping)
		{
			gettimeofday(&startescape,NULL);
			escaping = true;
		}

		speed = (int)rint(escape_speed * escapedir * 1e3);
		if(escape_max_turnspeed)
		{
			// pick a random turnrate in
			// [-escape_max_turnspeed,escape_max_turnspeed]
			escape_turnrate_deg = (int)rint((escape_max_turnspeed)*180/M_PI);
			turnrate = (int)(2.0 * escape_turnrate_deg *
					rand()/(RAND_MAX+1.0)) -
					escape_turnrate_deg/2 + 1;
		}
		else
			turnrate = 0;
	//	ROS_INFO("case 1");
		PutCommand(speed, turnrate);
	}
	else if(dist < ((dist_eps * 1e3)-((dist_eps * 1e3)*0.1)))
	{
		speed = dist - 1000;
		turnrate = 0;
		ROS_INFO("case 1");
		//printf("case 1");
		PutCommand( speed, turnrate );
	}
	// CASE 2: The robot is at the goal, within user-specified tolerances, so
	//         stop.
	else if(dist < ((dist_eps * 1e3)+((dist_eps * 1e3)*0.1)))// && vfh_mode==0)
	{     	
			
		speed = 0;

		float Desired_Angle = static_cast<float> ((atan2((goal_x - odom_pose[1]),
				(goal_z - odom_pose[0]))
				* 180 / M_PI - odom_pose[2]));

		ROS_ERROR("D_D: %.2f",Desired_Angle);
		if(Desired_Angle < -10)
			turnrate = Desired_Angle;
		else if(Desired_Angle	 > 10)
			turnrate = Desired_Angle;
		else
			turnrate = 0;		

		ROS_INFO("case 2 ");
		//printf("case 2 ");

		ROS_INFO("W : %d", turnrate);
		PutCommand( speed, turnrate );

	}
	// CASE 3: The robot is too far from the goal position, so invoke VFH to
	//         get there.
	else if (dist > (dist_eps * 1e3))
	{
		float Desired_Angle = static_cast<float> ((90 + atan2((goal_x - odom_pose[1]),
				(goal_z - odom_pose[0]))
				* 180 / M_PI - odom_pose[2]));

		while (Desired_Angle > 360.0)
			Desired_Angle -= 360.0;
		while (Desired_Angle < 0)
			Desired_Angle += 360.0;

		vfh_Algorithm->Update_VFH(laser_ranges,
				(int)(odom_vel[0]),
				Desired_Angle,
				dist,
				static_cast<float> (dist_eps * 1000),
				speed,
				turnrate);
		ROS_INFO("case 3 ");
		// HACK: if we're within twice the distance threshold,
		// and still going fast, slow down.

		if((dist < (dist_eps * 1e3 * 2.0)) &&
				(speed > (vfh_Algorithm->GetCurrentMaxSpeed() / 2.0)))
		{

			speed =
					(int)rint(vfh_Algorithm->GetCurrentMaxSpeed() / 2.0);
		}
		ROS_INFO("case 3 ");
		PutCommand( speed, turnrate );
	}
}

int main(int argc, char **argv)
{
	ros::init(argc, argv,"leg_follow");
	ros::NodeHandle nh;

	double cell_size;
	int window_diameter;
	int sector_angle;
	double safety_dist_0ms;
	double safety_dist_1ms;
	int max_speed;
	int max_speed_narrow_opening;
	int max_speed_wide_opening;
	int max_acceleration;
	int min_turnrate;
	int max_turnrate_0ms;
	int max_turnrate_1ms;
	double min_turn_radius_safety_factor;
	double free_space_cutoff_0ms;
	double obs_cutoff_0ms;
	double free_space_cutoff_1ms;
	double obs_cutoff_1ms;
	double weight_desired_dir;
	double weight_current_dir;

	// read the synchronous flag from the cfg file: defaults to not synchronous
	nh.param("synchronous", synchronous_mode, false);
	nh.param("cell_size", cell_size, 0.1*1e3);
	nh.param("window_diameter", window_diameter, 61);
	nh.param("sector_angle", sector_angle, 5);
	nh.param("safety_dist_0ms", safety_dist_0ms, 0.1*1e3); /*original*/
	//nh.param("safety_dist_0ms", safety_dist_0ms, 0.15 *1e3); /*tuning*/
	nh.param("safety_dist_1ms", safety_dist_1ms, safety_dist_0ms); /**/
	nh.param("max_speed", max_speed, 400);//200); /*original*/
	//nh.param("max_speed", max_speed, 200);//200); /*tuning*/
	nh.param("max_speed_narrow_opening", max_speed_narrow_opening, max_speed-50);
	nh.param("max_speed_wide_opening", max_speed_wide_opening, max_speed);
	nh.param("max_acceleration", max_acceleration, 80);/*original*/
	//nh.param("max_acceleration", max_acceleration, 60);/*tuning*/
	nh.param("min_turnrate", min_turnrate, 0);
	nh.param("max_turnrate_0ms", max_turnrate_0ms, 60);//stop /**/
	nh.param("max_turnrate_1ms", max_turnrate_1ms, 60);//move /**/
	nh.param("min_turn_radius_safety_factor", min_turn_radius_safety_factor, 1.0);
	nh.param("free_space_cutoff_0ms", free_space_cutoff_0ms, 2000000.0);/*original*/
	//nh.param("free_space_cutoff_0ms", free_space_cutoff_0ms, 3000.0);/*tuning*/
	nh.param("obs_cutoff_0ms", obs_cutoff_0ms, free_space_cutoff_0ms);
	nh.param("free_space_cutoff_1ms", free_space_cutoff_1ms, free_space_cutoff_0ms);
	nh.param("obs_cutoff_1ms", obs_cutoff_1ms, free_space_cutoff_1ms);
	nh.param("weight_desired_dir", weight_desired_dir, 5.0);
	nh.param("weight_current_dir", weight_current_dir, 3.0);
	nh.param("distance_epsilon", dist_eps, 1.0); /*orignial*/
	//nh.param("distance_epsilon", dist_eps, 0.5); /*tuning*/
	nh.param("angle_epsilon", ang_eps, 10*M_PI/180);
	nh.param("escape_speed", escape_speed, 0.0);
	nh.param("escape_time", escape_time, 0.0);
	nh.param("escape_max_turnrate", escape_max_turnspeed, 0.0);

	vfh_Algorithm = new VFH_Algorithm(cell_size,
			window_diameter,
			sector_angle,
			safety_dist_0ms,
			safety_dist_1ms,
			max_speed,
			max_speed_narrow_opening,
			max_speed_wide_opening,
			max_acceleration,
			min_turnrate,
			max_turnrate_0ms,
			max_turnrate_1ms,
			min_turn_radius_safety_factor,
			free_space_cutoff_0ms,
			obs_cutoff_0ms,
			free_space_cutoff_1ms,
			obs_cutoff_1ms,
			weight_desired_dir,
			weight_current_dir);

	Robot_Init();
	vfh_Algorithm->Init();

	robot_vel_pub = nh.advertise<geometry_msgs::Twist>("/cmd_vel", 1);
	legs_pos_sub_ = nh.subscribe("global_leg_detection", 1, LD_Callback);
	ultra_sensor_sub_ = nh.subscribe("/DeviceNode/UltraSonic/data", 1, US_Callback);
	laser_sub_ = nh.subscribe("/scan", 1, LS_Callback);
	odom_sub_ = nh.subscribe("/odom", 1, OD_Callback);

	ultra_sensor = (int*)calloc(5, sizeof(int));

	track_human_.id_ = "-1";
	track_human_.pos_.x = 0;
	track_human_.pos_.y = 0;
	track_human_.pos_.z = 0;
	max_spd_ = 0.7;

	ros::Rate r(60);
	std::string robot_state;
	nh.setParam("ROBOT_STATE", "robot_on");

	for(int i=0; i<100; i++)
	{
		ros::spinOnce();
		r.sleep();
	}
	while(ros::ok())
	{
		ros::spinOnce();
		nh.getParam("ROBOT_STATE", robot_state);
		if(robot_state == "robot_on")
		{
			robot_flag = true;
		}
		else if(robot_state == "robot_off")
		{
			robot_flag = false;

			geometry_msgs::Twist robot_vel;
			robot_vel.linear.x = 0;
			robot_vel.angular.z = 0;
			robot_vel_pub.publish(robot_vel);
		}
		r.sleep();
	}
	return 0;
}
