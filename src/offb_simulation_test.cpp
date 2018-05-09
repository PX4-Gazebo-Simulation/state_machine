 /**
* @file     : offb_simulation_test.cpp
* @brief    : offboard simulation test: demo rewritten -> 4 setpoints flight -> complete state machine.
* @author   : libn
* @Time     : Aug 25, 201610:06:42 PM
*/

#include <ros/ros.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/TwistStamped.h>			/* local velocity setpoint. -libn */
#include <state_machine/CommandBool.h>
#include <state_machine/SetMode.h>
#include <state_machine/State.h>
#include <state_machine/CommandTOL.h>
#include <state_machine/Setpoint.h>
#include <state_machine/DrawingBoard10.h>

/* subscribe messages from pixhawk. -libn */
#include <state_machine/TASK_STATUS_CHANGE_P2M.h>

void state_machine_func(void);
/* mission state. -libn */
static const int takeoff = 0;
static const int takeoff_hover = 1;
static const int mission_point_A = 2;
static const int mission_point_A_hover_recognize = 3;
static const int mission_search = 4;
static const int mission_scan = 5;
static const int mission_relocate = 6;
static const int mission_operate_move = 7;
static const int mission_operate_hover = 8;
static const int mission_operate_spray = 9;
static const int mission_stop = 10;
static const int mission_fix_failure = 11;
static const int mission_home = 12;
static const int mission_home_hover = 13;
static const int land = 14;
int loop = 1;	/* loop calculator: loop = 1/2/3/4/5. -libn */
// current mission state, initial state is to takeoff
int current_mission_state = takeoff;
ros::Time mission_last_time;	/* timer used in mission. -libn */
bool display_screen_num_recognized = false;	/* to check if the num on display screen is recognized. -libn */
bool relocate_valid = false;	/* to complete relocate mission. -libn */
int mission_failure = 0;

int current_mission_num;	/* mission num: 5 subtask -> 5 current nums.	TODO:change mission num. -libn */

bool velocity_control_enable = false;

ros::Time mission_timer_t;	/* timer to control the whole mission. -libn */
ros::Time loop_timer_t;	/* timer to control subtask. -libn */

state_machine::State current_state;
state_machine::State last_state;
state_machine::State last_state_display;
void state_cb(const state_machine::State::ConstPtr& msg){
	last_state_display.mode = current_state.mode;
	last_state_display.armed = current_state.armed;
	current_state = *msg;
}

state_machine::Setpoint setpoint_indexed;
void SetpointIndexedCallback(const state_machine::Setpoint::ConstPtr& msg)
{
	setpoint_indexed = *msg;
}

// local position msg callback function
geometry_msgs::PoseStamped current_pos;
void pos_cb(const geometry_msgs::PoseStamped::ConstPtr& msg)
{
    current_pos = *msg;
	// ROS_INFO("I got current pos");
}

/* 10 drawing board positions. -libn */
state_machine::DrawingBoard10 board10;
void board_pos_cb(const state_machine::DrawingBoard10::ConstPtr& msg)
{
	board10 = *msg;
}

/* 4 setpoints. -libn */
geometry_msgs::PoseStamped setpoint_A;
geometry_msgs::PoseStamped setpoint_B;
geometry_msgs::PoseStamped setpoint_C;
geometry_msgs::PoseStamped setpoint_D;
geometry_msgs::PoseStamped setpoint_H;	/* home position. -libn */

geometry_msgs::PoseStamped pose_pub;
geometry_msgs::TwistStamped vel_pub;	/* velocity setpoint to be published. -libn */

state_machine::TASK_STATUS_CHANGE_P2M task_status_change_p2m_data;
void task_status_change_p2m_cb(const state_machine::TASK_STATUS_CHANGE_P2M::ConstPtr& msg){
	task_status_change_p2m_data = *msg;

	ROS_INFO("subscribing task_status_change_p2m: %5.3f %d %d",
				task_status_change_p2m_data.spray_duration,
				task_status_change_p2m_data.task_status,
				task_status_change_p2m_data.loop_value);

}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "offb_node");
    ros::NodeHandle nh;

    ros::Subscriber state_sub = nh.subscribe<state_machine::State>
            ("mavros/state", 10, state_cb);
    ros::Publisher local_pos_pub = nh.advertise<geometry_msgs::PoseStamped>
            ("mavros/setpoint_position/local", 10);

    /* Velocity setpoint. -libn */
    ros::Publisher local_vel_pub = nh.advertise<geometry_msgs::TwistStamped>
                ("/mavros/setpoint_velocity/cmd_vel", 10);

    ros::ServiceClient arming_client = nh.serviceClient<state_machine::CommandBool>
            ("mavros/cmd/arming");
    ros::ServiceClient set_mode_client = nh.serviceClient<state_machine::SetMode>
            ("mavros/set_mode");

    // takeoff and land service
    // ros::ServiceClient takeoff_client = nh.serviceClient<mavros_msgs::CommandTOL>("mavros/cmd/takeoff");
    ros::ServiceClient land_client = nh.serviceClient<state_machine::CommandTOL>("mavros/cmd/land");
    state_machine::CommandTOL landing_cmd;
    landing_cmd.request.min_pitch = 1.0;

	/* receive indexed setpoint. -libn */
	ros::Subscriber setpoint_Indexed_sub = nh.subscribe("Setpoint_Indexed", 100 ,SetpointIndexedCallback);
	/* get pixhawk's local position. -libn */
	ros::Subscriber local_pos_sub = nh.subscribe<geometry_msgs::PoseStamped>("mavros/local_position/pose", 10, pos_cb);

	ros::Subscriber DrawingBoard_Position_sub = nh.subscribe<state_machine::DrawingBoard10>
		            ("DrawingBoard_Position10", 10, board_pos_cb);
	board10.drawingboard.resize(10);		/* MUST! -libn */

    //the setpoint publishing rate MUST be faster than 2Hz
    ros::Rate rate(10.0);

    // wait for FCU connection
    while(ros::ok() && current_state.connected){
        ros::spinOnce();
        rate.sleep();
    }

    pose_pub.pose.position.x = 0;
    pose_pub.pose.position.y = 0;
    pose_pub.pose.position.z = 2;
    pose_pub.pose.orientation.x = 0;			/* orientation expressed using quaternion. -libn */
	pose_pub.pose.orientation.y = 0;			/* w = cos(theta/2), x = nx * sin(theta/2),  y = ny * sin(theta/2), z = nz * sin(theta/2) -libn */
	pose_pub.pose.orientation.z = 0.707;
	pose_pub.pose.orientation.w = 0.707;		/* set yaw* = 90 degree(default in simulation). -libn */

	setpoint_H.pose.position.x = 0.0f;	/* pose(x,y) is not used, just for safe. -libn */
	setpoint_H.pose.position.y = 0.0f;
	setpoint_H.pose.position.z = 3.0f;

    //send a few setpoints before starting
    for(int i = 100; ros::ok() && i > 0; --i){
        local_pos_pub.publish(pose_pub);
        ros::spinOnce();
        rate.sleep();
    }

    current_mission_num = 5;

    ros::Time last_request = ros::Time::now();
    ros::Time last_show_request = ros::Time::now();
    mission_timer_t = ros::Time::now();
    loop_timer_t = ros::Time::now();
	mission_last_time = ros::Time::now();
    last_state_display = current_state;
    last_state.mode = current_state.mode;
    last_state.armed = current_state.armed;
    ROS_INFO("current_state.mode = %s",current_state.mode.c_str());
    ROS_INFO("armed status: %d",current_state.armed);

	velocity_control_enable = true;

    while(ros::ok()){
    	/* mode switch display(Once). -libn */
    	if(current_state.mode == "MANUAL" && last_state.mode != "MANUAL")
    	{
    		last_state.mode = "MANUAL";
    		ROS_INFO("switch to mode: MANUAL");
    	}
    	if(current_state.mode == "POSCTL" && last_state.mode != "POSCTL")
    	{
    		last_state.mode = "POSCTL";
    		ROS_INFO("switch to mode: POSCTL");
    	}
    	if(current_state.mode == "OFFBOARD" && last_state.mode != "OFFBOARD")
    	{
    		last_state.mode = "OFFBOARD";
    		ROS_INFO("switch to mode: OFFBOARD");
    	}
    	if(current_state.armed && !last_state.armed)
    	 		{

    		last_state.armed = current_state.armed;
    		ROS_INFO("UAV armed!");
    	}

		// landing
		if(current_state.armed && current_mission_state == land)	/* set landing mode until uav stops. -libn */
		{
			if(current_state.mode != "AUTO.LAND" && (ros::Time::now() - last_request > ros::Duration(5.0)))
			{
				if(land_client.call(landing_cmd) && landing_cmd.response.success)
				{
					ROS_INFO("AUTO LANDING!");
				}
				last_request = ros::Time::now();
			}
		}

		// /* mission state display. -libn */
		// if(current_state.armed){
		// 	ROS_INFO("current_state.armed");
		// }
		
		// show the current pose:
		// ROS_INFO("current quaternion: %f %f %f %f",current_pos.pose.orientation.x,current_pos.pose.orientation.y,
		// current_pos.pose.orientation.z,current_pos.pose.orientation.w);


		// if(current_state.mode == "OFFBOARD" && current_state.armed)	/* set message display delay(0.5s). -libn */
		if(current_state.mode == "OFFBOARD" && current_state.armed)	/* set message display delay(0.5s). -libn */
		{


			//////////////////////////////////////////////////////////////////
			vel_pub.twist.linear.x = 0.0f;
			vel_pub.twist.linear.y = 0.0f;
			vel_pub.twist.linear.z = 2.0f;
			vel_pub.twist.angular.x = 0.0f;
			vel_pub.twist.angular.y = 0.0f;
			vel_pub.twist.angular.z = 0.0f;

			pose_pub.pose.position.x = 5;
			pose_pub.pose.position.y = 5;
			pose_pub.pose.position.z = 2;
			
			
			
			if((abs(current_pos.pose.position.x - current_pos.pose.position.x) < 0.2) &&      // switch to next state
			   (abs(current_pos.pose.position.y - current_pos.pose.position.y) < 0.2) &&
			   (abs(current_pos.pose.position.z - setpoint_H.pose.position.z) < 0.8))
			{
				velocity_control_enable = false;
			}


			if(ros::Time::now() - mission_last_time > ros::Duration(5))	/* hover for 5 seconds. -libn */
			{
				
				if(land_client.call(landing_cmd) && landing_cmd.response.success)
				{
					ROS_INFO("AUTO LANDING!");
				}
			}



			if(velocity_control_enable)
			{
				local_vel_pub.publish(vel_pub);
				ROS_INFO("current vel control");
				mission_last_time = ros::Time::now();
			}
			else
			{
				local_pos_pub.publish(pose_pub);
				ROS_INFO("current pos control");
			}


			ros::spinOnce();
			rate.sleep();
		}
		else{
			if(current_state.mode == "MANUAL") ROS_INFO("MANUAL");
			if(current_state.mode == "OFFBOARD") ROS_INFO("OFFBOARD");
			if(current_state.mode == "AUTO.LAND") ROS_INFO("AUTO.LAND");
			ros::spinOnce();
			rate.sleep();
		}

    }

    return 0;
}


