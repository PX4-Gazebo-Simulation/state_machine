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
// #include <state_machine/SetMode.h>
#include <state_machine/State.h>
#include <state_machine/CommandTOL.h>
#include <state_machine/Setpoint.h>
#include <state_machine/DrawingBoard10.h>

#include <state_machine/Thrust.h>
#include <state_machine/AttitudeTarget.h>

#include <state_machine/Input_Game.h>
#include <state_machine/Restart_Finished.h>
#include <state_machine/AttControlRunning.h>
#include <state_machine/Key.h>

/* subscribe messages from pixhawk. -libn */
// #include <state_machine/TASK_STATUS_CHANGE_P2M.h>

// attitude control:
state_machine::AttitudeTarget att_pub;	/* attitude setpoint to be published. -libn */
float theta = 0.0f;						/* attitude: rotation angle. -libn */
float x_axis,y_axis,z_axis;


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

ros::Time mission_timer_t;	/* timer to control the whole mission. -libn */
ros::Time loop_timer_t;	/* timer to control subtask. -libn */

bool Restart_env = false;								// message: DRL controller -> UAV through action!
/* UAV restarting finished message: UAV -> DRL controller */
state_machine::Restart_Finished restart_finished;		// message: UAV -> DRL controller
// restart_finished.finished = false;
state_machine::AttControlRunning att_running;		// message(UAV -> DRL controller): running att_control, ready for DRL memory()!


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

/* input of the game(environment): action */
state_machine::Input_Game input_game;
void input_game_cb(const state_machine::Input_Game::ConstPtr& msg)
{
	input_game = *msg;
	// ROS_INFO("action: %f", input_game.action);
	if (input_game.action > 100.0f)	{Restart_env=true;}
	else {Restart_env=false;}

}

/* input of the game(environment): action */
geometry_msgs::TwistStamped local_vel;
void local_vel_cb(const geometry_msgs::TwistStamped::ConstPtr& msg)
{
	local_vel = *msg;
	ROS_INFO("vel_z: %f", local_vel.twist.linear.z);
	
}

/* keyboard. -libn */
state_machine::Key key;
/*	keyboard:
*	KEY_LEFT=276
*	KEY_RIGHT=275
*	KEY_DOWN=274
*	KEY_UP=273
*	KEY_w=119
*	KEY_s=115
*/
void keyboard_cb(const state_machine::Key::ConstPtr& msg)
{
	key = *msg;
	if(key.code == 276 || key.code == 275)
	{
		if(x_axis != 1.0f)
		{
			theta = 0.0f;
		}
		x_axis = 1.0f;
		y_axis = 0.0f;
		z_axis = 0.0f;
		ROS_INFO("Rotate along x_axis!");
	}
	if(key.code == 273 || key.code == 274)
	{
		if(y_axis != 1.0f)
		{
			theta = 0.0f;
		}
		x_axis = 0.0f;
		y_axis = 1.0f;
		z_axis = 0.0f;
		ROS_INFO("Rotate along y_axis!");
	}
	if(key.code == 97 || key.code == 100)
	{
		if(z_axis != 1.0f)
		{
			theta = 0.0f;
		}
		x_axis = 0.0f;
		y_axis = 0.0f;
		z_axis = 1.0f;
		ROS_INFO("Rotate along z_axis!");
	}

	if(key.code == 273 || key.code == 276 || key.code == 97)
	{
		theta += 0.1f;
	}
	if(key.code == 274 || key.code == 275 || key.code == 100)
	{
		theta -= 0.1f;
	}
	// thrust:
	if(key.code == 119)				// keyboard: w
	{
		att_pub.thrust += 0.03f;
		ROS_INFO("Thrust: %f",att_pub.thrust);
	}
	if(key.code == 115)				// keyboard: s
	{
		att_pub.thrust -= 0.03f;
		ROS_INFO("Thrust: %f",att_pub.thrust);
	}
	ROS_INFO("Rotate angle: theta = %f",theta);

	// attitude control:
	att_pub.orientation.x = x_axis*sinf(theta/2);			/* orientation expressed using quaternion. -libn */
	att_pub.orientation.y = y_axis*sinf(theta/2);			/* w = cos(theta/2), x = nx * sin(theta/2),  y = ny * sin(theta/2), z = nz * sin(theta/2) -libn */
	att_pub.orientation.z = z_axis*sinf(theta/2);
	att_pub.orientation.w = cosf(theta/2);		/* set yaw* = 90 degree(default in simulation). -libn */


	// ROS_INFO("key = %d",key.code);
}

/* 4 setpoints. -libn */
geometry_msgs::PoseStamped setpoint_A;
geometry_msgs::PoseStamped setpoint_B;
geometry_msgs::PoseStamped setpoint_C;
geometry_msgs::PoseStamped setpoint_D;
geometry_msgs::PoseStamped setpoint_H;	/* home position. -libn */

geometry_msgs::PoseStamped pose_pub;
geometry_msgs::TwistStamped vel_pub;	/* velocity setpoint to be published. -libn */
geometry_msgs::TwistStamped attitude_pub; /* attitude setpoint to be published. -libn */
state_machine::Thrust thrust_pub;	/* thrust setpoint to be published. -libn */



// state_machine::TASK_STATUS_CHANGE_P2M task_status_change_p2m_data;
// void task_status_change_p2m_cb(const state_machine::TASK_STATUS_CHANGE_P2M::ConstPtr& msg){
// 	task_status_change_p2m_data = *msg;

// 	ROS_INFO("subscribing task_status_change_p2m: %5.3f %d %d",
// 				task_status_change_p2m_data.spray_duration,
// 				task_status_change_p2m_data.task_status,
// 				task_status_change_p2m_data.loop_value);

// }

state_machine::Key k;

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

    /* Attitude setpoint. -libn */
    ros::Publisher local_attitude_pub = nh.advertise<geometry_msgs::TwistStamped>
                ("/mavros/setpoint_attitude/cmd_vel", 10);

    /* Thrust setpoint. -libn */
    ros::Publisher local_thrust_pub = nh.advertise<state_machine::Thrust>
                ("/mavros/setpoint_attitude/thrust", 10);	

    /* ATT setpoint. -libn */
    ros::Publisher local_att_pub = nh.advertise<state_machine::AttitudeTarget>
                ("/mavros/setpoint_raw/attitude", 10);											

    ros::ServiceClient arming_client = nh.serviceClient<state_machine::CommandBool>
            ("mavros/cmd/arming");
    // ros::ServiceClient set_mode_client = nh.serviceClient<state_machine::SetMode>
    //         ("mavros/set_mode");

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


    /* Action: game_input. -libn */
	ros::Subscriber input_game_sub = nh.subscribe<state_machine::Input_Game>
		("game_input", 10, input_game_cb);

    /* UAV restart_finished_msg. -libn */
    ros::Publisher restart_finished_pub = nh.advertise<state_machine::Restart_Finished>
                ("/restart_finished_msg", 10);
	/* UAV ATT control running. -libn */
	ros::Publisher att_running_pub = nh.advertise<state_machine::AttControlRunning>
				("/att_running_msg", 10);

    /* UAV velocity. -libn */
	ros::Subscriber local_vel_sub = nh.subscribe<geometry_msgs::TwistStamped>
		("/mavros/local_position/velocity", 10, local_vel_cb);			

	ros::Subscriber keyboard_sub = nh.subscribe<state_machine::Key>
		("keyboard/keydown", 10, keyboard_cb);

    //the setpoint publishing rate MUST be faster than 2Hz
    ros::Rate rate(10.0);

    // wait for FCU connection
    while(ros::ok() && current_state.connected){
        ros::spinOnce();
        rate.sleep();
    }

    pose_pub.pose.position.x = 0;
    pose_pub.pose.position.y = 0;
    pose_pub.pose.position.z = 1;
    pose_pub.pose.orientation.x = 0;			/* orientation expressed using quaternion. -libn */
	pose_pub.pose.orientation.y = 0;			/* w = cos(theta/2), x = nx * sin(theta/2),  y = ny * sin(theta/2), z = nz * sin(theta/2) -libn */
	pose_pub.pose.orientation.z = 0.707;
	pose_pub.pose.orientation.w = 0.707;		/* set yaw* = 90 degree(default in simulation). -libn */

	setpoint_H.pose.position.x = 0.0f;	/* pose(x,y) is not used, just for safe. -libn */
	setpoint_H.pose.position.y = 0.0f;
	setpoint_H.pose.position.z = 15.0f;

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

	bool velocity_control_enable = true;
	bool pos_control_enable = false;
	bool att_control_enable = false;
	float error_height = 0.0f;


	// velocity control:
	vel_pub.twist.linear.x = 0.0f;
	vel_pub.twist.linear.y = 0.0f;
	vel_pub.twist.linear.z = 1.0f;
	vel_pub.twist.angular.x = 0.0f;
	vel_pub.twist.angular.y = 0.0f;
	vel_pub.twist.angular.z = 0.0f;

	// position control:
	pose_pub.pose.position.x = 5;
	pose_pub.pose.position.y = 5;
	pose_pub.pose.position.z = 5;

	// attitude control:
	att_pub.thrust = 0.56f;
	att_pub.orientation.x = x_axis*sinf(theta/2);			/* orientation expressed using quaternion. -libn */
	att_pub.orientation.y = y_axis*sinf(theta/2);			/* w = cos(theta/2), x = nx * sin(theta/2),  y = ny * sin(theta/2), z = nz * sin(theta/2) -libn */
	att_pub.orientation.z = z_axis*sinf(theta/2);
	att_pub.orientation.w = cosf(theta/2);		/* set yaw* = 90 degree(default in simulation). -libn */

	att_running.running = false;	// message(UAV -> DRL controller): running att_control, ready for DRL memory()!

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

		// attitude_pub.twist.linear.x = 0.15;
		// attitude_pub.twist.linear.y = 0.15;
		// attitude_pub.twist.linear.z = 0.15;
		// attitude_pub.twist.angular.x = 0.0f;
		// attitude_pub.twist.angular.y = 0.0f;
		// attitude_pub.twist.angular.z = 0.0f;

		// thrust_pub.thrust = 100.0f;

		// att_pub.body_rate.x = 90.0f;
		// att_pub.body_rate.y = 0.0f;
		// att_pub.body_rate.z = 0.0f;



		// att_running.running = true;			// start DRL Memory::observe()!
		att_running_pub.publish(att_running);
		// restart_finished.finished = true;	// send to DRL controller!	//Not used!
		restart_finished_pub.publish(restart_finished);


		// if(current_state.mode == "OFFBOARD" && current_state.armed)	/* set message display delay(0.5s). -libn */
		if(current_state.mode == "OFFBOARD" && current_state.armed)	/* set message display delay(0.5s). -libn */
		{
			ROS_INFO("Current height: %f", current_pos.pose.position.z);
			if(velocity_control_enable)
			{
				local_vel_pub.publish(vel_pub);
				ROS_INFO("velocity_control_enable: %d", velocity_control_enable);
				att_running.running = false;			// stop DRL Memory::observe()!
				restart_finished.finished = false;	// send to DRL controller!
			
				if(velocity_control_enable &&
				(abs(current_pos.pose.position.x - current_pos.pose.position.x) < 0.2) &&      // switch to next state
				(abs(current_pos.pose.position.y - current_pos.pose.position.y) < 0.2) &&
				(abs(current_pos.pose.position.z - setpoint_H.pose.position.z) < 0.8))
				{				
					att_control_enable = true;
					input_game.action = 0.56f;
					pos_control_enable = false;
					velocity_control_enable = false;
				}

			}
		


			if(att_control_enable)
			{
				ROS_INFO("att_control_enable: %d", att_control_enable);
				ROS_INFO("current action: %f", input_game.action);
				// if(input_game.action == 1.0)	{att_pub.thrust += 0.03f;}
				// if(input_game.action == 0.0)	{att_pub.thrust -= 0.03f;}

				/* Limit: 2) vertical velocity limit:[-3,3]. */
				/* discard negtive input when vel.z is large than 3m/s downward; positive input when upward. */
				if(local_vel.twist.linear.z > -3.0)		/* large vertical velocity downward! */
				{
					if(input_game.action == 0.0)	{att_pub.thrust -= 0.03f;}
					
				}
				else ROS_INFO("Positive input is discarded.");
				if(local_vel.twist.linear.z < 3.0)		/* large vertical velocity downward! */
				{
					if(input_game.action == 1.0)	{att_pub.thrust += 0.03f;}
					
				}
				else ROS_INFO("Negtive input is discarded.");

				/* Limit: 1) thrust limit:[0.25,0.78]. */
				if(att_pub.thrust > 0.78)	{att_pub.thrust = 0.78;ROS_INFO("Thrust limit meets");}
				if(att_pub.thrust < 0.40)	{att_pub.thrust = 0.40;ROS_INFO("Thrust limit meets");}

				att_pub.thrust = 0.56;
				ROS_INFO("Thrust: %f", att_pub.thrust);
				local_att_pub.publish(att_pub);

				att_running.running = true;			// start DRL Memory::observe()!
				restart_finished.finished = true;	// send to DRL controller!
				// att_running_pub.publish(att_running);

				if(Restart_env == true)							// switch to next state
				{

					pos_control_enable = true;
					velocity_control_enable = false;
					att_control_enable = false;

					att_running.running = false;			// stop DRL Memory::observe()!
					restart_finished.finished = false;	// send to DRL controller!
					
					// att_running_pub.publish(att_running);
				}
			}

			if(pos_control_enable)		/* we need to recovery now! */
			{	
				/* How to recovery:
				 * 1) velocity recovery if current velocity is too harsh
				 * 2) position recovery to get desired pos.
				 */
				
				if((local_vel.twist.linear.z > 3.0) || (local_vel.twist.linear.z < -3.0))
				{
					local_vel_pub.publish(vel_pub);
					ROS_INFO("Emergency! Vel recovery!");
				}
				else
				{
					pose_pub.pose.position.x = setpoint_H.pose.position.x;
					pose_pub.pose.position.y = setpoint_H.pose.position.y;
					pose_pub.pose.position.z = 15.0f;

					att_running.running = false;			// stop DRL Memory::observe()!
					restart_finished.finished = false;	// send to DRL controller!


					local_pos_pub.publish(pose_pub);
					ROS_INFO("pos_control_enable: %d", pos_control_enable);
					error_height = fabs(current_pos.pose.position.z - pose_pub.pose.position.z);
					ROS_INFO("current_height: %f, desired_height: %f, error: %f", current_pos.pose.position.z, pose_pub.pose.position.z, error_height);
					ROS_INFO("if judgement: %d", (abs(current_pos.pose.position.z - pose_pub.pose.position.z) < 0.1));
					

					restart_finished.finished = false;

					if(fabs(current_pos.pose.position.z - pose_pub.pose.position.z) < 0.1)			// switch to next state
					{
						ROS_INFO("Switching: pos_control -> att_control!");
						att_control_enable = true;
						velocity_control_enable = false;
						pos_control_enable = false;			

						/* restart training from the intial env status */					
						att_pub.thrust = 0.56f;

						// restart_finished.finished = true;	// send to DRL controller!
						// restart_finished_pub.publish(restart_finished);
					}

				}



			}




			// if(ros::Time::now() - mission_last_time > ros::Duration(40))	/* hover for 5 seconds. -libn */
			// {
				
			// 	if(land_client.call(landing_cmd) && landing_cmd.response.success)
			// 	{
			// 		ROS_INFO("AUTO LANDING!");
			// 	}
			// }
			
		    




		}
		else{
			// if(current_state.mode == "MANUAL") ROS_INFO("MANUAL");
			// if(current_state.mode == "OFFBOARD") ROS_INFO("OFFBOARD");
			// if(current_state.mode == "AUTO.LAND") ROS_INFO("AUTO.LAND");
			
		}
		// local_attitude_pub.publish(attitude_pub);
		// local_thrust_pub.publish(thrust_pub);

		ros::spinOnce();
		rate.sleep();

    }

    return 0;
}


