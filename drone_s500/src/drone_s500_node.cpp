#include <ros/ros.h>
#include <geometry_msgs/PoseStamped.h>
#include <mavros_msgs/CommandBool.h>
#include <mavros_msgs/SetMode.h>
#include <mavros_msgs/State.h>

mavros_msgs::State current_state;
void state_cb(const mavros_msgs::State::ConstPtr& msg){
    current_state = *msg;
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "offb_node");
    ros::NodeHandle nh;
    
    ros::Subscriber state_sub = nh.subscribe<mavros_msgs::State>("mavros/state", 10, state_cb);
    ros::Publisher local_pos_pub = nh.advertise<geometry_msgs::PoseStamped>("mavros/setpoint_position/local", 10);
    ros::ServiceClient arming_client = nh.serviceClient<mavros_msgs::CommandBool>("mavros/cmd/arming");
    ros::ServiceClient set_mode_client = nh.serviceClient<mavros_msgs::SetMode>("mavros/set_mode");

    //the setpoint publishing rate MUST be faster than 2Hz
    ros::Rate rate(20.0);

    // wait for FCU connection
    while (ros::ok() && !current_state.connected){
    	ros::spinOnce();
	rate.sleep();
    }
    ROS_INFO("Flight Control Unit connected");
   
    geometry_msgs::PoseStamped target_pose;
    target_pose.pose.position.x = 0;
    target_pose.pose.position.y = 0;
    target_pose.pose.position.z = 2.0;
    
    // send a few setpoints before starting
    for(int i = 100; ros::ok() && i > 0; --i){
	local_pos_pub.publish(target_pose);
	ros::spinOnce();
	rate.sleep();    
   }

   // initialize set mode object for offboard mode setting
   mavros_msgs::SetMode offb_set_mode;
   offb_set_mode.request.custom_mode = "OFFBOARD";
   
   // initialize set mode object for auto land mode setting
   mavros_msgs::SetMode land_set_mode;
   land_set_mode.request.custom_mode = "AUTO.LAND";

   mavros_msgs::CommandBool arm_cmd;
   arm_cmd.request.value = true;

   mavros_msgs::CommandBool disarm_cmd;
   disarm_cmd.request.value = false;


   ros::Time last_request = ros::Time::now();

    while(ros::ok()){
        if( current_state.mode != "AUTO.LAND" && current_state.mode != "OFFBOARD" && (ros::Time::now() - last_request > ros::Duration(3.0))){
            if( set_mode_client.call(offb_set_mode) && offb_set_mode.response.mode_sent){
                ROS_INFO("Offboard enabled");
            }
            last_request = ros::Time::now();
        } 

	else if( current_state.mode == "OFFBOARD" && !current_state.armed && (ros::Time::now() - last_request > ros::Duration(3.0)) ){
		if( arming_client.call(arm_cmd) && arm_cmd.response.success){
                    ROS_INFO("Vehicle armed");
                }
                last_request = ros::Time::now();
	}

	else if( current_state.mode != "AUTO.LAND" && current_state.armed && (ros::Time::now() - last_request > ros::Duration(25.0))){
		if( set_mode_client.call(land_set_mode) && land_set_mode.response.mode_sent){
                    ROS_INFO("Vehicle landing");
                }
                last_request = ros::Time::now();
	}	

	else {
            if( current_state.mode == "AUTO.LAND" && current_state.armed && (ros::Time::now() - last_request > ros::Duration(5.0))){
                if( arming_client.call(disarm_cmd) && disarm_cmd.response.success){
                    ROS_INFO("Vehicle disarmed");
                }
                last_request = ros::Time::now();
            }
        }

        local_pos_pub.publish(target_pose);

        ros::spinOnce();
        rate.sleep();
    }

    return 0;

}
