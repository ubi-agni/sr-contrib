/**
 * @file   real_shadowhand.cpp
 * @author Ugo Cupcic <ugo@shadowrobot.com>, Contact <contact@shadowrobot.com>
 * @date   Tue May 25 17:50:42 2010
 *
 * @brief
 *
 *
 */

//our robot code
#include <robot/robot.h>
#include <robot/hand.h>
#include <robot/hand_protocol.h>

#include <ros/ros.h>

#include "sr_hand/hand/real_shadowhand.h"

namespace shadowhand
{
  RealShadowhand::RealShadowhand()
    : Shadowhand()
  {
    /* We need to attach the program to the robot, or fail if we cannot. */
    if (robot_init()<0)
      {
	ROS_FATAL("Robot interface broken\n");
	ROS_BREAK();
      }

    /* We need to attach the program to the hand as well, or fail if we cannot. */
    if (hand_init()<0)
      {
	ROS_FATAL("Hand interface broken\n");
	ROS_BREAK();
      }


    initializeMap();
  }

  RealShadowhand::~RealShadowhand()
  {
  }

  void RealShadowhand::initializeMap()
  {
    JointData tmpData;

    tmpData.position = 0.0;
    tmpData.target = 0.0;
    tmpData.temperature = 0.0;
    tmpData.current = 0.0;
    tmpData.force = 0.0;
    tmpData.flags = "";

    for(unsigned int i=0; i<NUM_HAND_JOINTS + 4; i++)
      {
	std::string name=hand_joints[i].joint_name;	
	tmpData.jointIndex = i;

	joints_map[name] = tmpData;

	ROS_INFO("NAME[%d]: %s ", i, name.c_str());
      }


    parameters_map["d"] = PARAM_d;
    parameters_map["i"] = PARAM_i;
    parameters_map["p"] = PARAM_p;
    parameters_map["target"] = PARAM_target;
    parameters_map["sensor"] = PARAM_sensor;
    
    parameters_map["valve"] = PARAM_valve;
    parameters_map["dead"] = PARAM_deadband;
    parameters_map["deadband"] = PARAM_deadband;
    parameters_map["imax"] = PARAM_imax;
    parameters_map["offset"] = PARAM_output_offset;
    parameters_map["shift"] = PARAM_shift;
    parameters_map["max"] = PARAM_output_max;
    
    //! the parameters for the motors
    parameters_map["motor_maxforce"] = PARAM_motor_maxforce;
    parameters_map["motor_safeforce"] = PARAM_motor_safeforce;
    
    parameters_map["force_p"] = PARAM_force_p;
    parameters_map["force_i"] = PARAM_force_i;
    parameters_map["force_d"] = PARAM_force_d;
    
    parameters_map["force_imax"] = PARAM_force_imax;
    parameters_map["force_out_shift"] = PARAM_force_out_shift;
    parameters_map["force_deadband"] = PARAM_force_deadband;
    parameters_map["force_offset"] = PARAM_force_offset;
    
    parameters_map["sensor_imax"] = PARAM_sensor_imax;
    parameters_map["sensor_out_shift"] = PARAM_sensor_out_shift;
    parameters_map["sensor_deadband"] = PARAM_sensor_deadband;
    parameters_map["sensor_offset"] = PARAM_sensor_offset;
    parameters_map["max_temp"] = PARAM_max_temperature;
    parameters_map["max_temperature"] = PARAM_max_temperature;
    parameters_map["max_current"] = PARAM_max_current;
  }

  short RealShadowhand::sendupdate(std::string joint_name, double target)
  {               
    //search the sensor in the hash_map
    JointsMap::iterator iter = joints_map.find(joint_name);
      
    if(iter != joints_map.end())
      {
	JointData tmpData = joints_map.find(joint_name)->second;
	int index_hand_joints = tmpData.jointIndex;

	//trim to the correct range
	if (target < hand_joints[index_hand_joints].min_angle)
	  target = hand_joints[index_hand_joints].min_angle;
	if (target > hand_joints[index_hand_joints].max_angle)
	  target = hand_joints[index_hand_joints].max_angle;
	  
	//here we update the actual hand angles
	robot_update_sensor(&hand_joints[index_hand_joints].joint_target, target);
	return 0;
      }
    else
      {
	ROS_ERROR("Can't open sensor [%s]", joint_name.c_str());
	return -1;
      }

  }

  JointData RealShadowhand::getJointData(std::string joint_name)
  {
    JointsMap::iterator iter = joints_map.find(joint_name);

    //joint not found
    if(iter == joints_map.end())
      {
	ROS_ERROR("Joint %s not found.", joint_name.c_str());
	JointData noData;
	noData.position = 0.0;
	noData.target = 0.0;
	noData.temperature = 0.0;
	noData.current = 0.0;
	noData.force = 0.0;
	noData.flags = "";
	noData.jointIndex = 0;
	return noData;
      }

    //joint found
    JointData tmpData = iter->second;
    int index = tmpData.jointIndex;

    tmpData.position = robot_read_sensor(&hand_joints[index].position);
    tmpData.target = robot_read_sensor(&hand_joints[index].joint_target);

    //more information
    if (hand_joints[index].a.smartmotor.has_sensors) 
      {
	tmpData.temperature =  robot_read_sensor( &hand_joints[index].a.smartmotor.temperature );
	tmpData.current = robot_read_sensor( &hand_joints[index].a.smartmotor.current );
	tmpData.force = robot_read_sensor( &hand_joints[index].a.smartmotor.torque );
	tmpData.flags = "";
      }

    joints_map[joint_name] = tmpData;


    return tmpData;
    /*

      shadowhand::joints_data msg;
      std::vector<shadowhand::joint> myVector;
  
      sensor_msgs::JointState jointstate_msg;
  
      //get the data from the hand
      for (unsigned int i=0; i<NUM_HAND_JOINTS + 4; ++i)
      {
      shadowhand::joint joint;
      //name
      std::string jointName = hand_joints[i].joint_name;
      joint.joint_name = jointName;
            
      // /!\ don't publish the joint0s to joint_states otherwise
      // it won't match the model	  
      bool isJoint0 = false;
      std::string joint0 = "J0";

      int foundAt = jointName.find(joint0);
      if( foundAt != -1 )
      isJoint0 = true;
      //      if( isJoint0 )
      //	{
      //	  jointstate_msg.effort.push_back(0.0);
      //	  }

      //publish 0 for joint1 effort
      std::string joint1 = "FJ1";
      foundAt = jointName.find(joint1);
      if( foundAt != -1 )
      jointstate_msg.effort.push_back(0.0);
      
      if( !isJoint0 )
      {
      //target & position
      joint.joint_position = robot_read_sensor(&hand_joints[i].position);

      jointstate_msg.name.push_back(hand_joints[i].joint_name);

      jointstate_msg.position.push_back( (robot_read_sensor(&hand_joints[i].position)*3.14159f / 180.0f));

      //WARNING the velocity vector contains the targets... IT's a
      //hack
      //TODO: remove this hack
      jointstate_msg.velocity.push_back((robot_read_sensor(&hand_joints[i].joint_target)*3.14159f / 180.0f));
      }
      joint.joint_target = robot_read_sensor(&hand_joints[i].joint_target);
      
      //more information
      if (hand_joints[i].a.smartmotor.has_sensors) 
      {
      //torque
      joint.joint_torque = robot_read_sensor( &hand_joints[i].a.smartmotor.torque );
      //if( !isJoint0 )
      jointstate_msg.effort.push_back(robot_read_sensor( &hand_joints[i].a.smartmotor.torque ));
	  
      //temperature & current
      joint.joint_temperature = robot_read_sensor(&hand_joints[i].a.smartmotor.temperature);
      joint.joint_current = robot_read_sensor(&hand_joints[i].a.smartmotor.current);
	  
      //check for error_flags
      uint64_t uuid = robot_node_id(hand_joints[i].a.smartmotor.nodename);
      struct hand_protocol_flags fl;
      fl = hand_protocol_get_status_flags(uuid);
      if (fl.valid) 
      {
      struct hand_protocol_flags_smart_motor f;
      f = fl.u.smart_motor;
      //concatenate the flags in a stringstream
      std::stringstream ss;
	      
      bool at_least_one_error_flag = false;
      if( f.nfault_pin )
      {
      at_least_one_error_flag = true;
      ss << "NFAULT ";
      ROS_ERROR( "[%s]: NFAULT", hand_joints[i].joint_name );
      }
      if( f.temperature_cutout )
      {
      at_least_one_error_flag = true;
      ss << "TEMP ";
      }
      if( f.current_throttle )
      {
      at_least_one_error_flag = true;
      ss << "CURRENT ";
      }
      if( f.force_hard_limit )
      {
      at_least_one_error_flag = true;
      ss << "FORCE ";
      }
      if( hand_protocol_dead(uuid) )
      {
      at_least_one_error_flag = true;
      ss << "DEAD ";
      }
	      
      //set the message flags
      joint.error_flag = ss.str();
      //if a flag is up, then print a warning as well
      if( at_least_one_error_flag )
      ROS_WARN( "[%s]: %s", hand_joints[i].joint_name, (ss.str()).c_str());
      }
      }
      
      myVector.push_back(joint);
      }
  
      //set the standard message
      msg.joints_list_length = NUM_HAND_JOINTS;
      msg.joints_list = myVector;
  
      //publish standard data (pos, target, current, temp, force, ...)
      shadowhand_pub.publish(msg);
  
      //publish JointState message
      shadowhand_jointstate_pub.publish(jointstate_msg);
    */
  }

  Shadowhand::JointsMap RealShadowhand::getAllJointsData()
  {
    //update the map for each joints
    for(JointsMap::const_iterator it = joints_map.begin(); it != joints_map.end(); ++it)
      getJointData(it->first);

    //return the map
    return joints_map;
  }

  short RealShadowhand::setContrl(std::string contrlr_name, JointControllerData ctrlr_data)
  {
    struct controller_config myConfig;
    memset(&myConfig, 0, sizeof(myConfig));

    //set the nodename from contrlr_name
    myConfig.nodename = contrlr_name.c_str();

    controller_read_from_hardware(&myConfig);
    ROS_DEBUG("%s", controller_to_string(&myConfig));

    for( unsigned int i = 0; i < ctrlr_data.data.size() ; ++i )
      {
	std::string name = ctrlr_data.data[i].name;
	ParametersMap::iterator iter = parameters_map.find(name);
       
	//parameter not found
	if(iter == parameters_map.end())
	  {
	    ROS_ERROR("Parameter %s not found.", name.c_str());
	    continue;
	  }
       
	//parameter found
	controller_update_param( &myConfig, (controller_param)iter->second, ctrlr_data.data[i].value.c_str() );
      }

    int result_ctrlr = controller_write_to_hardware(&myConfig);
    if (result_ctrlr)
      {
	ROS_ERROR("Failed to update contrlr (%s)", myConfig.nodename );
	return -1;
      }

    return 0;
  }  

  JointControllerData RealShadowhand::getContrl(std::string contrlr_name)
  {
    struct controller_config myConfig;
    memset(&myConfig, 0, sizeof(myConfig));

    //set the nodename from contrlr_name
    myConfig.nodename = contrlr_name.c_str();

    controller_read_from_hardware(&myConfig);
    
    JointControllerData tmp_data;

    ROS_WARN("The get contrlr function is not implemented in the real shadowhand.");

    return tmp_data;

  }

  short RealShadowhand::setConfig(std::vector<std::string> myConfig)
  {
    ROS_WARN("The set config function is not implemented in the real shadowhand.");


    /*
      hand_protocol_config_t cfg;
      hand_protocol_get_config(cfg);

      //set the transmit rate value
      int value = msg->rate_value;
      cfg->u.palm.tx_freq[num]=value;

      //send the config to the palm.
      hand_protocol_set_config(cfg);
    */


    return 0;
  }

  void RealShadowhand::getConfig(std::string joint_name)
  {
    ROS_WARN("The get config function is not yet implement in the real shadow hand.");
  }

/*
  char* ShadowhandDiagnosticer::get_setpoint_name(uint16_t s_num)
  {
    //  struct sensor temp;
    static char name_buff[256];

    snprintf(name_buff, 256, "%s.%d", "smart_motor_setpoints", s_num);

    //TODO: get the sensor name
    if (0==robot_channel_to_sensor("smart_motor_setpoints", s_num, &temp))
      {
      const char *name = robot_sensor_calibration_name(&temp);
      return (char*)name;
      }
    return name_buff;
  }*/

  std::vector<DiagnosticData> RealShadowhand::getDiagnostics()
  {
    std::vector<DiagnosticData> returnVector;
    
    DiagnosticData tmpData;

    //concatenate the flags in a stringstream
    std::stringstream ss;


    //get the data from the hand
    for (unsigned int index=0; index<NUM_HAND_JOINTS + 4; ++index)
      {
	tmpData.joint_name = std::string( hand_joints[index].joint_name );
	tmpData.level = 0;

	tmpData.position = robot_read_sensor(&hand_joints[index].position);
	tmpData.target = robot_read_sensor(&hand_joints[index].joint_target);

	//more information
	if (hand_joints[index].a.smartmotor.has_sensors) 
	  {
	    tmpData.temperature =  robot_read_sensor( &hand_joints[index].a.smartmotor.temperature );
	    tmpData.current = robot_read_sensor( &hand_joints[index].a.smartmotor.current );
	    tmpData.force = robot_read_sensor( &hand_joints[index].a.smartmotor.torque );

	    //check for error_flags
	    uint64_t uuid = robot_node_id(hand_joints[index].a.smartmotor.nodename);
	    struct hand_protocol_flags fl;
	    fl = hand_protocol_get_status_flags(uuid);
	    if (fl.valid) 
	      {
		struct hand_protocol_flags_smart_motor f;
		f = fl.u.smart_motor;

		//empty the stringstream
		ss.str("");
	      
		bool at_least_one_error_flag = false;
		if( f.nfault_pin )
		  {
		    at_least_one_error_flag = true;
		    ss << "NFAULT ";
		    ROS_ERROR( "[%s]: NFAULT", hand_joints[index].joint_name );
		  }
		if( f.temperature_cutout )
		  {
		    at_least_one_error_flag = true;
		    ss << "TEMP ";
		  }
		if( f.current_throttle )
		  {
		    at_least_one_error_flag = true;
		    ss << "CURRENT ";
		  }
		if( f.force_hard_limit )
		  {
		    at_least_one_error_flag = true;
		    ss << "FORCE ";
		  }
		if( hand_protocol_dead(uuid) )
		  {
		    at_least_one_error_flag = true;
		    ss << "DEAD ";
		  }
	      
		//set the message flags
		tmpData.flags = ss.str();
		//if a flag is up, then print a warning as well
		if( at_least_one_error_flag )
		  {
		    ROS_WARN( "[%s]: %s", hand_joints[index].joint_name, (ss.str()).c_str());
		    tmpData.level = 1;
		  }
	      }
	  }
	
	returnVector.push_back(tmpData);
      }

    return returnVector;
  }
} //end namespace