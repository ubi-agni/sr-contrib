#!/usr/bin/env python

import sys
import time
import rospy
from generate_moveit_config import * 
from srdf_parser_py.srdf import SRDF



if __name__ == '__main__':
  """ 
  generate and load one the following moveit yaml config file based on srdf on parameter server
  syntax  generate_load_moveit_config command [template_file]
  fake_controllers (no template)
  ompl_planning (template required)
  kinematics (template required)
  joint_limits (template required)
  """
  # detect the command to be executed
  if len(sys.argv) > 1:
    command = sys.argv[1]
    if command in ['fake_controllers', 'ompl_planning', 'kinematics', 'joint_limits']:
      ns = rospy.get_namespace()
      # wait for parameters
      while not rospy.has_param('/robot_description_semantic'):
        time.sleep(0.5)
        print "waiting for robot_description_semantic"
      # load the srdf from the parameter server
      srdf_str=rospy.get_param('/robot_description_semantic')   
      # parse it
      robot = SRDF.from_xml_string(srdf_str)
      # generate the desired yaml and load it.
      if command == "fake_controllers":
        generate_fake_controllers(robot,ns_=ns)
      elif command == "ompl_planning":
        # get the template file
        if len(sys.argv) > 2:
          template_path=sys.argv[2]
          generate_ompl_planning(robot,template_path=template_path,ns_=ns)
        else:
          print "ompl_planning generation requires a template file, none provided"
      elif command == "kinematics":
        # get the template file
        if len(sys.argv) > 2:
          template_path=sys.argv[2]
          generate_kinematics(robot,template_path=template_path,ns_=ns)
        else:
          print "kinematics generation requires a template file, none provided"
          sys.exit(1)
      elif command == "joint_limits":
        # get the template file
        if len(sys.argv) > 2:
          template_path=sys.argv[2]
          generate_joint_limits(robot,template_path=template_path,ns_=ns)
        else:
          print "joint_limits generation requires a template file, none provided"  
      else:
        print "wrong argument "+command
   
      print "Successfully loaded "+command+ " params"
    else:
      print "Unrecognized command "+command+". Choose among fake_controllers, ompl_planning, kinematics joint_limits"
  else:
    print "Argument needed. Choose among fake_controllers, ompl_planning, kinematics joint_limits"
