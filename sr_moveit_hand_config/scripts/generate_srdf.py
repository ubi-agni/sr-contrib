#!/usr/bin/env python

import sys
import os
from urdf_parser_py.urdf import URDF
import xacro
import rospy
import xml
from xml.dom.minidom import parse
from xacro import set_substitution_args_context
from rosgraph.names import load_mappings

if __name__ == '__main__':
  """ 
  generate the srdf according to the urdf
  syntax  generate_srdf
  """
  if len(sys.argv) > 1:
    srdf_xacro_file = sys.argv[1]
    srdf_xacro_file = open(srdf_xacro_file, 'r')
  
    while not rospy.has_param('/robot_description'):
      rospy.sleep(0.5)
      print "waiting for robot_description"
      
    # load the srdf from the parameter server
    urdf_str=rospy.get_param('/robot_description')   
    # parse it
    robot = URDF.from_xml_string(urdf_str)
    ff=robot.joint_map.has_key("FFJ4")
    mf=robot.joint_map.has_key("MFJ4")
    rf=robot.joint_map.has_key("RFJ4")
    lf=robot.joint_map.has_key("RFJ4")
    th=robot.joint_map.has_key("THJ4")
    is_lite=not robot.joint_map.has_key("WRJ2")
    #print "found ",ff, mf, rf, lf, th, is_lite 
    set_substitution_args_context(load_mappings(['robot_name:=shadowhand_motor','ff:='+str(int(ff)),'mf:='+str(int(mf)),'rf:='+str(int(rf)),'lf:='+str(int(lf)),'th:='+str(int(th)),'is_lite:='+str(int(is_lite))]))

    srdf_xacro_xml = parse(srdf_xacro_file)
    xacro.process_includes(srdf_xacro_xml, os.path.dirname(sys.argv[0]))
    xacro.eval_self_contained(srdf_xacro_xml)
    
    rospy.set_param("robot_description_semantic",srdf_xacro_xml.toprettyxml(indent=' '))
    srdf_xacro_file.close()
  else:
    print "No srdf.xacro file provided"
 
