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
    srdf_xacro_filename = sys.argv[1]
    
    while not rospy.has_param('/robot_description'):
      rospy.sleep(0.5)
      print "waiting for robot_description"
      
    # load the srdf from the parameter server
    urdf_str=rospy.get_param('/robot_description')   
    # parse it
    robot = URDF.from_xml_string(urdf_str)
    ff=mf=rf=lf=th=False
    is_lite=True
    for key in robot.joint_map:
      #print "joint_map key:",key
      if not ff and key.endswith("FFJ4"):
        ff=True
        prefix=key[0:-4]
      if not mf and key.endswith("MFJ4"):
        mf=True
      if not rf and key.endswith("RFJ4"):
        rf=True
      if not lf and key.endswith("RFJ4"):
        lf=True
      if not th and key.endswith("THJ4"):
        th=True
      if is_lite and key.endswith("WRJ2"):
        is_lite=False
    #print "found ",ff, mf, rf, lf, th, is_lite 
    print "prefix ",prefix
    if len(prefix)>0:
      set_substitution_args_context(load_mappings(['prefix:='+str(prefix),'robot_name:=shadowhand_motor','ff:='+str(int(ff)),'mf:='+str(int(mf)),'rf:='+str(int(rf)),'lf:='+str(int(lf)),'th:='+str(int(th)),'is_lite:='+str(int(is_lite))]))
      # the prefix version of the srdf_xacro must be loaded
      srdf_xacro_filename = srdf_xacro_filename[0:srdf_xacro_filename.find(".xacro.srdf")]+"_prefix.xacro.srdf"
    else:
      set_substitution_args_context(load_mappings(['robot_name:=shadowhand_motor','ff:='+str(int(ff)),'mf:='+str(int(mf)),'rf:='+str(int(rf)),'lf:='+str(int(lf)),'th:='+str(int(th)),'is_lite:='+str(int(is_lite))]))
    
    srdf_xacro_file = open(srdf_xacro_filename, 'r')
    srdf_xacro_xml = parse(srdf_xacro_file)
    
    xacro.process_includes(srdf_xacro_xml, os.path.dirname(sys.argv[0]))
    xacro.eval_self_contained(srdf_xacro_xml)
    
    rospy.set_param("robot_description_semantic",srdf_xacro_xml.toprettyxml(indent=' '))
    srdf_xacro_file.close()
  else:
    print "No srdf.xacro file provided"
 
