#!/usr/bin/env python
#
# author: Guillaume Walck <gwalck@techfak.uni-bielefeld.de>
#

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
  syntax  generate_srdf <srdf.xacro filename> [output filename]
  Note : The srdf.xacro filename should be the version without _prefix, 
        as the name with prefix is generated from the one without if needed
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
    # find which version of the hand should be generated (which fingers)
    # also find the prefix if any
    extracted_prefix=False
    prefix=""
    ff=mf=rf=lf=th=ffbio=mfbio=rfbio=lfbio=thbio=False
    is_lite=True
    for key in robot.joint_map:
      #any joint is supposed to have the same prefix and a joint name with 4 chars
      if not extracted_prefix:
        prefix=key[0:-4]
        extracted_prefix=True
        
      if not ff and key.endswith("FFJ4"):
        ff=True
      if not mf and key.endswith("MFJ4"):
        mf=True
      if not rf and key.endswith("RFJ4"):
        rf=True
      if not lf and key.endswith("LFJ4"):
        lf=True
      if not th and key.endswith("THJ4"):
        th=True
      if is_lite and key.endswith("WRJ2"):
        is_lite=False
    print "found fingers (ff mf rf lf th)",(ff, mf, rf, lf, th)
   
    for key in robot.link_map:
      if not ffbio and key.endswith("ffbiotac"):
        ffbio=True
      if not mfbio and key.endswith("mfbiotac"):
        mfbio=True
      if not rfbio and key.endswith("rfbiotac"):
        rfbio=True
      if not lfbio and key.endswith("lfbiotac"):
        lfbio=True
      if not thbio and key.endswith("thbiotac"):
        thbio=True
      
    print "found bio fingers (ff mf rf lf th)",(ffbio, mfbio, rfbio, lfbio, thbio)
    
    #TODO extract the robot name
    # selection the correct hand version through arg substitution
    if len(prefix)>0:
      set_substitution_args_context(load_mappings(['prefix:='+str(prefix),'robot_name:=shadowhand_motor', 
      'ff:='+str(int(ff)),'mf:='+str(int(mf)),'rf:='+str(int(rf)),'lf:='+str(int(lf)),'th:='+str(int(th)),'is_lite:='+str(int(is_lite)), 
      'ff_bio:='+str(int(ffbio)),'mf_bio:='+str(int(mfbio)),'rf_bio:='+str(int(rfbio)),'lf_bio:='+str(int(lfbio)),'th_bio:='+str(int(thbio))]))
      # the prefix version of the srdf_xacro must be loaded
      srdf_xacro_filename = srdf_xacro_filename[0:srdf_xacro_filename.find(".xacro.srdf")]+"_prefix.xacro.srdf"
    else:
      set_substitution_args_context(load_mappings(['robot_name:=shadowhand_motor',
      'ff:='+str(int(ff)),'mf:='+str(int(mf)),'rf:='+str(int(rf)),'lf:='+str(int(lf)),'th:='+str(int(th)),'is_lite:='+str(int(is_lite)), 
      'ff_bio:='+str(int(ffbio)),'mf_bio:='+str(int(mfbio)),'rf_bio:='+str(int(rfbio)),'lf_bio:='+str(int(lfbio)),'th_bio:='+str(int(thbio))]))
    # open and parse the xacro.srdf file
    srdf_xacro_file = open(srdf_xacro_filename, 'r')
    srdf_xacro_xml = parse(srdf_xacro_file)
    
    # expand the xacro
    xacro.process_includes(srdf_xacro_xml, os.path.dirname(sys.argv[0]))
    xacro.eval_self_contained(srdf_xacro_xml)
    
    
    if len(sys.argv) > 2:
      output_path=sys.argv[2]
      if output_path.startswith("_"):
        output_path=None
    else:
      output_path=None
    
    # Upload or output the input string on the correct param namespace or file
    if output_path==None:
      print " Loading SRDF on parameter server"
      rospy.set_param("robot_description_semantic",srdf_xacro_xml.toprettyxml(indent=' '))
    else:
      print " Writing SRDF to file ",output_path
      fw = open(output_path, "wb")
      fw.write(srdf_xacro_xml.toprettyxml(indent=' '))
      fw.close()

    srdf_xacro_file.close()
  else:
    print "No srdf.xacro file provided"
 
