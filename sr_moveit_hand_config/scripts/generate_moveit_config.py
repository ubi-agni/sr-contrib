#!/usr/bin/env python

import sys
import argparse
from srdf_parser_py.srdf import SRDF
import rosparam
import yaml

def yaml_reindent(s, numspaces):
  s_indent = "\n".join((numspaces * " ") + i for i in s.splitlines())
  return s_indent

def generate_fake_controllers(robot, output_path=None, ns_=None):
    """
    Generate fake_controller yaml and direct it to file or load it to parameter server.
	    @param robot: Parsed SRDF
	    @type  robot: SRDF object
	    @param output_path: file_path to save the generated data in, will load on parameter server if empty
	    @type  output_path: str
      @param ns_: namespace
	    @type  ns_: str
    """
    output_str=""  
    output_str+="controller_list:\n"
    for group in robot.groups:
      controller_name="  - name: fake_"+group.name+"_controller\n"
      output_str+=controller_name
      output_str+="    joints:\n"
      if len(group.joints)==0:
          output_str+="      []\n"
      else:
        for joint in group.joints:
            output_str+="      - "+joint.name+"\n"
    
    if output_path==None:
      paramlist=rosparam.load_str(output_str,"generated", default_namespace=ns_)
      for params,ns in paramlist:
        rosparam.upload_params(ns, params)
    else:
      fw = open(output_path, "wb")
      fw.write(output_str)
      fw.close()

            
def generate_ompl_planning(robot,template_path="ompl_planning_template.yaml", output_path="None", ns_=None):
    """
    Generate ompl_planning yaml and direct it to file or load it to parameter server.
	    @param robot: Parsed SRDF
	    @type  robot: SRDF object
      @param template_path: file_path to the required yaml template file
	    @type  template_path: str
	    @param output_path: file_path to save the generated data in, will load on parameter server if empty
	    @type  output_path: str
      @param ns_: namespace
	    @type  ns_: str
    """
    output_str=""  
    
    stream = open(template_path, 'r')
    yamldoc = yaml.load(stream)
    output_str+="planner_configs:\n"
    output_str+=yaml_reindent(yaml.dump(yamldoc["planner_configs"], default_flow_style=False, allow_unicode=True),2)
    output_str+="\n"
    for group in robot.groups:
      output_str+=group.name+":\n"
      group_config = yamldoc[group.name]
      group_dump = yaml.dump(group_config, default_flow_style=False, allow_unicode=True)
      output_str+=yaml_reindent(group_dump,2)
      output_str+="\n"
    stream.close()
    
    if output_path==None:
      paramlist=rosparam.load_str(output_str,"generated", default_namespace=ns_)
      for params,ns in paramlist:
        rosparam.upload_params(ns, params)
    else:
      fw = open(output_path, "wb")
      fw.write(output_str)
      fw.close()
    

def generate_kinematics(robot,template_path="kinematics_template.yaml" , output_path=None, ns_=None):
    """
    Generate kinematics yaml and direct it to file or load it to parameter server.
	    @param robot: Parsed SRDF
	    @type  robot: SRDF object
      @param template_path: file_path to the required yaml template file
	    @type  template_path: str
	    @param output_path: file_path to save the generated data in, will load on parameter server if empty
	    @type  output_path: str
      @param ns_: namespace
	    @type  ns_: str
    """
    output_str="" 
    stream = open(template_path, 'r')
    yamldoc = yaml.load(stream)
    for group in robot.groups:
      if yamldoc.has_key(group.name):
        kinematics_config = yamldoc[group.name]
        output_str+=group.name+":\n"
        kinematics_dump = yaml.dump(kinematics_config, default_flow_style=False, allow_unicode=True)
        output_str+=yaml_reindent(kinematics_dump,2)
        output_str+="\n"
    stream.close()
    if output_path==None:
      paramlist=rosparam.load_str(output_str,"generated", default_namespace=ns_)
      for params,ns in paramlist:
        rosparam.upload_params(ns, params)
    else:
      fw = open(output_path, "wb")
      fw.write(output_str)
      fw.close()    
    

def generate_joint_limits(robot,template_path="joint_limits_template.yaml", output_path=None, ns_=None):
    """
    Generate joint_limits yaml and direct it to file or load it to parameter server.
	    @param robot: Parsed SRDF
	    @type  robot: SRDF object
      @param template_path: file_path to the required yaml template file
	    @type  template_path: str
	    @param output_path: file_path to save the generated data in, will load on parameter server if empty
	    @type  output_path: str
      @param ns_: namespace
	    @type  ns_: str
    """
    output_str=""
    stream = open(template_path, 'r')
    yamldoc = yaml.load(stream)
    output_str+="joint_limits:\n"
    for joint in robot.group_map["shadow_hand"].joints:
      if yamldoc["joint_limits"].has_key(joint.name):
        joint_limits_config = yamldoc["joint_limits"][joint.name]
        output_str+="  "+joint.name+":\n"
        joint_limits_dump = yaml.dump(joint_limits_config, default_flow_style=False, allow_unicode=True)
        output_str+=yaml_reindent(joint_limits_dump,4)
        output_str+="\n"
    stream.close()
    if output_path==None:
      paramlist=rosparam.load_str(output_str,"generated", default_namespace=ns_)
      for params,ns in paramlist:
        rosparam.upload_params(ns, params)
    else:
      fw = open(output_path, "wb")
      fw.write(output_str)
      fw.close()
    
if __name__ == '__main__':
  
  parser = argparse.ArgumentParser(usage='Load an SRDF file')
  parser.add_argument('file', type=argparse.FileType('r'), nargs='?', default=None, help='File to load. Use - for stdin')
  #parser.add_argument('-o', '--output', type=argparse.FileType('w'), default=None, help='Dump file to XML')
  args = parser.parse_args()

  if args.file is not None:
      robot = SRDF.from_xml_string(args.file.read())
      generate_fake_controllers(robot,output_path="fake_controllers.yaml")
      generate_ompl_planning(robot,"ompl_planning_template.yaml", output_path="ompl_planning.yaml")
      generate_kinematics(robot,"kinematics_template.yaml", output_path="kinematics.yaml")
      generate_joint_limits(robot,"joint_limits_template.yaml", output_path="joint_limit.yaml")
  else:
      robot = SRDF.from_xml_string(args.file.read())
      print ("No file SRDF provided")


      
