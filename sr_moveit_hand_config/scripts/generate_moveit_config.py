#!/usr/bin/env python
#
# author: Guillaume Walck <gwalck@techfak.uni-bielefeld.de>
#

import sys
import argparse
import rosparam
import yaml
import xml
from xml.dom.minidom import parse, parseString

def yaml_reindent(s, numspaces):
  """
  Add numspaces space in fron of each line of the input string
    @param s: input string
    @type s:  str
    @param numspaces: number of spaces to indent the string with
    @type numspaces:  int
    @return s_intend: indented string
  """
  s_indent = "\n".join((numspaces * " ") + i for i in s.splitlines())
  return s_indent

        
def get_joint_names(group):
  joint_names=[]
  joints = group.getElementsByTagName("joint")
  for joint in joints:
    joint_names.append(joint.getAttribute("name"))
  return joint_names


def find_prefix(srdf):
  """
  Find the prefix using the always available shadow_hand group name
    @param robot: parsed SRDF
    @type robot:  SRDF object
    @return prefix: prefix in a string
  """
  prefix=""
  # find the group first
  for group in srdf.getElementsByTagName("group"):
    name=group.getAttribute("name")
    if name.endswith("shadow_hand"):
      prefix=name[0:name.find("shadow_hand")]        
  return prefix
  
def upload_output_params(upload_str,output_path=None,ns_=None):
  """
  Upload or output the input string on the correct param namespace or file
    @param upload_str: string to be uploaded or written
    @type upload_str:  str
    @param output_path: output path for the file to be written. Upload if None
    @type output_path:  str
    @param ns_: namespace to use when uploading to param server
    @type ns_:  str
  """
  if output_path==None:
    paramlist=rosparam.load_str(upload_str,"generated", default_namespace=ns_)
    for params,ns in paramlist:
      rosparam.upload_params(ns, params)
  else:
    fw = open(output_path, "wb")
    fw.write(upload_str)
    fw.close()

def generate_fake_controllers(srdf, output_path=None, ns_=None):
    """
    Generate fake_controller yaml and direct it to file or load it to parameter server.
	    @param srdf: Parsed SRDF
	    @type  srdf: XML object
	    @param output_path: file_path to save the generated data in, will load on parameter server if empty
	    @type  output_path: str
      @param ns_: namespace
	    @type  ns_: str
    """
    output_str=""  
    output_str+="controller_list:\n"
    # for each group
    for group in srdf.getElementsByTagName("group"):
      if group.firstChild:
        controller_name="  - name: fake_"+group.getAttribute("name")+"_controller\n"
        output_str+=controller_name
        output_str+="    joints:\n"
        
        joint_names=get_joint_names(group)
        if len(joint_names)==0:
            output_str+="      []\n"
        else:
          for joint_name in joint_names:
              output_str+="      - "+joint_name+"\n"
    # load on param server or output to file
    upload_output_params(output_str,output_path,ns_)

            
def generate_ompl_planning(srdf,template_path="ompl_planning_template.yaml", output_path=None, ns_=None):
    """
    Generate ompl_planning yaml and direct it to file or load it to parameter server.
	    @param srdf: Parsed SRDF
	    @type  srdf: XML object
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
    # find prefix
    prefix=find_prefix(srdf)
    # for each group
    for group in srdf.getElementsByTagName("group"):
      # strip prefix if any
      group_name=group.getAttribute("name")
      group_name=group_name[len(prefix):]
      if yamldoc.has_key(group_name):
        output_str+=group.getAttribute("name")+":\n"
        group_config = yamldoc[group.getAttribute("name")]
        group_dump = yaml.dump(group_config, default_flow_style=False, allow_unicode=True)
        output_str+=yaml_reindent(group_dump,2)
        output_str+="\n"
    stream.close()
    # load on param server or output to file
    upload_output_params(output_str,output_path,ns_)
    

def generate_kinematics(srdf,template_path="kinematics_template.yaml" , output_path=None, ns_=None):
    """
    Generate kinematics yaml and direct it to file or load it to parameter server.
	    @param srdf: Parsed SRDF
	    @type  srdf: XML object
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
    # find prefix
    prefix=find_prefix(srdf)
    # for each group
    for group in srdf.getElementsByTagName("group"):
      # strip prefix if any
      group_name=group.getAttribute("name")
      group_name=group_name[len(prefix):]
      if yamldoc.has_key(group_name):
        kinematics_config = yamldoc[group.getAttribute("name")]
        output_str+=group.getAttribute("name")+":\n"
        kinematics_dump = yaml.dump(kinematics_config, default_flow_style=False, allow_unicode=True)
        output_str+=yaml_reindent(kinematics_dump,2)
        output_str+="\n"
    stream.close()
    # load on param server or output to file
    upload_output_params(output_str,output_path,ns_)
    

def generate_joint_limits(srdf,template_path="joint_limits_template.yaml", output_path=None, ns_=None):
    """
    Generate joint_limits yaml and direct it to file or load it to parameter server.
	    @param srdf: Parsed SRDF
	    @type  srdf: XML object
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
    # find full hand key name
    for group in srdf.getElementsByTagName("group"):
      name=group.getAttribute("name")
      if name.endswith("shadow_hand"):
        group_name=name
        # for each joint in full hand group
        for full_joint_name in get_joint_names(group):
          joint_name = full_joint_name[-4:]
          #print "testing joint name ",joint_name," out of ",joint.name
          if yamldoc["joint_limits"].has_key(joint_name):
            joint_limits_config = yamldoc["joint_limits"][joint_name]
            output_str+="  "+full_joint_name+":\n"
            joint_limits_dump = yaml.dump(joint_limits_config, default_flow_style=False, allow_unicode=True)
            output_str+=yaml_reindent(joint_limits_dump,4)
            output_str+="\n"
    stream.close()
    # load on param server or output to file
    upload_output_params(output_str,output_path,ns_)
    
if __name__ == '__main__':
  
  parser = argparse.ArgumentParser(usage='Load an SRDF file')
  parser.add_argument('file', type=argparse.FileType('r'), nargs='?', default=None, help='File to load. Use - for stdin')
  args = parser.parse_args()

  if args.file is not None:
      robot = parseString(args.file.read())
      generate_fake_controllers(robot,output_path="fake_controllers.yaml")
      generate_ompl_planning(robot,"ompl_planning_template.yaml", output_path="ompl_planning.yaml")
      generate_kinematics(robot,"kinematics_template.yaml", output_path="kinematics.yaml")
      generate_joint_limits(robot,"joint_limits_template.yaml", output_path="joint_limit.yaml")
  else:
      print ("No file SRDF provided")


      
