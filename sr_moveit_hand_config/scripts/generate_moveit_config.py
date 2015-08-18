#!/usr/bin/env python
#
# author: Guillaume Walck <gwalck@techfak.uni-bielefeld.de>
#

import sys
import argparse
import rosparam
import yaml
from srdf_parser_py.srdf import SRDF

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

        
def find_prefix(robot):
  """
  Find the prefix using the always available shadow_hand group name
    @param robot: parsed SRDF
    @type robot:  SRDF object
    @return prefix: prefix in a string
  """
  prefix=""
  for key in robot.group_map:
    if key.endswith("shadow_hand"):
      prefix=key[0:key.find("shadow_hand")]
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
    # for each group
    for group in robot.groups:
      controller_name="  - name: fake_"+group.name+"_controller\n"
      output_str+=controller_name
      output_str+="    joints:\n"
      if len(group.joints)==0:
        output_str+="      []\n"
      else:
        for joint in group.joints:
          output_str+="      - "+joint.name+"\n"
    # load on param server or output to file
    upload_output_params(output_str,output_path,ns_)

            
def generate_ompl_planning(robot,template_path="ompl_planning_template.yaml", output_path=None, ns_=None):
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
    # find prefix
    prefix=find_prefix(robot)
    # for each group
    for group in robot.groups:
      # strip prefix if any
      group_name=group.name[len(prefix):]
      if yamldoc.has_key(group_name):
        output_str+=group.name+":\n"
        group_config = yamldoc[group_name]
        group_dump = yaml.dump(group_config, default_flow_style=False, allow_unicode=True)
        output_str+=yaml_reindent(group_dump,2)
        output_str+="\n"
    stream.close()
    # load on param server or output to file
    upload_output_params(output_str,output_path,ns_)
    

def generate_kinematics(robot,template_path="kinematics_template.yaml" , output_path=None, ns_=None):
    """
    Generate kinematics yaml and direct it to file or load it to parameter server.
	    @param srdf: Parsed SRDF
	    @type  srdf: XML object
      @param template_path: file_path to the required yaml template file (biotac version will be loaded automatically)
	    @type  template_path: str
	    @param output_path: file_path to save the generated data in, will load on parameter server if empty
	    @type  output_path: str
      @param ns_: namespace
	    @type  ns_: str
    """
    output_str="" 
    # open template file
    stream = open(template_path, 'r')
    yamldoc = yaml.load(stream)
    # open biotac template file
    biotac_template_path = template_path[0:template_path.find("_template")]+"_biotac_template.yaml"
    biotac_stream = open(biotac_template_path, 'r')
    yamldocbiotac = yaml.load(biotac_stream)
    
    # find prefix
    prefix=find_prefix(robot)

    # find full hand key name
    sh_group=None
    for group in robot.groups:
      if group.name.endswith("shadow_hand"):
        sh_group=group
        break
        
    # detect biotac fingers
    is_bio={"first_finger": False,
            "middle_finger": False,
            "ring_finger": False,
            "little_finger": False,
            "thumb": False}

    for mylink in sh_group.links:
      link_name = mylink.name
      if not is_bio["first_finger"] and link_name.endswith("ffbiotac"):
        is_bio["first_finger"]=True
      if not is_bio["middle_finger"] and link_name.endswith("mfbiotac"):
        is_bio["middle_finger"]=True
      if not is_bio["ring_finger"] and link_name.endswith("rfbiotac"):
        is_bio["ring_finger"]=True
      if not is_bio["little_finger"] and link_name.endswith("lfbiotac"):
        is_bio["little_finger"]=True
      if not is_bio["thumb"] and link_name.endswith("thbiotac"):
        is_bio["thumb"]=True
    
    # for each group
    for group in robot.groups:
      kinematics_config=None
      # strip prefix if any
      group_name=group.name[len(prefix):]
      # check for biotac link for this group
      is_group_bio = is_bio.get(group_name)
      if is_group_bio:
        if yamldocbiotac.has_key(group_name):
          kinematics_config = yamldocbiotac[group_name]
      else:
        if yamldoc.has_key(group_name):
          kinematics_config = yamldoc[group_name]
          
      if kinematics_config !=None:
        output_str+=group.name+":\n"
        kinematics_dump = yaml.dump(kinematics_config, default_flow_style=False, allow_unicode=True)
        output_str+=yaml_reindent(kinematics_dump,2)
        output_str+="\n"
        
        
    stream.close()
    biotac_stream.close()
    # load on param server or output to file
    upload_output_params(output_str,output_path,ns_)
    

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
    # find full hand key name
    for key in robot.group_map:
      if key.endswith("shadow_hand"):
        group_name=key
    
    if group_name!=None:
      # for each joint in full hand group
      for joint in robot.group_map[group_name].joints:
        joint_name = joint.name[-4:]
        #print "testing joint name ",joint_name," out of ",joint.name
        if yamldoc["joint_limits"].has_key(joint_name):
          joint_limits_config = yamldoc["joint_limits"][joint_name]
          output_str+="  "+joint.name+":\n"
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
      robot = SRDF.from_xml_string(args.file.read())
      generate_fake_controllers(robot,output_path="fake_controllers.yaml")
      generate_ompl_planning(robot,"ompl_planning_template.yaml", output_path="ompl_planning.yaml")
      generate_kinematics(robot,"kinematics_template.yaml", output_path="kinematics.yaml")
      generate_joint_limits(robot,"joint_limits_template.yaml", output_path="joint_limit.yaml")
  else:
      print ("No file SRDF provided")


      
