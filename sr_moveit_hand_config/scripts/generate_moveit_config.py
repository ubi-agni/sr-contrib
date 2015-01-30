#!/usr/bin/env python

import sys
import argparse
from srdf_parser_py.srdf import SRDF
import yaml


def yaml_reindent(s, numspaces):
  s_indent = "\n".join((numspaces * " ") + i for i in s.splitlines())
  return s_indent


def generate_fake_controllers(robot, output_path="ompl_planning.yaml"):
    fo = open(output_path, "wb")
    fo.write("controller_list:\n") 
    for group in robot.groups:
      controller_name="  - name: fake_"+group.name+"_controller\n"
      fo.write(controller_name)
      fo.write("    joints:\n")
      if len(group.joints)==0:
          fo.write("      []\n")
      else:
        for joint in group.joints:
            fo.write("      - "+joint.name+"\n")
    fo.close()
            
def generate_ompl_planning(robot,template_path="ompl_planning_template.yaml", output_path="ompl_planning.yaml"):
    fo = open(output_path, "wb")
    stream = open(template_path, 'r')
    yamldoc = yaml.load(stream)
    fo.write("planner_configs:\n")
    fo.write(yaml_reindent(yaml.dump(yamldoc["planner_configs"], default_flow_style=False, allow_unicode=True),2))
    fo.write("\n")
    for group in robot.groups:
      fo.write(group.name+":\n") 
      group_config = yamldoc[group.name]
      group_dump = yaml.dump(group_config, default_flow_style=False, allow_unicode=True)
      fo.write(yaml_reindent(group_dump,2))
      fo.write("\n")
    stream.close()
    fo.close()

def generate_kinematics(robot,template_path="kinematics_template.yaml" , output_path="kinematics.yaml"):
    fo = open(output_path, "wb")
    stream = open(template_path, 'r')
    yamldoc = yaml.load(stream)
    for group in robot.groups:
      if yamldoc.has_key(group.name):
        kinematics_config = yamldoc[group.name]
        fo.write(group.name+":\n") 
        kinematics_dump = yaml.dump(kinematics_config, default_flow_style=False, allow_unicode=True)
        fo.write(yaml_reindent(kinematics_dump,2))
        fo.write("\n")
    stream.close()
    fo.close()

def generate_joint_limits(robot,template_path="joint_limits_template.yaml", output_path="joint_limits.yaml"):
    fo = open(output_path, "wb")
    stream = open(template_path, 'r')
    yamldoc = yaml.load(stream)
    fo.write("joint_limits:\n")
    for joint in robot.group_map["shadow_hand"].joints:
      if yamldoc["joint_limits"].has_key(joint.name):
        joint_limits_config = yamldoc["joint_limits"][joint.name]
        fo.write("  "+joint.name+":\n")
        joint_limits_dump = yaml.dump(joint_limits_config, default_flow_style=False, allow_unicode=True)
        fo.write(yaml_reindent(joint_limits_dump,4))
        fo.write("\n")
    stream.close()
    fo.close()
    
if __name__ == '__main__':
  
  parser = argparse.ArgumentParser(usage='Load an SRDF file')
  parser.add_argument('file', type=argparse.FileType('r'), nargs='?', default=None, help='File to load. Use - for stdin')
  #parser.add_argument('-o', '--output', type=argparse.FileType('w'), default=None, help='Dump file to XML')
  args = parser.parse_args()

  if args.file is not None:
      robot = SRDF.from_xml_string(args.file.read())
      generate_fake_controllers(robot, "fake_controllers.yaml")
      generate_ompl_planning(robot,"ompl_planning_template.yaml")
      generate_kinematics(robot,"kinematics_template.yaml")
      generate_joint_limits(robot,"joint_limits_template.yaml")
  else:
      print ("No file SRDF provided")
      

  #if args.output is not None:
  #    args.output.write(robot.to_xml_string())

      
