from urdfpy import URDF
import rospy
import rospkg
import random
import rail_util
import rail_cad_model_util as rail_cad

def make_pkg_path_absolute(rob_desc):
    '''ros urdfs include relative ros package paths that cannot be handled by urdfpy, so they get replaced by the absolute paths'''
    rospack = rospkg.RosPack()
    while rob_desc.find('package://'):
        start = rob_desc.find('package://')+len('package://')
        stop = rob_desc.find('/',start)
        pkg = rob_desc[start:stop]
        pkg_path = rospack.get_path(pkg)
        pkg_path.replace(pkg,'')
        rob_desc.replace('package://',pkg_path)
    return rob_desc

def get_robot_urdf(description_param='/robot_description'):
    '''get robot description urdf object from parameter server'''
    try:
        rob_desc = rospy.get_param(description_param)
    except:
        print("robot description unavailable, check parameter server")
    #make compatible with urdfpy
    rob_desc = make_pkg_path_absolute(rob_desc)
    #urdfpy cannot read urdf from string so save to file and load to URDF object
    with open('temp.urdf','w') as writer:
        writer.write(rob_desc)
    robot = URDF.load('temp.urdf')
    return robot

def create_rail_dict(ip, robot, anon=False, name=None):
    '''create dictionary with rail reference for creating the robot in rail from urdf object and ip\n
    set anon to True to add individual random number to ID and name to allow for multiple robots of same type\n
    you can specify a custom name, otherwise its the same as the ID both taken from the urdf'''
    if not anon: id = robot.name
    else: id = robot.name + str(random.random(1,1000))
    base_link_id = robot.base_link.name
    if not name: name = id
    rail_dict = {"ip":ip, "ref_data":{'ID':id,'linkID': base_link_id,'name':name}}
    return rail_dict

def initialize_ros2rail(ip, description_param=None, anon=False, name=None):
    robot = get_robot_urdf(description_param)
    rail_dict = create_rail_dict(ip,robot,anon,name)
    rail_util.init_rail(rail_dict)
    links = rail_util.links_from_transform(robot,rail_dict)
    rail_util.send_data_to_rail(links,rail_dict)
    rail_cad.create_stl_representation(robot,rail_dict)
