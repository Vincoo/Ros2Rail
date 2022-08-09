import rospy
import argparse
from init import initialize_ros2rail
import rail_util

def rail_callback(data,args):
    """take received joint angles and send changed transformations to RAIL\n
    takes joint_states data, urdf description and rail reference as input"""
    #generate configuration dictionary from data received on topic joint_states
    cfg = {}
    for name, value in zip(data.name, data.position):
        cfg[name:value]
    #generate rail data based on configuration
    robot = args[0]
    rail_dict = args[1]
    links = rail_util.links_from_transform(robot,rail_dict,cfg)
    rail_util.send_data_to_rail(links,rail_dict)

def join_states_listener(robot,rail_dict):
    rospy.init_node("listen_joints2rail",anonymous=True)
    sub = rospy.Subscriber("/joint_states",JointState,rail_callback,(robot, rail_dict))
    rospy.spin()

if __name__ == "__main__":
    parser = argparse.ArgumentParser(description='Specify Connection Parameters')
    parser.add_argument('-r','--rail_ip', help='The IP of the RAIL-System, defaults to http://localhost:8080/api', default='http://localhost:8080/api')
    parser.add_argument('-d', metavar='--description', help='Robot description parameter in parameter server', default='/robot_description')
    parser.add_argument('-n','--robot_name', help='The name of the Robot you want to visualize, defaults from urdf', default=None)
    parser.add_argument('-anon','--anonymous', help='Adds random number to ID and name to allow for multiple of same type', action="store_true")
    parser.add_argument('-temp','--temporary_robot', help='deletes the robot after connection to the robot is lost', action="store_true")
    args = parser.parse_args()
    
    robot, rail_dict = initialize_ros2rail(args.rail_ip,args.description,args.anonymous,args.robot_name)
    try:
        join_states_listener(robot,rail_dict)
    except rospy.ROSInterruptException:
        pass

    if args.temporary_robot:
        rail_util.delete_mechanism(rail_dict)