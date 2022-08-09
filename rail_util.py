import yaml
import requests
import time
import numpy as np

def init_rail(rail_dict):
        """initialize base frame in rail from info in dict \n
        rail_dict has format: rail_dict = {"ip":IP, "ref_data": {'ID':robot_ID, 'linkID':base_link, 'name':rail_name}}
        """
        #quite ofte RAIL is either forgotten to start in a docker container
        #or is not available at all. Therefore the following gives oportunity
        #to start RAIL when programm wants to connect to it or to ignore it
        #completely
        try_connect = True
        while try_connect == True:
            try:
                #initialize base frame
                ip = rail_dict['ip']
                data = rail_dict['ref_data']
                data['transformation':list(np.eye(4))]
                timestamp = int(time.time())
                header = {'referenceFrame' : 'SELF', 'timestamp' : timestamp, 'action' : "UPDATEORCREATE", 'identity' : ['ID','linkID']}
                payload = {'options' : header, 'data' : data}
                r = requests.post(ip,json = payload)
                rail_ignored = False
                try_connect = False
                print("RAIL RESPONDED: " + str(r.content))
            except Exception:
                print('Could not connect to RAIL')
                checked = False
                while checked == False:
                    check = input('Do you want to proceed without RAIL? [y/n]')
                    if check == 'n':
                        print('trying again to connect to RAIL')
                        checked = True
                    elif check == 'y':
                        print('do not bother with RAIL')
                        checked = True
                        try_connect = False
                        rail_ignored = True
                    else:
                        print('input y or n')
        return rail_ignored

def links_from_transform(robot,rail_dict,cfg=None):
    '''takes a URDF object, the rail reference and optionally a joint configuration as input and generates links to be send to RAIL
    '''
    fk = robot.link_fk(cfg)
    links = []
    #The base link is excluded so that its transformation is not updated, this allows for overwriting the base links position externally 
    for link in robot.links[1:]:
        link = {'ID' : rail_dict['ref_data']['ID'], 'linkID':link.name, 'transformation' : list(fk[link])}
        links.append(link)
    return links #returns i to allow for adding more Links after execution with using i as input

def send_data_to_rail(links, rail_dict):
    '''takes list of links as input, constructs rail messsage and sends all transformation data to the rail at once'''
    #create dictionaries according to RAIL-specification
    source = {'ID' : rail_dict['ref_data']['ID']}
    timestamp = int(time.time())
    header = {'source' : source, 'referenceFrame' : 'SELF', 'timestamp' : timestamp, 'action' : "UPDATEORCREATE", 'identity' : ['ID','linkID']}
    payload = {'options' : header, 'data' : links}
    #send data to rail
    r = requests.post(rail_dict['ip'],json = payload)
    print("RAIL RESPONDED: " + str(r.content))

def delete_mechanism(rail_dict):
        remains_left = True
        while remains_left:
            print("REQUEST DELETION IN RAIL")
            timestamp = int(time.time())
            header = {'timestamp' : timestamp, 'action' : "DELETE", 'identity' : ['ID','linkID']}
            target = {'ID' : rail_dict['ref_data']['ID']}
            payload = {'options' : header, 'data' : target}
            r = requests.post(rail_dict['ip'],json = payload)
            print("RAIL RESPONDED: " + str(r.content))
            r_dict = yaml.safe_load(r.content)
            status = r_dict["status"]
            if status["status"] == 1:
                print("OBJECT DELETED")
            else:
                print("STATUS NOT 1")
                print("STOPPING DELETION")
                remains_left = False