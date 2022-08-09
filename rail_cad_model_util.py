import trimesh
import base64
import yaml
import time
import requests

class Yaml_Stl:
    def __init__(self, val):
        self.val = val

def yaml_represent(dumper, data):
    return dumper.represent_scalar('tag:rail,2021:stl', base64.b64encode(data.val).decode('unicode-escape'), style=">")

def read_binary_stl(file_path):
    file = open(file_path, "rb")
    content = file.read()
    return content

def create_stl_representation(robot, rail_dict):
        """takes URDF object and rail reference and generates yaml representations of the links geometry-files and send those to the RAIL-System\n
        rail_dict has format: {"ip":IP, "ref_data": {'ID':base_id, 'linkID':0, 'name':rail_name}}
        """
        print("LOADING FILES AND PARSING TO YAML")
        links = []
        for link in robot.links:
            if link.visuals:
                for visual in link.visuals:
                    if visual.mesh.filename:
                        file = visual.mesh.filename
                        scale = visual.mesh.scale
                        mesh = trimesh.load(file, force='mesh')
                        mesh = mesh.apply_scale(scale)
                        binary_stl = trimesh.exchange.stl.export_stl(mesh)

                        yaml.add_representer(Yaml_Stl, yaml_represent)
                        cad = Yaml_Stl(binary_stl)
                        link = {'ID' : rail_dict['ref_data']['ID'],'linkID' : link.name, 'cad' : cad}
                        print("GEOMETRY ADDED FOR LINK: " + link.name)
            else:
                    print("NO GEOMETRY SPECIFIED FOR LINK: " + link.name)
                    #link = {'ID' : rail_dict['ref_data']['ID'],'linkID' : link.name}
            links.append(link)
        print("SENDING YAML TO RAIL-SYSTEM")
        #create dictionaries according to RAIL-specification
        source = {'ID':rail_dict['ref_data']['ID'], 'linkID' : 0}
        timestamp = int(time.time())
        header = {'source' : source, 'referenceFrame' : 'SELF', 'timestamp' : timestamp, 'action' : "UPDATEORCREATE", 'identity' : ['ID','linkID']}
        payload = {'options' : header, 'data' : links}
        data = yaml.dump(payload, default_flow_style=False)
        r = requests.post(rail_dict["ip"],data = data,headers={'content-type': 'text/x-yaml'})
        print("RAIL RESPONDED: " + str(r.content))