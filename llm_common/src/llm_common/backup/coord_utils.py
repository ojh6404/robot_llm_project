#!/usr/bin/env python3
import tf
import numpy as np
import yaml
import skrobot
from skrobot_hsr.hsrb import HSRB
from skrobot_hsr.ros.hsrb import HSRBROSRobotInterface

def matrix2quaternion(matrix):
    mat = np.eye(4)
    mat[:3, :3] = matrix
    quat = tf.transformations.quaternion_from_matrix(mat)
    return quat

def coord_to_dict(coord):
    pos = coord.copy_worldcoords().worldpos().tolist()
    rot = coord.copy_worldcoords().quaternion.tolist()
    coord_dict = {"pos": pos, "rot": rot}
    return coord_dict

def dict_to_coord(coord_dict):
    coord = skrobot.coordinates.Coordinates(
        pos = np.array(coord_dict["pos"]),
        rot = np.array(coord_dict["rot"]))
    return coord

def dump_place_dict(file_name, place_dict):
    with open(file_name, 'w') as file:
        yaml.dump(place_dict, file, allow_unicode=True, default_flow_style=None)

def load_place_dict(file_name):
    with open(file_name) as file:
        place_dict = yaml.load(file)
    return place_dict

