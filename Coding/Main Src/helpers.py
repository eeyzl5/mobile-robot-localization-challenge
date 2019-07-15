"""
Helper File: write in this module all of the functions that 
do not need to be in the class. All the functions that do not need to change
a state can be added here. 
E.g. conversion of numbers, concatenation, split.

I would suggest to write here all of the image processing functions
"""

import cv2
import math

from constants import *

def cal_angle(segx, segy, dir):
    num_rows = 420
    num_col = 320
    dir_ang = 0
    x1 = 400
    y1 = 170

    est_loc = arena_map(segx, segy)
    x = est_loc[0]
    y = est_loc[1]
    print('est x {}:'.format(x))
    print('est y {}:'.format(y))
    angle_vert = math.floor (math.degrees ( math.atan2(x1-x,y1-y)))
    #v_ang= 300 / math.sqrt( ( (x1-x)*(x1-x) )+ ( (y1-y) * (y1-y) ) )
    angle_hor = math.degrees ( math.atan2(300, math.sqrt( ( (x1-x)*(x1-x) )+ ( (y1-y) * (y1-y) ) ))  )
    print('vertical angle: {}.'.format(angle_vert))
    print('horizontal angle: {}'.format(angle_hor))

    if dir == "north":
        dir_ang = math.floor((angle_vert)/20)
    if dir == "south":
        dir_ang = math.floor((angle_vert+180)/20)
    if dir == "east":
        dir_ang = math.floor((angle_vert+90)/20)
    if dir == "west":
        dir_ang = math.floor((angle_vert+270)/20)
    print('normalized angle {}'.format(dir_ang))
    estimated_angle = [dir_ang , angle_hor]
    return estimated_angle


def arena_map(x,y):
    seg_x = 85
    seg_y = 64
    #arena  = [[0 for x in range(5)] for j in range(5)]
    loc_seg_x = math.floor((x-1)*85 + (85/2))
    loc_seg_y = math.floor((y-1)*64 + (64/2))
    pos_est = [loc_seg_x, loc_seg_y]
    return pos_est 