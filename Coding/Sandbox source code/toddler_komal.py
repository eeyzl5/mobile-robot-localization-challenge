#!/usr/bin/env python

import time
import math


class Toddler:
    __version = '2018a'

    def __init__(self, IO):
        print('[Toddler] I am toddler {} playing in a sandbox'.format(Toddler.__version))

        self.camera = IO.camera.initCamera('pi', 'low')
        self.getInputs = IO.interface_kit.getInputs
        self.getSensors = IO.interface_kit.getSensors
        self.mc = IO.motor_control
        self.sc = IO.servo_control

        self.hall_init()
        self.poi_init()
        self.odometer = 0

    def control(self):
        self.odometer = self.hall_counter()
#        print('{}\t{}'.format("Hall Counter: ",self.hcounter))
#        print('{}\t{}'.format("Odometer: ",self.odometer))
#        print('{}\t{}'.format(self.getSensors(), self.getInputs()))
#        print('{}\t{}'.format("IR sensor: ",self.getSensors()[6]))


#        self.poi_detect()
#        self.set_move(10)
        vals_t = self.cal_angle( 1,5,"east")
#        self.set_turn("left",vals_t[0])
	self.sc.engage()
        self.sc.setPosition(vals_t[1])
#	self.cal_angle( 200,300,"north")
#        self.set_turn("left",4)

#        self.sc.engage()
#        self.sc.setPosition(0 if self.getSensors()[0] >= 500 else 180)

#        time.sleep(0.05)

    def vision(self):
#        image = self.camera.getFrame()
#        self.camera.imshow('Camera', image)
        time.sleep(0.05)



###################################################
#                Hall Counter                     #
###################################################

    def hall_init(self):
        self.hcounter = 0
        self.mc.stopMotors() # Stop the motor and get the first samples of hall sensors
        self.hall_now = self.getInputs()[7]
        time.sleep(0.5)
        self.hall_pre = self.getInputs()[7]

    def hall_counter(self):
        self.hall_now = self.getInputs()[7]
        self.diff = self.hall_now - self.hall_pre
        self.hall_pre = self.hall_now
        if self.diff != 0:
            self.hcounter += 1
        return (self.hcounter)*2.5 +2.2 # return the extact distance in cm

####################################################
#                POI detection                     #
####################################################

    def poi_init(self):
        self.flag = 0
        self.mc.setMotor(1,100) # turn on the light bulb

    def poi_detect(self):
        # both of the front light sensors should be on POI first
        if self.getSensors()[0] > 40 and self.getSensors()[1] > 40:
            self.flag = 1 # set a flag if so

        if (self.getSensors()[2] > 15 or (self.getSensors()[0] <40 and self.getSensors()[1] < 40)) and self.flag == 1:
            self.mc.stopMotors()

        if self.flag != 1:
            self.mc.setMotor(2,100)
            self.mc.setMotor(4,100)


#####################################################
#               Close loop Controls                 #
#####################################################


    def set_move(self, number_of_counts):
        number_of_counts = number_of_counts - (1 if self.hcounter < 5 else 2)
        if self.hcounter > number_of_counts:
            self.mc.stopMotors()
        else:
            self.mc.setMotor(2,100)
            self.mc.setMotor(4,100)

    def set_turn(self, direction, number_of_counts):
        if direction == "left":
            direction = 1
        elif direction == "right":
            direction = -1
        else:
            direction = 0

        if self.hcounter > number_of_counts-1:
            self.mc.stopMotors()
        else:
            self.mc.setMotor(2, direction*100)
            self.mc.setMotor(4, (-1)*direction*100)

    def cal_angle(self, segx,segy,dir):
	num_rows = 420
	num_col = 320
	dir_ang = 0
	x1 = 400
	y1 = 170

	est_loc = self.arena_map(segx, segy)
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


    def arena_map(self, x,y):
	seg_x = 85
	seg_y = 64
	#arena  = [[0 for x in range(5)] for j in range(5)]
	loc_seg_x = math.floor((x-1)*85 + (85/2))
	loc_seg_y = math.floor((y-1)*64 + (64/2))
	pos_est = [loc_seg_x, loc_seg_y]
	return pos_est 
