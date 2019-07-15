#!/usr/bin/env python

import time
import math
import numpy as np
import sys
import cv2
from collections import deque

from constants import *
import helpers
import localization

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
        self.reset_counter = 20


        self.status = "IDLE"
        self.cycle_timer = STANDARD_IDLE_CYCLES
        #self.next_status_q = deque([["MOVING", {"direction": "FORWARD", "hall_counts": 2}],
        #["IDLE", {"cycles": 70}],
        #["TURNING", {"direction": "LEFT", "hall_counts": 4}],
        #["MOVING", {"direction": "FORWARD", "hall_counts": 20}],
        #["MOVING", {"direction": "FORWARD", "hall_counts": 20}],
        #["MOVING", {"direction": "FORWARD", "hall_counts": 20}],
        #["MOVING", {"direction": "FORWARD", "hall_counts": 20}],
        #["MOVING", {"direction": "FORWARD", "hall_counts": 20}], ["WAITING_FOR_INPUT", {}]])
        self.next_status_q = deque([])

        self.waypoints = deque([(300, 35), (200, 35), (100, 35), (55,40), (50, 145), (74, 257)])
        self.next_waypoint = None
        self.POIs_found = []

        self.collision_detected = False
        self.hall_counts = 0

        self.poi_init()
        self.use_sonar = False

        self.initial_x = 387 #384
        self.initial_y = 30 #170
        self.initial_ang = 270

        self.map = localization.Map((self.initial_x, self.initial_y), angle=self.initial_ang, n_particles=300)
        self.map.create_particles()

        self.map.set_waypoints(self.waypoints)

        self.frame = self.map.plot_map2(True)

    def control(self):
        self.odometer, self.protractor = self.hall_counter()

        if self.getInputs()[1] == 1:
            self.stop_motors()
            print("STOPPING ROBOT")
            sys.exit(0)

        start_time = time.time()
        self.status_router()
        end_time = time.time()

        delta = (end_time - start_time)
        t = 0.05 - delta if delta < 0.05 else 0.05

        time.sleep(t)

    def vision(self):
#        image = self.camera.getFrame()
#        self.camera.imshow('Camera', image)
        cv2.imshow('Map', self.frame)
        cv2.waitKey(1) & 0xFF
        time.sleep(0.05)
        #return



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
        displacement = self.hcounter*2.5+2.2
        turning_angle = self.hcounter*20.95 -3.5
        return displacement, turning_angle # return the extact distance in cm

####################################################
#                POI detection                     #
####################################################

    def poi_init(self):
        self.poi_flag = False
        self.poi_detection_active = True
        self.mc.setMotor(1,100) # turn on the light bulb

    def poi_detect(self):
        # both of the front light sensors should be on POI first
        self.mc.setMotor(1,100) # turn on the light bulb
        if (len(self.POIs_found) > 0):
            for poi in self.POIs_found:
                if ( abs(self.map.robot.x - poi[0]) < 40 and abs(self.map.robot.y - poi[1]) < 40):
                    return
        if not self.poi_detection_active:
            return

        if (self.getSensors()[0] > 40 and self.getSensors()[1] > 40):
            self.poi_flag = True # set a flag if so

        if ( # self.getSensors()[2] > 15 or
            (self.getSensors()[0] < 50 and self.getSensors()[1] < 50) and self.poi_flag):
            self.poi_detection_active = False
            print("="*20)
            print("="*20)
            print("POI FOUND")
            print("="*20)
            print("="*20)
            self.mc.stopMotors()
            self.update_status("EVALUATING")
            self.collision_detected = True
            self.stop_motors()
            self.map.move_particles(self.odometer)
            sensors = self.getSensors()
            self.map.robot.set_sensor_measurements(sensors[7], sensors[6])
            self.map.resample_particles()
            self.map.estimate_robot()
            self.frame = self.map.plot_map2(True)
            self.POIs_found.append((self.map.robot.x, self.map.robot.y))
            self.poi_detection_active = True
            self.update_status("IDLE")#, next="POI_DETECTED")




#=========================================================#
# Giovanni's Control Section
#=========================================================#

    ##################################
    ## Status management section


    def update_status(self, new_status, **kwargs):
        """
        new_status: <String> new status to set
        kwargs: "next" is the status that should be set after the current one
                when the timer expires. This option adds the action to the top of the queue.
                "params" are the parameters to pass with the "next" instruction.

        Update the status of the robot and queues a subsequent instruction to execute 
        after.
        """
        self.status = new_status
        if "next" in kwargs:
            params = kwargs.get("params") if "params" in kwargs else {}
            self.next_status_q.appendleft([kwargs.get("next"), params])
    
    def empty_next_q(self):
        self.next_status_q = deque([])

    def status_router(self):
        """
        Based on the internal status of the robot call the correct methods.
        If the cycle timer has been set to a value higher than 0 this method 
        proceeds to set the next instruction
        """

        # Manage cycle_timer
        if (self.cycle_timer > 0):
            #print("Current cycle timer: {}".format(self.cycle_timer))
            #print("\n")
            self.cycle_timer -= 1
            if (self.cycle_timer <= 0):
                self.cycle_timer = 0
                self.stop_motors()
                self.update_status("IDLE")

        if (self.status == "IDLE" and self.cycle_timer <= 0):
            self.handle_next_status()

        if (self.status == "MOVING"):
            #print("Hall Counts: {} ------ HCOUNTER: {}".format(self.hall_counts, self.hcounter))
            self.mc.setMotor(2, self.right_side_speed)
            self.mc.setMotor(4, self.left_side_speed)


            #if(self.poi_detect())
            if ((self.check_IR_collision() or self.check_sonar_collision()) and not self.collision_detected):
                self.stop_motors()
                print("=*#\n"*5)
                print("MOVING COLLISION STOP")
                print("=*#\n"*5)
                sensors = self.getSensors()
                print("*"*10)
                print("IR sensors: L {} -- R {}".format(sensors[7], sensors[6]))
                print("Sonar: {}".format(sensors[4]))
                print("*"*10)

                
                self.update_status("EVALUATING")
                self.collision_detected = True
                self.stop_motors()
                self.map.move_particles(self.odometer)
                sensors = self.getSensors()
                self.map.robot.set_sensor_measurements(sensors[7], sensors[6])
                self.map.resample_particles()
                self.map.estimate_robot()
                self.frame = self.map.plot_map2(True)
                self.update_status("IDLE", next="AVOID_COLLISION2")
                return
            
            #self.hall_counts = abs(self.hall_counts) - (1 if self.hcounter < 5 else 2)
            if self.hall_counts > 0 and self.hcounter > self.hall_counts:
                self.update_status("EVALUATING")
                self.stop_motors()
                self.map.move_particles(self.odometer)
                sensors = self.getSensors()
                self.map.robot.set_sensor_measurements(sensors[7], sensors[6])
                self.map.resample_particles()
                self.map.estimate_robot()
                self.frame = self.map.plot_map2(True)
                self.update_status("IDLE", )
            
            self.poi_detect()
            
        if (self.status == "TURNING"):
            self.mc.setMotor(2, self.right_side_speed)
            self.mc.setMotor(4, self.left_side_speed)
            #if (self.check_IR_collision() and not self.collision_detected):
            #    self.collision_detected = True
            #    self.stop_motors()
            #    self.update_status("IDLE", next="AVOID_COLLISION")

            #print("Hall Counts: {}".format(self.hall_counts))
            #print("Counter: {}".format(self.hcounter))
            if (self.hall_counts > 0 and self.hcounter > abs(self.hall_counts)-1):
                self.update_status("EVALUATING")
                self.mc.stopMotors()
                self.map.turn_particles(self.protractor)
                sensors = self.getSensors()
                self.map.robot.set_sensor_measurements(sensors[7], sensors[6])
                self.map.resample_particles()
                self.map.estimate_robot()
                self.frame = self.map.plot_map2(True)
                self.update_status("IDLE")

            sonar = self.getSensors()[4]

            if self.use_sonar:
                if sonar > SONAR_THRESHOLD and self.check_IR_turn:
                    print("=*#\n"*5)
                    print("TURNING SONAR STOP")
                    print("=*#\n"*5)
                    self.update_status("EVALUATING")
                    self.use_sonar = False
                    self.mc.stopMotors()
                    ang = self.protractor if self.protractor > 5 else 5
                    self.map.turn_particles(self.protractor)
                    sensors = self.getSensors()
                    self.map.robot.set_sensor_measurements(sensors[7], sensors[6])
                    self.map.resample_particles()
                    self.map.estimate_robot()
                    self.frame = self.map.plot_map2(True)
                    self.update_status("IDLE")
                    #self.cycle_timer = TURNING_SONAR_CYCLES
        
        if (self.status == "WAITING_FOR_INPUT"):
            #print('{}\t{}'.format(self.getSensors(), self.getInputs()))
            self.mc.stopMotors()
            if self.getInputs()[0] == 1:
                self.update_status("IDLE", next="RESTORE_DIRECTION")

    def handle_next_status(self):
        # Get next instruction to execute
        if (len(self.next_status_q) > 0):
            next = self.next_status_q.popleft()
        else:
            next = ["IDLE", {}]
            self.stop_motors()
            self.next_status_q = deque([
               ["PLAN_PATH", {}],
               ["IDLE", {"cycles": 3*STANDARD_IDLE_CYCLES}],
                ])
            #self.update_status("IDLE")

        if (next[0] == "IDLE"):
            if (len(self.next_status_q) > 0):
                self.update_status("IDLE")
            else:
                self.cycle_timer = STANDARD_IDLE_CYCLES
                #self.update_status("IDLE", next="MOVING")
                self.update_status("IDLE")
            if ("cycles" in next[1]):
                self.cycle_timer = next[1].get("cycles")
            self.stop_motors()

        elif (next[0] == "MOVING"):
            self.initial_odometer = self.odometer
            direction = next[1].get("direction") if "direction" in next[1] else "FORWARD"
            
            if ("cycles" in next[1]):
                #self.cycle_timer = next[1].get("cycles")
                self.move(direction, cycles=next[1].get("cycles"))
            elif ("hall_counts" in next[1]):
                self.move(direction, hall_counts=next[1].get("hall_counts"))
            else:
                self.move(direction)
            self.update_status("MOVING")
        
        elif (next[0] == "TURNING"):
            
            direction = next[1].get("direction") if "direction" in next[1] else "LEFT"
            if ("cycles" in next[1]):
                self.turn(direction, cycles=next[1].get("cycles"))
            elif ("hall_counts" in next[1]):
                self.turn(direction, hall_counts=next[1].get("hall_counts"))
            elif ("use_sonar" in next[1]):
                self.use_sonar = next[1].get("use_sonar")
                self.cycle_timer = 0
                self.turn(direction)
            else:
                self.turn(direction)
        
        elif (next[0] == "RESET_POI"):
            self.poi_init()
            self.update_status("IDLE")

        elif (next[0] == "WAIT_FOR_RESTART"):
            self.cycle_timer = 0
            self.stop_motors()
            self.update_status("WAITING_FOR_INPUT")

        
        elif (next[0] == "AVOID_COLLISION"):
            self.stop_motors()
            self.cycle_timer = AVOID_COLLISION_CYCLES
            direction = "RIGHT" if self.getSensors()[7] > self.getSensors()[6] else "LEFT"
            self.next_status_q = deque([
                ["IDLE", {"cycles": STANDARD_IDLE_CYCLES}],
                ["MOVING", {"direction": "BACKWARD", "cycles": BACKWARD_CYCLES_COLLISION}],
                ["IDLE", {"cycles": STANDARD_IDLE_CYCLES}],
                ["RESET_COLLISION_DETECTION", {}],
                ["TURNING", {"direction": direction, "use_sonar": True}],
                ["IDLE", {"cycles": STANDARD_IDLE_CYCLES}]
                ])
            self.update_status("AVOID_COLLISION")

        elif (next[0] == "AVOID_COLLISION2"):
            self.stop_motors()
            self.cycle_timer = AVOID_COLLISION_CYCLES
            direction = "RIGHT" if self.getSensors()[7] > self.getSensors()[6] else "LEFT"
            self.next_status_q = deque([
                ["IDLE", {"cycles": STANDARD_IDLE_CYCLES}],
                ["TURNING", {"direction": direction, "hall_counts": 2}],
                ["IDLE", {"cycles": STANDARD_IDLE_CYCLES}],
                ["RESET_COLLISION_DETECTION", {}],
                ])
            self.update_status("AVOID_COLLISION")

        elif (next[0] == "POI_DETECTED"):
            self.stop_motors()
            self.cycle_timer = POI_DETECTED_CYCLES
            vals_t = self.cal_angle(self.test_x, self.test_y, self.test_dir)
            self.poi_angle = {"angle": vals_t[0], "direction": "RIGHT"}
            #print("Angle: {}".format(vals_t))
	    #self.set_turn(vals_t[0])
            self.sc.engage()
            self.sc.setPosition(vals_t[1])
            self.next_status_q = deque([
                ["IDLE", {"cycles": STANDARD_IDLE_CYCLES}],
                ["TURNING", {"direction": "LEFT", "hall_counts": vals_t[0]}],
                ["WAIT_FOR_RESTART", {}]
                ])
            self.update_status("POI_DETECTED")
        
        elif (next[0] == "RESTORE_DIRECTION"):
            self.next_status_q = deque([
                ["IDLE", {"cycles": STANDARD_IDLE_CYCLES}],
                ["TURNING", {"direction": self.poi_angle["direction"], "hall_counts": self.poi_angle["angle"]}],
                ["IDLE", {"cycles": STANDARD_IDLE_CYCLES}],
                ["MOVING", {"direction": "FORWARD", "cycles": POI_RESET_FORWARD}],
                ["RESET_POI", {}],
                ])

        elif (next[0] == "POINT_ANTENNA"):
            self.cycle_timer = POINT_ANTENNA_CYCLES
            self.stop_motors()
            self.set_antenna_angle(next[1].get("angle"))

        elif (next[0] == "RESET_COLLISION_DETECTION"):
            print("Resetting collision")
            self.cycle_timer = COLLISION_RESET_CYCLES
            self.collision_detected = False
            self.update_status("RESET_COLLISION_DETECTION")

        elif (next[0] == "PLAN_PATH"):
            self.update_status("PLANNING_PATH")
            if self.next_waypoint == None:
                if len(self.waypoints) < 1:
                    sys.exit(0)
                self.next_waypoint = self.waypoints.popleft()
            if (abs(self.map.robot.x - self.next_waypoint[0]) < 20 and abs(self.map.robot.y - self.next_waypoint[1]) < 20  ):
                if len(self.waypoints) < 1:
                    sys.exit(0)
                self.next_waypoint = self.waypoints.popleft()
            #move, turn = self.map.plan_trajectory(*self.next_waypoint)
            move, turn = self.map.plan_trajectory2(*self.next_waypoint)
            if turn["angle"] >= 20:
                self.next_status_q.append(["TURNING", {"direction": turn["direction"], "hall_counts": turn["angle"]/20}])
                #self.next_status_q.append(["UPDATE_POSITION", {"operation": "TURN", "par": turn["angle_ws"]}])
            self.next_status_q.append(["MOVING", {"direction": "FORWARD", "hall_counts": move/2.5}])
            #self.next_status_q.append(["UPDATE_POSITION", {"operation": "MOVE", "par": move}])
            self.next_status_q.append(["IDLE", {"cycles": STANDARD_IDLE_CYCLES}])
            self.update_status("IDLE")


        elif (next[0] == "UPDATE_POSITION"):
            #self.update_status("UPDATING_POSITION")
            oper = next[1].get("operation")
            #sensors = self.getSensors()
            #self.map.robot.set_sensor_measurements(sensors[7], sensors[6])
            #if oper == "TURN":
            #    self.map.turn_particles(next[1].get("par"))
            #else:
            #    self.map.move_particles(next[1].get("par"))
            #self.map.resample_particles()
            #self.update_status("IDLE")
            
            

        else:
            self.stop_motors()
            self.update_status("IDLE")
            print("ERROR in next status: {}".format(next[0]))
        
        #print("Status: {}".format(self.status))
        #print(self.next_status_q)

    ## END Status management section
    ##################################

    ##################################
    ## Movement and maneuvers methods

    def check_IR_collision(self):
        """
        Returns True if the ir sensors detect a value above the constant
        IR_COLLISION_THRESHOLD, False otherwise.
        """
        sensors = self.getSensors()
        return (sensors[6] >= IR_COLLISION_THRESHOLD or 
                sensors[7] >= IR_COLLISION_THRESHOLD)

    def check_sonar_collision(self):
        sensors = self.getSensors()
        return sensors[4] <= SONAR_THRESHOLD

    def check_IR_turn(self):
        """
        Returns True if the ir sensors detect a value above the constant
        IR_COLLISION_THRESHOLD, False otherwise.
        """
        sensors = self.getSensors()
        return (sensors[6] >= IR_TURNING_THRESHOLD or 
                sensors[7] >= IR_TURNING_THRESHOLD)

    def move(self, direction, **kwargs):
        """
        direction: <String> if "BACKWARD" the robot moves backward
        kwargs: "cycles" to set for how many iterations of the control cycle to move forward.

        Starts the motors with the correct values. 
        Default direction is forward, if the parameter backward 
        is given the robot moves back.
        Takes into account the constant TEST_MODE
        """
        drc = direction if direction == "BACKWARD" else "FORWARD"
        if (TEST_MODE == True):
            print("TEST")
            return 1
        speed = -100 if direction == "BACKWARD" else 100
        self.right_side_speed = speed
        self.left_side_speed = speed
        self.mc.setMotor(2, speed)
        self.mc.setMotor(4, speed)
        if "cycles" in kwargs:
            self.cycle_timer = kwargs.get("cycles")
        elif "hall_counts" in kwargs:
            self.hall_init()
            self.cycle_timer = 0
            self.hall_counts = kwargs.get("hall_counts")

        else:
            self.cycle_timer = 0
        #self.update_status("MOVING", next="IDLE", params={"cycles": STANDARD_IDLE_CYCLES})
        self.update_status("MOVING")
        return 0
        
    def set_antenna_angle(self, angle):
        """
        angle: <int> angle at which to point the antenna
        """
        if (type(angle) is not int):
            print("Error: Antenna angle has to be an integer")
            angle = 0
        self.sc.engage()
        self.sc.setPosition(int(angle))
        self.update_status("POINT_ANTENNA", next="IDLE", params={"cycles": STANDARD_IDLE_CYCLES})
        print("Setting antenna to " + str(angle))
        #self.sc.disengage()
        return 0
        
    def turn(self, direction, **kwargs):
        """
        direction: <String> the direction in which to turn.
        kwargs: "cycles" to set for how many iterations of the control cycle

        Sets the motors to turn the robot. Default direction is left, the exact value of 
        "RIGHT" has to be specified to turn in the other direction. I recommend to explicitly write
        the "LEFT" direction too for readability of code.
        """

        if (TEST_MODE == True):
            print("TEST")
            return 1
        self.mc.setMotor(2, -100 if direction == "RIGHT" else 100)
        self.mc.setMotor(4, 100 if direction == "RIGHT" else -100)
        self.right_side_speed = -100 if direction == "RIGHT" else 100
        self.left_side_speed = 100 if direction == "RIGHT" else -100
        self.hall_counts = 0
        if "cycles" in kwargs:
            self.cycle_timer = kwargs.get("cycles")
        elif "hall_counts" in kwargs:
            self.hall_init()
            self.cycle_timer = 0
            self.hall_counts = kwargs.get("hall_counts")

            print('{}\t{}'.format("Orientation: ", self.protractor))
        else:
            self.cycle_timer = 0
        #self.update_status("TURNING", next="IDLE", params={"cycles": STANDARD_IDLE_CYCLES})
        self.update_status("TURNING")
        return 0

    def stop_motors(self):
        if (TEST_MODE == True):
            return 1
        self.mc.stopMotors()
    
    ## END Movement and maneuvers methods
    ##################################


    #=============================================
    # KOMAL's code
    def cal_angle(self, segx,segy,dir):
        num_rows = 425
        num_col = 320
        dir_ang = 0
        x1 = 435
        y1 = 215

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
                    dir_ang = math.floor((angle_vert+260)/20)
        print('normalized angle {} {}'.format(dir_ang,angle_hor))
        estimated_angle = [dir_ang , angle_hor]
        return estimated_angle

    def arena_map(self, x,y):
        seg_x = 85
        seg_y = 64
        #arena  = [[0 for x in range(5)] for j in range(5)]
        loc_seg_x_cm = math.floor((x-1)*seg_x + (seg_x/2))
        loc_seg_y_cm = math.floor((y-1)*seg_y + (seg_y/2))
        pos_est = [loc_seg_x_cm, loc_seg_y_cm]
        return pos_est

    def arena_map_cm(self, x_cm, y_cm):
	seg_x = 85
        seg_y = 64
	loc_seg_x = round((x_cm/seg_x)+0.5)
	loc_seg_y = round((y_cm/seg_y)+0.5)
	pos_est = [loc_seg_x, loc_seg_y]
        return pos_est 

    def movement_tracker(self, hcount):
	move_made = hcount*3
	#self.arena_map_cm(42,96)
	init_loc = self.arena_map(self.start_x, self.start_y)
	#print('*****************init loc:  {}\t{}'.format(self.start_x, self.start_y))
	if self.start_x == 5 and self.start_y ==4:
		updated_loc_cm = init_loc[1] - move_made
		updated_loc_grid = self.arena_map_cm(init_loc[0], updated_loc_cm)
		print('***************** changed y loc in cm:  {}\t{}'.format(init_loc[0],updated_loc_cm))
		self.start_x = updated_loc_grid[0]
		self.start_y = updated_loc_grid[1]
		print('*****************new loc:  {}\t{}'.format(updated_loc_grid[0], updated_loc_grid[1]))
		#self.komal_control(self.start_x,self.start_x,"south")


	#self.start_x

    def mapped_movement(self):
	self.collision_detected=self.check_IR_collision()
        self.set_move(1)
        print('{}\t{}'.format("Hall Counter: ",self.hcounter))
	if self.collision_detected == True:
		self.movement_tracker(self.hcounter)
		#print('*****************new loc:  {}\t{}'.format(self.start_x, self.start_y))
		#time.sleep(0.05)
		#self.komal_control(self.start_x,self.start_y,"south")
		return True
	else:
		return False
    def arena_map(self, x,y):
        seg_x = 85
        seg_y = 64
        #arena  = [[0 for x in range(5)] for j in range(5)]
        loc_seg_x_cm = math.floor((x-1)*seg_x + (seg_x/2))
        loc_seg_y_cm = math.floor((y-1)*seg_y + (seg_y/2))
        pos_est = [loc_seg_x_cm, loc_seg_y_cm]
        return pos_est 

#==================================================
# END KOMAL's KODE
#==================================================


#==================================================
#==================================================
#==================================================
# STUFF TO BACKUP
#==================================================
#==================================================
#==================================================


#####################################################
#               Close loop Controls                 #
#####################################################

"""
    def set_move(self, number_of_counts):
        if number_of_counts < 0:
            direction = -1 # backward
        elif number_of_counts > 0:
            direction = 1 # forward
        else:
            direction = 0

       # number_of_counts = abs(number_of_counts) - (1 if self.hcounter < 5 else 2)
       # if self.hcounter > number_of_counts:
       #     self.mc.stopMotors()
	if self.collision_detected == True:
		self.mc.stopMotors()
        else:
            self.mc.setMotor(2,direction*100)
            self.mc.setMotor(4,direction*100)
            
        print('{}\t{}'.format("Odometer: ",self.odometer))

    def set_turn(self, number_of_counts):
        if number_of_counts < 0:
            direction = 1 # left
        elif number_of_counts > 0:
            direction = -1 # right
        else:
            direction = 0

        if self.hcounter > abs(number_of_counts)-1:
            self.mc.stopMotors()
        else:
            self.mc.setMotor(2, direction*100)
            self.mc.setMotor(4, (-1)*direction*100)
            
        print('{}\t{}'.format("Orientation: ", self.protractor))

    def poi_detect(self):
        # both of the front light sensors should be on POI first

        self.mc.setMotor(1,100) # turn on the light bulb
        if self.getSensors()[0] > 40 and self.getSensors()[1] > 40:
            self.flag = 1 # set a flag if so

        if ( # self.getSensors()[2] > 15 or
            (self.getSensors()[0] <50 and self.getSensors()[1] < 50) and self.flag == 1):
            self.mc.stopMotors()

        #    return True
        #return False

        if self.poi_flag != 1:
            self.mc.setMotor(2,100)
            self.mc.setMotor(4,100)

    def komal_control(self, x, y, dir):
        if self.reset_counter > 0:
	    self.reset_counter -= 1
            self.sc.engage()
	    self.sc.setPosition(0)
            return

        vals_t = self.cal_angle(x, y, dir)
        self.set_turn((-1)*vals_t[0])
        self.sc.engage()
        self.sc.setPosition(vals_t[1])
"""

