#!/usr/bin/env python

import time


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
        self.odometer, self.protractor = self.hall_counter()
        print('{}\t{}'.format("Hall Counter: ",self.hcounter))
#        print('{}\t{}'.format(self.getSensors(), self.getInputs()))
#        print('{}\t{}'.format("IR sensor: ",self.getSensors()[6]))


#        self.poi_detect()
#        self.set_move(2)
#        self.set_turn(2)

        self.angle_hor, self.angle_ver = self.point_antenna(28,74,0)
#        vals_t = self.cal_angle(1,5,"east")
#        self.set_turn(vals_t[0])
        self.sc.engage()
        self.sc.setPosition(self.angle_ver)
#        self.cal_angle(200,300,"north")
        self.set_turn(self.angle_hor)

        time.sleep(0.05)

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
        displacement = self.hcounter*2.5+2.2
        turning_angle = self.hcounter*20.95 -3.5
        return displacement, turning_angle # return the extact distance in cm


####################################################
#                POI detection                     #
####################################################

    def poi_init(self):
        self.flag = 0
        self.mc.setMotor(1,100) # turn on the light bulb

    def poi_detect(self):
        # both of the front light sensors should be on POI first

	self.mc.setMotor(1,100) # turn on the light bulb
        if self.getSensors()[0] > 40 and self.getSensors()[1] > 40:
            self.flag = 1 # set a flag if so

        if self.getSensors()[0] <50 and self.getSensors()[1] < 50) and self.flag == 1:
            self.mc.stopMotors()

        if self.flag != 1:
            self.mc.setMotor(2,100)
            self.mc.setMotor(4,100)


#####################################################
#               Close loop Controls                 #
#####################################################


    def set_move(self, number_of_counts):
        if number_of_counts < 0:
            direction = -1 # backward
        elif number_of_counts > 0:
            direction = 1 # forward
        else:
            direction = 0

        number_of_counts = abs(number_of_counts) - (1 if self.hcounter < 5 else 2)
        if self.hcounter > number_of_counts:
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
            
        print('{}\t{}'.format("Orientation: ",self.protractor))


###################################################
#                Point Antenna                    #
###################################################


    def point_antenna(self, x, y, orientation):
        x_sat = 433
        y_sat = 249
        angle_hor = math.floor(math.degrees(math.atan2(y_sat-y, x_sat-x)))
        angle_ver = math.degrees(math.atan2(270, math.sqrt(((x_sat-x)**2 + (y_sat-y)**2))))
        angle_hor = angle_hor - orientation
        angle_hor = (angle_hor + 3.35)/20.95
        print('vertical angle: {}.'.format(angle_ver))
        print('horizontal angle: {}'.format(angle_hor))
        return angle_hor, angle_ver














