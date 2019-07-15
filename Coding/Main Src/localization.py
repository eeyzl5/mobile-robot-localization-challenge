#import matplotlib.pyplot as plt
#import matplotlib.path as mplPath
#from matplotlib import collections  as mc
import numpy as np
from copy import copy
import cv2

#==========================================================================================#
#                                         UTILITIES                                        #
#==========================================================================================#
                                                                                           #
def grad_to_radians(ang):
    return float(ang) / 180.0 * np.pi

def radians_to_grad(ang):
    return ang * 180.0 / np.pi

def ccw(A,B,C):
    return (C.y-A.y) * (B.x-A.x) > (B.y-A.y) * (C.x-A.x)

def intersect(A,B,C,D):
    return ccw(A,C,D) != ccw(B,C,D) and ccw(A,B,C) != ccw(A,B,D)

def intersect_check(seg1, seg2):
    A = Point(*seg1[0])
    B = Point(*seg1[1])
    C = Point(*seg2[0])
    D = Point(*seg2[1])
    return intersect(A,B,C,D)

def seg_length(seg):
    return np.sqrt(np.power(seg[0][0] - seg[1][0], 2) + np.power(seg[0][1] - seg[1][1], 2))

def find_intersection(x1,y1,x2,y2,x3,y3,x4,y4):
    px= ( (x1*y2-y1*x2)*(x3-x4)-(x1-x2)*(x3*y4-y3*x4) ) / ( (x1-x2)*(y3-y4)-(y1-y2)*(x3-x4) ) 
    py= ( (x1*y2-y1*x2)*(y3-y4)-(y1-y2)*(x3*y4-y3*x4) ) / ( (x1-x2)*(y3-y4)-(y1-y2)*(x3-x4) )
    return [px, py]

def sigma_sensor(x):
    return 6 + 25 * (1 / (1 + np.exp(-0.1 * x + (80*0.1))))

def cm_to_IR_val(value):
    return 6.618e+02 * np.exp(-5.271e-02 * value) + 7.549e+01

def IR_val_to_cm(ir_val):
    if (ir_val < 100):
        return 120
    return -18.9717 * np.log(0.00151103 * (ir_val - 75.49))

def sensor_measurement_cm_sigma(value_cm):
    sigma = sigma_sensor(value_cm)
    error = (abs(cm_to_IR_val(value_cm) - cm_to_IR_val(value_cm + sigma)) + 
            abs(cm_to_IR_val(value_cm) - cm_to_IR_val(value_cm - sigma)))/2
    return error

def rouletteWheelSampling(particles, w, N=0):
    if N == 0:
        N = len(particles)
    idx = range(len(particles))
    selected_idxs = np.random.choice(idx, size=N, replace=True, p=w)

    new_particles = []
    for i in selected_idxs:
        p = copy(particles[i])
        new_particles.append(p)
    return new_particles

def gaussian(mu, sigma, x):        
    return np.exp(- ((mu - x) ** 2) / (sigma ** 2) / 2.0) / np.sqrt(2.0 * np.pi * (sigma ** 2))


def ray_tracing_method(x,y,poly):
    """
    Check if the point is inside the polygon
    """
    n = len(poly)
    inside = False

    p1x,p1y = poly[0]
    for i in range(n+1):
        p2x,p2y = poly[i % n]
        if y > min(p1y,p2y):
            if y <= max(p1y,p2y):
                if x <= max(p1x,p2x):
                    if p1y != p2y:
                        xints = (y-p1y)*(p2x-p1x)/(p2y-p1y)+p1x
                    if p1x == p2x or x <= xints:
                        inside = not inside
        p1x,p1y = p2x,p2y

    return inside

#==========================================================================================#
#                                       POINT CLASS                                        #
#==========================================================================================#
                                                                                           #
class Point:
	def __init__(self,x,y):
		self.x = x
		self.y = y

                                                                                           #
#==========================================================================================#
#                                   END POINT CLASS                                        #
#==========================================================================================#


#==========================================================================================#
#                                       ROBOT CLASS                                        #
#==========================================================================================#
                                                                                           #
class Robot:
    def __init__(self, x, y, angle):
        self.x = x
        self.y = y
        self.position = Point(x, y)

        self.move_noise_level = 0
        self.turn_noise_level = 0

        self.RAY_LENGTH = 100
        self.DELTA_ANGLE = np.pi / 4

        self.rs_angle = angle # reference system angle: 0 at north, counterclockwise
        self.update_angle()

        self.robot_width = 22.5
        self.robot_length = 25

        self.update_body()
        self.calc_rays()

    def get_position(self):
        return self.x, self.y

    def update(self):
        self.update_angle()
        self.update_body()
        self.calc_rays()

    def convert_rs_angle_2_angle(self, rs_angle):
        return (-rs_angle + 90 + 360) % 360

    def convert_angle_2_rs_angle(self, angle):
        return (-angle + 90 + 360) % 360

    def update_angle(self, ang=0):
        self.rs_angle = (self.rs_angle + ang + 360) % 360
        self.angle = self.convert_rs_angle_2_angle(self.rs_angle)
        self.rad_angle = float(self.angle) / 180.0 * np.pi
        self.direction = np.array([np.cos(self.rad_angle), np.sin(self.rad_angle)])
        self.left_direction = [np.cos(self.rad_angle + self.DELTA_ANGLE), np.sin(self.rad_angle + self.DELTA_ANGLE)]
        self.right_direction = [np.cos(self.rad_angle - self.DELTA_ANGLE), np.sin(self.rad_angle - self.DELTA_ANGLE)]

    
    def calc_rays(self):
        ray_points = np.array([self.left_direction,
                               self.right_direction]) * self.RAY_LENGTH + np.array([self.front_right, self.front_left ])
        self.rays = [(np.array(self.body[0]), ray_points[1]), 
                    (np.array(self.body[1]), ray_points[0])]
    
    def move(self, l):
        mov = self.direction * l
        n = np.random.normal(0, self.move_noise_level, 2) if self.move_noise_level > 0 else (0, 0) 
        self.x += mov[0] + n[0]
        self.y += mov[1] + n[1]
        self.update_body()
        self.calc_rays()

    def turn(self, ang, noise_level=0):
        n = np.random.normal(0, self.turn_noise_level) if self.turn_noise_level  > 0 else 0
        self.update_angle(ang + n)
        self.update_body()
        self.calc_rays()

    def set_sensor_measurements(self, left=0, right=0):
        #left = cm_to_IR_val(left)
        #right = cm_to_IR_val(right)
        self.sensor_measurements = (left, right)
        self.cm_measurements = (IR_val_to_cm(left), IR_val_to_cm(right))
    
    def set_ray_intersections(self, itsc):
        self.ray_intersections = itsc

    def set_sensor_estimations(self, measurements):
        self.sensor_estimations = measurements
    
    def update_body(self):
        left_dir = np.array([np.cos(self.rad_angle - np.pi / 2), np.sin(self.rad_angle - np.pi / 2)])
        right_dir = np.array([np.cos(self.rad_angle + np.pi / 2), np.sin(self.rad_angle + np.pi / 2)]) 
        self.body = ((self.x, self.y) + left_dir*self.robot_width/2 + self.direction*self.robot_length/2,
                (self.x, self.y) + right_dir*self.robot_width/2 + self.direction*self.robot_length/2,
                (self.x, self.y) + left_dir*self.robot_width/2 - self.direction*self.robot_length/2,
                (self.x, self.y) + right_dir*self.robot_width/2 - self.direction*self.robot_length/2)
        self.body_segments = [(self.body[0], self.body[1]), (self.body[1], self.body[3]), 
                                (self.body[2], self.body[0]), (self.body[3], self.body[2])]
        self.front_left = self.body[0]
        self.front_right = self.body[1]

                                                                                           #
#==========================================================================================#
#                                   END ROBOT CLASS                                        #
#==========================================================================================#
#     
#==========================================================================================#
#                                    PARTICLE CLASS                                        #
#==========================================================================================#
                                                                                           #
class Particle(Robot):
    def __init__(self, *args, **kwargs):
        #super(Particle, self).__init__(*args, **kwargs)
        #super(Particle, self).__init__(**kwargs)
        Robot.__init__(self, *args, **kwargs)
        #super(Particle, self).__init__(*args, **kwargs)
        self.turn_noise_level = 10
        self.move_noise_level = 7

    def measurement_prob(self, measurements):        
        # calculates how likely a measurement should be        
        prob = 1.0
        for i in range(len(self.sensor_estimations)):
            sigma_meas = sensor_measurement_cm_sigma(self.sensor_estimations[i])
            sigma_robot = sensor_measurement_cm_sigma(measurements[i])
            sigma = np.sqrt(sigma_meas**2 + sigma_robot*2)
            prob *= gaussian(measurements[i], sigma, self.sensor_estimations[i])
        self.prob = prob
        return prob
                                                                                           #
#==========================================================================================#
#                                END PARTICLE CLASS                                        #
#==========================================================================================#


#==========================================================================================#
#                                         MAP CLASS                                        #
#==========================================================================================#
                                                                                           #

class Map:
    def __init__(self, home=(384, 170), angle=90, n_particles=100):
        self.home = home
        self.initial_direction_ang = angle

        self.borders = [[(425, 0), (0, 0)], 
           [(0, 0), (0, 320)], 
           [(0, 320), (320, 320)], 
           [(320, 320), (320, 215)],
           [(320, 215), (425, 215)],
           [(425, 215), (425, 0)] ]

        self.triangle = [[(185, 220), (110, 220)], 
            [(147, 155), (185, 220)], 
            [(110,220),(147, 155)]]

        self.triangle_poly = [p[0] for p in self.triangle]

        self.rectangle = [[(105, 75), (105, 91)],
             [(105, 91), (320, 91)],
             [(320, 91), (320, 75)],
             [(320, 75), (105, 75)]]

        self.rectangle_poly = [p[0] for p in self.rectangle]
        self.cut_corners = [[(395, 0), (425, 30)], 
                            [(0, 250), (60, 320)]]


        self.corner1_poly = [(395, 0), (425, 30), (425, 0)]
        self.corner2_poly = [(0, 250), (60, 320), (0, 320)]
        
        self.segments = (self.borders + self.cut_corners 
                        + self.triangle + self.rectangle)

        self.borders_safe = [[(415, 10), (10, 10)], 
           [(10, 10), (10, 310)], 
           [(10, 310), (310, 310)], 
           [(310, 310), (310, 205)],
           [(310, 205), (415, 205)],
           [(415, 205), (405, 10)] ]

        self.rectangle_safe = [[(95, 65), (95, 101)],
             [(95, 101), (310, 101)],
             [(310, 101), (310, 65)],
             [(310, 65), (95, 65)]]
        self.rectangle_poly_safe = [p[0] for p in self.rectangle_safe]

        self.triangle_safe = [[(195, 230), (100, 230)], 
            [(147, 145), (195, 230)], 
            [(100, 230),(147, 145)]]
        self.triangle_poly_safe = [p[0] for p in self.triangle_safe]

        self.cut_corners_safe = [[(385, 0), (425, 40)], 
                            [(0, 240), (70, 320)]]
        self.corner1_poly_safe = [(385, 0), (425, 40), (425, 0)]
        self.corner2_poly_safe = [(0, 240), (70, 320), (0, 320)]
        self.segments_safe = (self.borders_safe + self.cut_corners_safe 
                        + self.triangle_safe + self.rectangle_safe)
        self.robot = Robot(self.home[0], self.home[1], self.initial_direction_ang)

        self.n_particles = n_particles
        self.particles = None

        self.path = []
        self.nodes = []
        self.waypoints = []

    def calc_ray_intersections(self, robot):
        checked_rays = []
        ray_intersections = []
        sensor_distances = []
        for i, ray in enumerate(robot.rays):
            intersections = []
            closest_intersection = []
            for seg in self.segments:
                if intersect_check(ray, seg):
                    [[(x1,y1),(x2,y2)],[(x3,y3),(x4,y4)]] = (ray, seg)
                    intersections.append(find_intersection(x1, y1, x2, y2, x3, y3, x4, y4))
            if len(intersections) > 0:
                min_dist = 100000
                for itsc in intersections:
                    dist = seg_length([(robot.body[i][0], robot.body[i][1]), itsc])
                    if dist < min_dist:
                        min_dist = dist
                        closest_intersection = itsc
                checked_rays.append([(robot.body[i][0], robot.body[i][1]), closest_intersection])
                ray_intersections.append(closest_intersection)
                sensor_distances.append(min_dist)
            else:
                checked_rays.append(ray)
                sensor_distances.append(120)
        robot.rays = checked_rays
        robot.set_ray_intersections(ray_intersections)
        robot.set_sensor_estimations(sensor_distances)


    def check_collision(self, particle):
        
        if ray_tracing_method(particle[0], particle[1], self.triangle_poly):
            return True
        if ray_tracing_method(particle[0], particle[1], self.rectangle_poly):
            return True
        if ray_tracing_method(particle[0], particle[1], self.corner1_poly):
            return True
        if ray_tracing_method(particle[0], particle[1], self.corner2_poly):
            return True
        return False

    def check_out_of_range(self, particle):
        if particle[0] >= 320 and particle[1] >= 215:
            return True
        if particle[0] >= 425 or particle[1] >= 320:
            return True
        if particle[0] <= 0 or particle[1] <= 0:
            return True
        return False

    def check_invalid_position(self, particle):
        return self.check_out_of_range(particle) or self.check_collision(particle)    

    ##########################################################
    #
    # ROBOT METHODS

    def move_robot(self, l):
        self.robot.move(l)
        self.calc_ray_intersections(self.robot)

    def turn_robot(self, ang):
        self.robot.turn(ang)
        self.calc_ray_intersections(self.robot)

    # END ROBOT METHODS
    #
    ##########################################################

    ##########################################################
    #
    # PARTICLE FILTER METHODS

    def create_particles(self):
        N = self.n_particles
        particles = []
        coords = zip(np.random.uniform(self.home[0]-30, self.home[0]+30, N), 
                     np.random.uniform(self.home[1]-30, self.home[1]+30, N), 
                     #np.ones(N)*300)
                     np.random.randint(self.initial_direction_ang - 15, self.initial_direction_ang + 15, N))
        #coords = zip(np.random.uniform(0, 425, N), 
        #             np.random.uniform(0, 320, N), 
        #             np.random.randint(self.initial_direction_ang - 15,self.initial_direction_ang + 15, N))
        particles = [Particle(*p) for p in coords if not self.check_invalid_position(p)]                
        while len(particles) < N:
            M = N - len(particles)
            coords = zip(np.random.uniform(self.robot.x - 60, self.robot.x + 60, M), np.random.uniform(self.robot.y - 50, self.robot.y + 50, M), 
                        np.random.randint(self.robot.rs_angle - 70 , self.robot.rs_angle + 70, M))
            new_particles = [Particle(*p) for p in coords if not self.check_invalid_position(p)]
            if len(new_particles) < 1:
                continue
            particles = particles + new_particles
        self.particles = particles

    def move_particles(self, l):
        print("Moving Particles by: {}".format(l))
        if len(self.particles) > 0:
            particles = []
            for particle in self.particles:
                particle.move(l)
                if not self.check_invalid_position((particle.x, particle.y)):
                    self.calc_ray_intersections(particle)
                    particles.append(particle)
            self.particles = particles

            #self.estimate_robot()
            return True
        else:
            return False
    def turn_particles(self, ang):
        for particle in self.particles:
            particle.turn(ang)
            self.calc_ray_intersections(particle)
        #self.estimate_robot()

    def resample_particles(self):
        weights = []
        for p in self.particles:
            w = p.measurement_prob(self.robot.cm_measurements)
            weights.append(w)
        weights = np.array(weights)
        weights = weights / np.sum(weights)
        self.particles = rouletteWheelSampling(self.particles, weights, self.n_particles)
        if len(self.particles) < self.n_particles:
            while len(self.particles) < self.n_particles:
                M = self.n_particles - len(self.particles)
                coords = zip(np.random.uniform(self.robot.x - 60, self.robot.x + 60, M), 
                        np.random.uniform(self.robot.y - 50, self.robot.y+50, M), np.random.randint(self.robot.rs_angle - 70,self.robot.rs_angle + 70, M))
                new_particles = [Particle(*p) for p in coords if not self.check_invalid_position(p)]
                if len(new_particles) < 1:
                    continue
                self.particles = self.particles + new_particles

    def estimate_robot(self):
        xx = 0; yy = 0; direction = np.zeros(2)
        for particle in self.particles:
            xx += particle.x
            yy += particle.y
            direction += particle.direction
        xx /= len(self.particles)
        yy /= len(self.particles)
        direction /= len(self.particles)
        self.robot.x = xx
        self.robot.y = yy
        if direction[0] == 0:
            ang = 0 if direction[1] > 0 else 180
        else:
            rad_ang = np.arctan2(direction[1], direction[0]) 
            #ang1 = rad_ang * 180 / np.pi
            ang =( radians_to_grad(rad_ang) + 360 ) % 360
        self.robot.rs_angle = self.robot.convert_angle_2_rs_angle(ang)
        self.robot.update()
        self.calc_ray_intersections(self.robot)
    # END PARTICLE FILTER
    #
    ##########################################################

    ##########################################################
    #
    # TRAJECTORY PLANNING
    RRT_EXTEND_DIST = 60
    RRT_EXTEND_DIST2 = 60
    SMOOTHING_ITERATIONS = 200
    SMOOTHING_STEP = 0.1
    RADIUS_TARGET = 20.0

    def set_waypoints(self, wp):
        self.waypoints = wp

    def check_collision_safe(self, point):
        
        if ray_tracing_method(point[0], point[1], self.triangle_poly_safe):
            return True
        if ray_tracing_method(point[0], point[1], self.rectangle_poly_safe):
            return True
        if ray_tracing_method(point[0], point[1], self.corner1_poly_safe):
            return True
        if ray_tracing_method(point[0], point[1], self.corner2_poly_safe):
            return True
        return False

    def check_out_of_range_safe(self, point):
        if point[0] >= 310 and point[1] >= 205:
            return True
        if point[0] >= 415 or point[1] >= 310:
            return True
        if point[0] <= 10 or point[1] <= 10:
            return True
        return False

    def check_invalid_position_safe(self, particle):
        return self.check_out_of_range_safe(particle) or self.check_collision_safe(particle)  
        
    def sample_point(self, xmin=0, xmax=425, ymin=0, ymax=320):
        x = np.random.uniform(xmin, xmax)
        y = np.random.uniform(ymin, ymax)
        while self.check_invalid_position_safe((x, y)):
            x = np.random.uniform(xmin, xmax)
            y = np.random.uniform(ymin, ymax)
        return (x,y)

    def find_nearest_neighbour(self, p):
        index = 0
        dist_min = float('inf')
        for (i, n) in enumerate(self.nodes):
            dist = np.sqrt(np.power(p[0]-n[0][0], 2) + np.power(p[1]-n[0][1], 2))
            if dist < dist_min:
                dist_min = dist
                index = i
        return index

    def check_segment_collision_safe(self, p1, p2):
        for seg in self.segments_safe:
            if intersect_check((p1, p2), seg):
                return True
        return False

    def get_next_angle(self, p0, p1=np.array([0,0]), p2=None):
        ''' compute angle (in degrees) for p0p1p2 corner
        Inputs:
            p0,p1,p2 - points in the form of [x,y]
        '''
        if p2 is None:
            p2 = p1 + np.array([1, 0])
        v0 = np.array(p0) - np.array(p1)
        v1 = np.array(p2) - np.array(p1)

        angle = np.math.atan2(np.linalg.det([v0,v1]),np.dot(v0,v1))
        return np.degrees(angle)

    def plan_trajectory(self, x, y):
        self.nodes = [((self.robot.x, self.robot.y), None)]
        target = (x, y)
        is_reached = False
        xmin = self.robot.x if self.robot.x < x else x
        xmax = self.robot.x if self.robot.x > x else x
        ymin = self.robot.y if self.robot.y < y else y
        ymax = self.robot.y if self.robot.y > y else y
        while not is_reached:

            p_sample = self.sample_point(xmin-30, xmax+30, ymin-30, ymax+30)
            index = self.find_nearest_neighbour(p_sample)
            p_near = self.nodes[index][0]
            dist = np.sqrt(np.power(p_sample[0]-p_near[0], 2) + np.power(p_sample[1]-p_near[1], 2))
            if dist > self.RRT_EXTEND_DIST:
                p_normalized_x = p_near[0] + self.RRT_EXTEND_DIST*(p_sample[0]-p_near[0])/dist
                p_normalized_y = p_near[1] + self.RRT_EXTEND_DIST*(p_sample[1]-p_near[1])/dist
            else:
                p_normalized_x = p_sample[0]
                p_normalized_y = p_sample[1]
            p_new = (p_normalized_x, p_normalized_y)
            if self.check_segment_collision_safe(p_new, p_near):
                continue
            self.nodes += [(p_new, index)]
            if np.sqrt(np.power(p_new[0]-target[0], 2) + np.power(p_new[1]-target[1], 2)) < self.RADIUS_TARGET:
                is_reached = True
        self.path = []
        index = len(self.nodes)-1
        self.path += [(x, y)]
        while index != None:
            self.path += [self.nodes[index][0]]
            index = self.nodes[index][1]
        #self.path += [(self.robot.x, self.robot.y)]

        next_move = seg_length((self.path[-1], self.path[-2]))

        angle = self.get_next_angle(
            self.robot.direction + np.array([self.robot.x, self.robot.y]),
            (self.robot.x, self.robot.y),
            self.path[-2]
            )
        direction = "LEFT" if angle < 0 else "RIGHT"
        next_angle = {"angle": abs(angle), "direction": direction, "angle_ws": angle}
        
        
        
        for k in range(self.SMOOTHING_ITERATIONS):
            index1 = np.random.randint(0, len(self.path)-1)
            index2 = np.random.randint(0, len(self.path)-1)
            if index1 != index2 and not self.check_segment_collision_safe(self.path[index1], self.path[index2]):
                if index1 < index2:
                    index_low = index1
                    index_up = index2
                else:
                    index_low = index2
                    index_up = index1
                    
                middle = []
                deltax = (self.path[index_up][0] - self.path[index_low][0])
                deltay = (self.path[index_up][1] - self.path[index_low][1])
                for l in np.arange(self.SMOOTHING_STEP, 1.0-self.SMOOTHING_STEP, self.SMOOTHING_STEP):
                    middle += [(self.path[index_low][0]+l*deltax, self.path[index_low][1]+l*deltay)]
                path = self.path[:index_low+1] + middle + self.path[index_up:]
        return next_move, next_angle

    def plan_trajectory2(self, x, y):
        self.nodes = [((self.robot.x, self.robot.y), None)]
        target = (x, y)
        is_reached = False
        xmin = self.robot.x if self.robot.x < x else x
        xmax = self.robot.x if self.robot.x > x else x
        ymin = self.robot.y if self.robot.y < y else y
        ymax = self.robot.y if self.robot.y > y else y
        self.path = [(self.robot.x, self.robot.y), (x, y)]

        next_move = seg_length((self.path[-1], self.path[-2]))
        next_move = self.RRT_EXTEND_DIST2 if self.RRT_EXTEND_DIST2 < next_move else next_move

        angle = self.get_next_angle(
            self.robot.direction + np.array([self.robot.x, self.robot.y]),
            (self.robot.x, self.robot.y),
            self.path[-2]
            )
        direction = "LEFT" if angle < 0 else "RIGHT"
        next_angle = {"angle": abs(angle), "direction": direction, "angle_ws": angle}
        return next_move, next_angle


    # TRAJECTORY PLANNING
    # END
    ##########################################################
    

    def plot_map2(self, returning=False):
        
        x_displ = 50
        x_fact = 2
        y_displ = 50
        y_fact = 2
        frame = np.ones((320 * y_fact + y_displ * 2, 425 * x_fact + x_displ*2, 3), np.uint8) * 255
        # Draw borders
        for seg in self.segments:
            cv2.line(frame, (seg[0][0]*x_fact + x_displ, seg[0][1]*y_fact + y_displ),
                             (seg[1][0]*x_fact + x_displ, seg[1][1]*y_fact + y_displ), (0,0,0), 5)

        # Draw Particles
        if self.particles is not None:
            for particle in self.particles:
                #ax.plot(particle.x, particle.y, 'yo', markersize=6)
                cv2.circle(frame,(int(particle.x)*x_fact + x_displ, 
                                int(particle.y)*y_fact + y_displ), 2, (155,30,30), -1)
                # PARTICLE DIRECTIONS change to True to show
                if False:
                    cv2.line(frame, (int(particle.x)*x_fact + x_displ, int(particle.y)*y_fact + y_displ),
                            (int(particle.x + particle.direction[0]* particle.RAY_LENGTH)*x_fact + x_displ,
                             (int(particle.y + particle.direction[1]* particle.RAY_LENGTH)*y_fact + y_displ)), (230,55,0), 2)

        # Draw path
        if len(self.path) > 0:
            for i in range(len(self.path)):
                if i > 0:
                    #plt.plot([path[i-1][0], path[i][0]], [path[i-1][1], path[i][1]], color='r', marker='.')
                    cv2.line(frame, (int(self.path[i-1][0]*x_fact + x_displ), int(self.path[i-1][1]*y_fact + y_displ)),
                             (int(self.path[i][0]*x_fact + x_displ), int(self.path[i][1]*y_fact + y_displ)), (0,140,0), 5)

        cv2.circle(frame,(int(self.robot.x)*x_fact + x_displ, 
                          int(self.robot.y)*y_fact + y_displ), 6, (0, 0, 255), -1)
        for seg in self.robot.body_segments:
            cv2.line(frame, (int(seg[0][0])*x_fact + x_displ, int(seg[0][1])*y_fact + y_displ),
                            (int(seg[1][0])*x_fact + x_displ, int(seg[1][1])*y_fact + y_displ), (0,0,255), 3)
        cv2.line(frame, (int(self.robot.x)*x_fact + x_displ, int(self.robot.y)*y_fact + y_displ),
                            (int(self.robot.x + self.robot.direction[0]* self.robot.RAY_LENGTH/4)*x_fact + x_displ,
                             (int(self.robot.y + self.robot.direction[1]* self.robot.RAY_LENGTH/4)*y_fact + y_displ)), (0,0,255), 3)
        # self.robot.x, self.robot.y, *(np.array(self.robot.direction) * self.robot.RAY_LENGTH/3)
        if len(self.waypoints) > 0:
            for wp in self.waypoints:
                cv2.circle(frame,(wp[0]*x_fact + x_displ, wp[1]*y_fact + y_displ), 6, (50, 200, 100), -1)

        # Draw rays 
        for seg in self.robot.rays:
            cv2.line(frame, (int(seg[0][0])*x_fact + x_displ, int(seg[0][1])*y_fact + y_displ),
                            (int(seg[1][0])*x_fact + x_displ, int(seg[1][1])*y_fact + y_displ), (140,255, 90), 3)

        frame = cv2.flip( frame, -1 )
        self.frame = frame
        if returning:
            return frame
        cv2.imshow('map', frame)


                                                                                           #
#==========================================================================================#
#                                     END MAP CLASS                                        #
#==========================================================================================#
    

