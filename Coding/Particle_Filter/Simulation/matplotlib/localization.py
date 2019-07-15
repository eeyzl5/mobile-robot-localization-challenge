import matplotlib.pyplot as plt
import matplotlib.path as mplPath
from matplotlib import collections  as mc
import numpy as np
from copy import copy

def grad_to_radians(ang):
    return ang / 180 * np.pi

class Point:
	def __init__(self,x,y):
		self.x = x
		self.y = y

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
    return -18.9717 * np.log(0.00151103 * (ir_val - 75.49))

def sensor_measurement_cm_sigma(value_cm):
    sigma = sigma_sensor(value_cm)
    error = (abs(cm_to_IR_val(value_cm) - cm_to_IR_val(value_cm + sigma)) + 
            abs(cm_to_IR_val(value_cm) - cm_to_IR_val(value_cm - sigma)))/2
    return error


def rouletteWheelSampling(particles, w):
    idx = range(len(particles))
    selected_idxs = np.random.choice(idx, size=len(particles), replace=True, p=w)
    
    # copy them
    new_particles = []
    for i in selected_idxs:
        p = copy(particles[i])
        new_particles.append(p)
    return new_particles

def gaussian(mu, sigma, x):        
    # Given a x position, mean mu and std sigma of Gaussian, calculates the probability
    # Note
    # mu: estimated distance by each particle's position, (map and landmarks are known)
    # x:  measured distance by the robot
    return np.exp(- ((mu - x) ** 2) / (sigma ** 2) / 2.0) / np.sqrt(2.0 * np.pi * (sigma ** 2))

class Robot:
    def __init__(self, x, y, angle):
        self.x = x
        self.y = y
        self.position = Point(x, y)

        self.move_noise_level = 0
        self.turn_noise_level = 0

        self.RAY_LENGTH = 100
        self.DELTA_ANGLE = np.pi / 4

        self.angle = (180 - angle + 360 - 90)%360
        self.update_angle()


        self.robot_width = 22.5
        self.robot_length = 25

        self.update_body()
        self.calc_rays()

    def get_position(self):
        return self.x, self.y

    def update_angle(self, ang=0):
        self.angle = (self.angle + ang +360) % 360
        self.rad_angle = self.angle / 180 * np.pi
        self.direction = np.array([np.cos(self.rad_angle), np.sin(self.rad_angle)])
        self.left_direction = [np.cos(self.rad_angle + self.DELTA_ANGLE), np.sin(self.rad_angle + self.DELTA_ANGLE)]
        self.right_direction = [np.cos(self.rad_angle - self.DELTA_ANGLE), np.sin(self.rad_angle - self.DELTA_ANGLE)]

    
    def calc_rays(self):
        ray_points = np.array([self.left_direction,
                               self.right_direction]) * self.RAY_LENGTH + np.array([self.front_right, self.front_left ])
        #ray_points[0] = [ray_points[0][0] + self.body[0][0], ray_points[0][1] + self.body[0][1]]
        #ray_points[1] = [ray_points[1][0] + self.body[1][0], ray_points[1][1] + self.body[1][1]]
        self.rays = [(np.array(self.body[0]), ray_points[1]), 
                    (np.array(self.body[1]), ray_points[0])]
                     #(np.array([self.x , self.y]) + np.array(self.body[1]), ray_points[1])]#, (position, ray_points[2])]
    
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
        left = cm_to_IR_val(left)
        right = cm_to_IR_val(right)
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

    

class Particle(Robot):
    def __init__(self, *args, **kwargs):
        super(Particle, self).__init__(*args, **kwargs)
        self.turn_noise_level = 2
        self.move_noise_level = 2

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
            [(147.5, 155), (185, 220)], 
            [(110,220),(147.5, 155)]]

        self.trianglePath = mplPath.Path(np.array([p[0] for p in self.triangle]))

        self.rectangle = [[(105, 75), (105, 91)],
             [(105, 91), (320, 91)],
             [(320, 91), (320, 75)],
             [(320, 75), (105, 75)]]

        self.rectanglePath = mplPath.Path(np.array([p[0] for p in self.rectangle]))

        self.cut_corners = [[(395, 0), (425, 30)], 
                            [(0, 250), (60, 320)]]

        self.corner1Path = mplPath.Path(np.array([(395, 0), (425, 30), (425, 0)]))
        self.corner2Path = mplPath.Path(np.array([(0, 250), (60, 320), (0, 320)]))

        self.segments = (self.borders + self.cut_corners 
                        + self.triangle + self.rectangle)

        self.robot = Robot(self.home[0], self.home[1], self.initial_direction_ang)

        self.n_particles = n_particles
        self.particles = None

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
        
        if self.trianglePath.contains_point(particle):
            return True
        
        if self.rectanglePath.contains_point(particle):
            return True
        
        if self.corner1Path.contains_point(particle):
            return True
        
        if self.corner2Path.contains_point(particle):
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

    def create_particles(self):
        N = self.n_particles
        particles = []
        coords = zip(np.random.uniform(self.home[0]-30, self.home[0]+30, N), 
                     np.random.uniform(self.home[1]-30, self.home[1]+30, N), 
                     np.random.randint(self.initial_direction_ang - 15,self.initial_direction_ang + 15, N))
        particles = [Particle(*p) for p in coords if not self.check_invalid_position(p)]                
        while len(particles) < N:
            M = N - len(particles)
            coords = zip(np.random.uniform(0, 425, M), np.random.uniform(0, 320, M), np.random.randint(0,360, M))
            new_particles = [Particle(*p) for p in coords if not self.check_invalid_position(p)]
            if len(new_particles) < 1:
                continue
            particles = particles + new_particles
        self.particles = particles
    

    def move_robot(self, l):
        self.robot.move(l)
        self.calc_ray_intersections(self.robot)

    def turn_robot(self, ang):
        self.robot.turn(ang)
        self.calc_ray_intersections(self.robot)

    def move_particles(self, l):
        if len(self.particles) > 0:

            particles = []
            for particle in self.particles:
                particle.move(l)
                if not self.check_invalid_position((particle.x, particle.y)):
                    self.calc_ray_intersections(particle)
                    particles.append(particle)
            self.particles = particles
            return True
        else:
            return False

    def turn_particles(self, ang):
        for particle in self.particles:
            particle.turn(ang)
            self.calc_ray_intersections(particle)

    def resample_particles(self):
        weights = []
        for p in self.particles:
            w = p.measurement_prob(self.robot.cm_measurements)
            weights.append(w)
            #pass # weight w is proportional to p(z|x)= p(z|x1)*p(z|x2)*.... 
        # quiz: normalize w (total probability), enter code below
        weights = np.array(weights)
        weights = weights / np.sum(weights)
        # quiz: apply resampling based on importance sampling below    
        self.particles = rouletteWheelSampling(self.particles, weights)
        if len(self.particles) < self.n_particles:
            while len(self.particles) < self.n_particles:
                M = self.n_particles - len(self.particles)
                coords = zip(np.random.uniform(0, 425, M), np.random.uniform(0, 320, M), np.random.randint(0,360, M))
                new_particles = [Particle(*p) for p in coords if not self.check_invalid_position(p)]
                if len(new_particles) < 1:
                    continue
                self.particles = self.particles + new_particles

    def estimate_robot(self):
        xx = 0; yy = 0; ang = 0
        for particle in self.particles:
            xx += particle.x
            yy += particle.y
            ang += particle.angle
        xx /= len(self.particles)
        yy /= len(self.particles)
        ang /= len(self.particles)
        ang = (ang + 720) % 360
        self.robot.x = xx
        self.robot.y = yy
        self.robot.ang = ang
        self.robot.update_angle()

    def plot_map(self, fig=None):
        #self.calc_rays()
        if fig is None:
            fig, ax = plt.subplots(figsize=(12,8))
        else:
            ax = plt.gca()
        if self.particles is not None:
            for particle in self.particles:
                ax.plot(particle.x, particle.y, 'yo', markersize=6)

        lc = mc.LineCollection(self.segments, colors='black', linewidths=2)
        rc = mc.LineCollection(self.robot.rays, colors='green', linewidths=2)

        ax.add_collection(lc)
        ax.add_collection(rc)
        ax.autoscale()
        ax.margins(0.1)

        ax.plot(self.home[0], self.home[1], 'bo', markersize=20)

        ax.plot(self.robot.x, self.robot.y, 'ro', markersize=5)
        #print(self.robot.body)
        #onewnonfwefwe +2
        robot_body = mc.LineCollection(self.robot.body_segments, colors='red', linewidths=2)
        ax.add_collection(robot_body)

        arr = plt.arrow(self.robot.x, self.robot.y, *(np.array(self.robot.direction) * self.robot.RAY_LENGTH/3), 
                        width= 1, head_width= 5, length_includes_head=True, color="red")
        ax.add_patch(arr)

        #if len(self.robot.ray_intersections) > 0:
        #    ax.plot(*zip(*self.robot.ray_intersections), 'go', markersize=6)

        plt.gca().invert_xaxis()
        return ax



    

