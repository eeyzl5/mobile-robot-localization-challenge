from localization import Map
import matplotlib.pyplot as plt
import time
import numpy as np

my_map = Map(angle=270, n_particles=200)
my_map.create_particles()

fig, ax = plt.subplots(figsize=(12,8))
"""
for i in range(100):
    m = np.random.uniform(0,1)
    if m < 0.3:
        ang = np.random.randint(-10, 10)
        my_map.turn_robot(-ang)
        my_map.robot.set_sensor_measurements(*my_map.robot.sensor_estimations)
        my_map.turn_particles(-ang)
        my_map.resample_particles()
    else:
        l = np.random.choice(np.arange(-0.2,6))
        my_map.move_robot(l)
        my_map.robot.set_sensor_measurements(*my_map.robot.sensor_estimations)
        rem = my_map.move_particles(l)
        my_map.resample_particles()
    #print("Particles: {}".format(len(my_map.particles)))
    ax = my_map.plot_map(fig)
    plt.draw()
    plt.pause(0.1)
    plt.clf()
"""
ax = my_map.plot_map(fig)
plt.show()

