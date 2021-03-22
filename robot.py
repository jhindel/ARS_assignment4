"""
author: Diego Di Benedetto, Julia Hindel
"""

import numpy as np
from localization import Localization


class Robot:
    # robot features
    radius = 20.
    sensor_range = 200
    distance_threshold = 150
    init_position = np.array([150., 130., 0.])
    # movement features
    max_speed = 5
    velocity_constant = 2
    rotation_constant = 0.3

    def __init__(self, world):
        # parameters used for calculating rotated position
        self.w = 0
        # velocity
        self.velocity = 0
        # list of detected walls
        self.old_position = []
        self.landmarks = [-1 for _ in world.get_obstacles()]
        self.world = world
        # position
        self.position = self.init_position
        self.localization = Localization(self.position)
        # coordinates of line indicating direction
        self.direction_coords = np.array([self.position[0], self.position[1], self.position[0] + 20, self.position[1]])
        # boolean determining if checks for intersection are needed
        self.world.init_agent(self)

    def get_position(self):
        return self.position

    def get_old_position(self):
        return self.old_position

    def get_radius(self):
        return self.radius

    def get_direction_coords(self):
        return self.direction_coords

    def get_velocity(self):
        return self.velocity

    def get_landmarks(self):
        return self.landmarks

    def update(self):
        # calculate new position and update sensor values
        new_position = self.timestep_movement()
        self.update_landmarks(new_position)
        self.update_direction_coords(new_position)
        self.old_position = self.position.copy()
        self.position = new_position.copy()
        self.localization.apply_filter(self.velocity, self.w, self.landmarks, self.world.get_obstacles(), self.position)
        print('result ', self.localization.state, self.position)
        self.w = 0
        self.world.move_agent(self)

    def update_landmarks(self, new_position):
        # create boundary lines of objects
        landmarks = self.world.get_obstacles()
        for i in range(len(landmarks)):
                point = landmarks[i]
                # distance from new_position to line
                d = np.sqrt(pow(point[0] - new_position[0], 2) + pow(point[1] - new_position[1], 2))
                if d < self.distance_threshold:
                    self.landmarks[i] = d
                else:
                    self.landmarks[i] = -1

    """
    used for calculation of new position
    """

    def timestep_movement(self, time=1):
        # clip max_speed
        self.velocity = np.clip(self.velocity, -self.max_speed, self.max_speed)
        return self.calculate_position(time)

    def calculate_position(self, time):
        # according to formula
        delta = 1
        temp_position = self.position + np.array([[delta*time*np.cos(self.position[2]), 0],
                                      [delta*time*np.sin(self.position[2]), 0],
                                      [0, delta*time]]).dot(np.array([self.velocity, self.w]))
        temp_position = np.squeeze(temp_position)
        return temp_position

    """
    update line indicating direction
    """

    def update_direction_coords(self, new_position):
        x0 = new_position[0]
        y0 = new_position[1]
        x1 = x0 + self.radius * np.cos(new_position[2])
        y1 = y0 + self.radius * np.sin(new_position[2])
        self.direction_coords = [x0, y0, x1, y1]

    """
    reaction to user input 
    """

    def event_reaction(self, key):
        if key == 'w':  # positive increment of left wheel
            self.increment(1, True, False)
        elif key == 's':  # negative increment of left wheel
            self.increment(-1, True, False)
        elif key == 'd':  # positive increment of right wheel
            self.increment(1, False, True)
        elif key == 'a':  # negative increment of right wheel
            self.increment(-1, False, True)
        print('key ', key)

    def increment(self, positive, v, w):
        self.flag = False
        if v:
            self.velocity += self.velocity_constant * positive
        if w:
            self.w += self.rotation_constant * positive
        print('increment', 'v', self.velocity, 'w', self.w)
