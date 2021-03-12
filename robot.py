"""
author: Diego Di Benedetto, Julia Hindel
"""

import math
import vector_calculation
import numpy as np
import random
from shapely.geometry import Point, Polygon


def chunks(lst, n):
    # Yield successive n-sized chunks from lst
    for i in range(0, len(lst), n):
        yield lst[i:i + n]

class Robot:
    # robot features
    radius = 20.
    n_sensors = 12
    sensor_range = 200
    distance_threshold = 100
    sensor_threshold_wall_collision = 10
    init_random = False
    init_position = np.array([150., 325., 0.])
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
        self.old_pos = []
        self.world = world
        # position
        self.position = self.init_position
        if self.init_random:
            self.init_robot_position()
        # coordinates of line indicating direction
        self.direction_coords = np.array([self.position[0], self.position[1], self.position[0] + 20, self.position[1]])
        # list of sensors
        self.sensors = self.set_sensors()
        # boolean determining if checks for intersection are needed
        self.intersection = False
        if self.world.display:
            self.world.init_agent(self)

    def get_position(self):
        return self.position

    def get_radius(self):
        return self.radius

    def get_sensors(self):
        return self.sensors

    def get_direction_coords(self):
        return self.direction_coords

    def get_velocity(self):
        return self.velocity

    def set_sensors(self):
        # create list of sensors including
        partition = 360 / self.n_sensors
        sensors = []
        for sensor_n in range(self.n_sensors):
            angle = partition * sensor_n
            x2 = self.position[0] + (self.sensor_range + self.radius) * np.cos(angle * np.pi / 180)
            y2 = self.position[1] + (self.sensor_range + self.radius) * np.sin(angle * np.pi / 180)
            # end_coordinates of sensor (x, y), distance to next wall, intersection point of next wall and sensor,
            # sensor_id
            sensors.append([x2, y2, self.sensor_range, -1, -1, sensor_n])
        return np.array(sensors)

    def init_robot_position(self):
        # initialize random position in current environment
        # successively checks whether the robot position is contained/intersected with obstacles
        obstacles = []
        found = False
        coords = self.world.room[self.world.key]
        for i in range(len(coords)):
            # print(len(coords[i]))
            coords_formatted = list(chunks(coords[i], 2))
            # print('formatted', coords_formatted)
            poly = Polygon(coords_formatted)
            obstacles.append(poly)
        while not found:
            found = True
            x_coord = random.randint(self.world.min_X + self.radius, self.world.max_X - self.radius)
            y_coord = random.randint(self.world.min_Y + self.radius, self.world.max_Y - self.radius)
            point = Point(x_coord, y_coord)
            circle = point.buffer(self.radius*4)
            # print(circle)
            if len(obstacles) == 1:
                if not obstacles[0].contains(circle):
                    found = False
                continue
            for i in range(len(obstacles)-1):
                # print(obstacles[i+1], obstacles[0].contains(circle), obstacles[i+1].contains(circle), obstacles[i+1].intersects(circle), x_coord, y_coord)
                if (not obstacles[0].contains(circle)) or (obstacles[i+1].contains(circle)) or (obstacles[i+1].intersects(circle)):
                    found = False
                    # print("setting to False")
        self.position = [x_coord, y_coord, 0.]

    """
    in every timestep apply certain transformations
    """

    def update(self):
        # calculate new position and update sensor values
        new_position = self.timestep_movement()
        temp_sensors = self.check_sensors(new_position)
        temp_sensors = self.update_boundaries(self.world.room[self.world.key], temp_sensors, new_position)
        self.update_direction_coords(new_position)
        old_position = self.position.copy()
        self.old_pos.append(old_position)
        self.position = new_position.copy()
        self.sensors = temp_sensors.copy()
        # make change in GUI application
        self.world.update_trajectory(self.get_position())
        if self.world.display:
            self.world.move_agent(self.position[0] - old_position[0], self.position[1] - old_position[1], self)

    """
    reaction to wall collision
    """

    def check_sensors(self, new_position):
        # update the position of the sensors according to the new angle and position
        temp_sensors = []
        angle = new_position[2]
        old_angle = self.position[2]
        for i in range(self.n_sensors):
            coordinates = self.sensors[i]
            old_sensor_angle = math.atan2(coordinates[1] - self.position[1], coordinates[0] - self.position[0])
            x2 = new_position[0] + ((self.sensor_range + self.radius) * np.cos(angle - old_angle + old_sensor_angle))
            y2 = new_position[1] + ((self.sensor_range + self.radius) * np.sin(angle - old_angle + old_sensor_angle))
            temp_sensors.append([x2, y2, self.sensor_range, -1, -1, i])
        return np.array(temp_sensors)

    def update_boundaries(self, obstacles, sensors, new_position):
        # create boundary lines of objects
        lines = []
        for object in obstacles:
            lines.append([object[0], object[1], object[2], object[1]])
            lines.append([object[2], object[1], object[2], object[3]])
            lines.append([object[0], object[3], object[2], object[3]])
            lines.append([object[0], object[3], object[0], object[1]])

        for i in range(len(sensors)):
            for line in lines:
                point = vector_calculation.line_intersect(*line, new_position[0], new_position[1], sensors[i][0],
                                       sensors[i][1])
                if point:
                    # distance from new_position to line
                    d = np.sqrt(pow(point[0] - new_position[0], 2) + pow(point[1] - new_position[1], 2)) - self.radius
                    if d < sensors[i][2]:
                        sensors[i][2] = d
                        # safe coordinates of line intersection if smaller than threshold
                        if d < self.distance_threshold:
                            sensors[i][3] = point[0]
                            sensors[i][4] = point[1]
                            self.intersection = True
        return sensors

    def apply_transformation(self, intermediate_position):
        # apply transformation (if touching wall in next step)
        if len(self.detected_walls) == 1:
            return self.calculate_transformed_position(self.detected_walls[0][0], self.detected_walls[0][1], intermediate_position)
        else:
            pos = self.calculate_transformed_position(self.detected_walls[0][0], self.detected_walls[0][1], intermediate_position)
            return self.calculate_transformed_position(self.detected_walls[1][0], self.detected_walls[1][1], pos)

    def calculate_transformed_position(self, sensor_point0, sensor_point1, intermediate_position):
        # calculate transformed position
        point_perpendicular_to_center = vector_calculation.get_point(sensor_point0, sensor_point1, self.position)
        vector_center_to_wall_perpendicular = vector_calculation.vector_between_points(self.position,
                                                                         point_perpendicular_to_center)
        point_perpendicular_to_new_center = vector_calculation.get_point(sensor_point0, sensor_point1, intermediate_position)
        new_center_transformed = vector_calculation.shift_point_with_vector(point_perpendicular_to_new_center,
                                                              np.negative(vector_center_to_wall_perpendicular))
        return np.append(new_center_transformed, intermediate_position[2])

    """
    used for calculation of new position
    """

    def timestep_movement(self, time=1):
        # clip max_speed
        self.velocity = np.clip(self.velocity, -self.max_speed, self.max_speed)
        temp_pos = self.calculate_position(time)
        self.w = 0
        return temp_pos

    def calculate_position(self, time):
        # according to formula
        temp_position = self.position.copy()
        delta = 1
        # if self.w == 0:
        #     temp_position[0] = np.round(
        #         self.position[0] + ((self.velocity[0] + self.velocity[1]) / 2) * np.cos(self.position[2]))
        #     temp_position[1] = np.round(
        #         self.position[1] + ((self.velocity[0] + self.velocity[1]) / 2) * np.sin(self.position[2]))
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
