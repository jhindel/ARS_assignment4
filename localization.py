import math
from collections import Counter
import numpy as np


class Localization:

    def __init__(self, position):
        self.state = position
        self.old_state = []
        self.covariance = np.array([[1, 0, 0], [0, 1, 0], [0, 0, 1]])
        self.R = np.array([[0.1, 0, 0], [0, 0.1, 0], [0, 0, 0.1]])
        self.Q = np.array([[0.1, 0, 0], [0, 0.1, 0], [0, 0, 0.1]])
        self.I = np.array([[1, 0, 0], [0, 1, 0], [0, 0, 1]])

    def apply_filter(self, velocity, w, landmarks, obstacles, position):
        z = self.sensor_model(landmarks, obstacles, position)
        delta = 1
        time = 1
        B = np.array([[delta * time * np.cos(self.state[2]), 0],
                      [delta * time * np.sin(self.state[2]), 0],
                      [0, delta * time]])
        self.old_state = self.state.copy()
        # no prediction
        if velocity == 0 and w == 0:
            temp_state = self.state
            temp_covariance = self.covariance
        # prediction
        else:
            temp_state = self.I.dot(self.state) + B.dot(np.array([velocity, w]))
            temp_covariance = self.I.dot(self.covariance).dot(self.I.T) + self.R

        # no correction
        if np.all(z) == 0:
            self.state = temp_state
            self.covariance = temp_covariance
        # correction
        else:
            K = temp_covariance.dot(self.I.T).dot(np.linalg.inv((self.I.dot(temp_covariance).dot(self.I.T) + self.Q)))
            # print("K", K)
            self.state = temp_state + K.dot((z - self.I.dot(temp_state)))
            self.covariance = (self.I - K.dot(self.I)).dot(temp_covariance)
        # print("covariance-matrix\n", self.covariance)
        return self.state, self.covariance

    def sensor_model(self, landmarks, obstacles, position):
        vector = []
        for i in range(len(obstacles)):
            if landmarks[i] > -1:
                vector.append(np.array([np.sqrt(pow(obstacles[i][0] - position[0], 2) + pow(obstacles[i][1] - position[1], 2)),
                               math.atan2(obstacles[i][1] - position[1], obstacles[i][0] - position[0]) - position[2], i]))
        intersection_points = []
        for i in range(len(vector) - 1):
            [x1, y1], r1 = obstacles[int(vector[i][2])], vector[i][0]
            [x2, y2], r2 = obstacles[int(vector[i + 1][2])], vector[i + 1][0]
            dx, dy = x2 - x1, y2 - y1
            d = math.sqrt(dx * dx + dy * dy)
            if d > r1 + r2:
                continue
            a = (r1 * r1 - r2 * r2 + d * d) / (2 * d)
            h = math.sqrt(r1 * r1 - a * a)
            xm = x1 + a * dx / d
            ym = y1 + a * dy / d
            xs1 = int(xm + h * dy / d)
            xs2 = int(xm - h * dy / d)
            ys1 = int(ym - h * dx / d)
            ys2 = int(ym + h * dx / d)
            intersection_points.append((xs1, ys1))
            intersection_points.append((xs2, ys2))

        error = np.array([np.random.normal(), np.random.normal(), np.random.normal()])
        if intersection_points:
            if len(intersection_points) > 2:
                c = Counter(intersection_points)
                intersection = c.most_common(1)[0][0]  # the one most common element... 2 would mean the 2 most common
                theta = 0
                for i in range(len(vector)):
                    x1, y1 = obstacles[int(vector[i][2])]
                    alpha = math.atan2(y1 - intersection[1], x1 - intersection[0])
                    theta += -vector[i][1] + alpha
                theta /= len(vector)
                return np.array([intersection[0], intersection[1], theta]) + error
        return np.zeros(3)
