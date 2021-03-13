import numpy as np


class Localization:
    covariance_init = 0.1
    R_init = 0.1
    Q_init = 0.1

    def __init__(self, position):
        self.state = position
        self.old_state = []
        self.covariance = np.array([[self.covariance_init, 0, 0], [0, self.covariance_init, 0],
                                    [0, 0, self.covariance_init]])
        self.R = np.array([[self.R_init, 0, 0], [0, self.R_init, 0], [0, 0, self.R_init]])
        self.Q = np.array([[self.Q_init, 0, 0], [0, self.Q_init, 0], [0, 0, self.Q_init]])
        self.I = np.array([[1, 0, 0], [0, 1, 0], [0, 0, 1]])

    def apply_filter(self, velocity, w, landmarks, obstacles):
        z = self.sensor_model(landmarks, obstacles)
        delta = 1
        time = 1
        B = np.array([[delta * time * np.cos(self.state[2]), 0],
                      [delta * time * np.sin(self.state[2]), 0],
                      [0, delta * time]])
        self.old_state = self.state.copy()
        temp_state = self.I.dot(self.state) + B.dot(np.array([velocity, w]))
        temp_covariance = self.I.dot(self.covariance).dot(self.I.T) + self.R
        K = temp_covariance.dot(self.I.T) * 1 / (self.I.dot(temp_covariance).dot(C.T) + self.Q)
        self.state = temp_state + K * (z - self.I.dot(temp_state))
        self.covariance = (self.I - K.dot(self.I)) * temp_covariance
        return self.state, self.covariance

    def sensor_model(self, landmarks, obstacles):
        z = 0
        for i in range(len(landmarks)):
            vector = np.array([np.sqrt(pow(obstacles[0] - self.state[0], 2) + pow(obstacles[1] - self.state[1], 2)),
                               np.atan2(obstacles[1]-self.state[1], obstacles[0]-self.state[0]) - self.state[2]])
        return z
