from tkinter import *
import time
import numpy as np
import math
from matplotlib.patches import Ellipse


class World:
    # size of the environment
    min_X = 0
    max_X = 800
    min_Y = 0
    max_Y = 800

    steps = 0
    radius = 10

    room = [[100, 100, 700, 100, 700, 700, 100, 700], [200, 200], [300, 500],
                        [450, 600], [600, 150], [550, 600], [550, 300], [200, 530],
                        [470, 470], [280, 210], [350, 130], [630, 240]]

    def __init__(self, window):
        self.window = window
        self.canvas = Canvas(self.window)
        self.draw_obstacles()
        self.canvas.configure(width=self.max_X, height=self.max_Y)
        self.canvas.pack(fill="both", expand=True)

    def init_agent(self, agent):
        self.canvas.pack()
        self.draw_robot(agent.get_position(), agent.get_radius())
        self.draw_landmark_lines(agent.get_position())

    def get_obstacles(self):
        return self.room[1:]

    def get_coordinate_obstacle(self, index):
        return self.room[index + 1]

    def draw_obstacles(self):
        for i, obstacle in enumerate(self.room):
            if i == 0:
                self.canvas.create_polygon(obstacle, outline='black', fill='white', tags=f"polygon{i}")
                self.canvas.tag_lower(f"polygon{i}")
            else:
                self.canvas.create_oval(obstacle[0] - self.radius, obstacle[1] - self.radius,
                                        obstacle[0] + self.radius, obstacle[1] + self.radius,
                                        fill='black', tags=f"oval{i}")
                self.canvas.tag_raise(f"oval{i}")

    def draw_robot(self, position, radius):
        # print(position, radius)
        x0 = position[0] - radius
        y0 = position[1] - radius
        x1 = position[0] + radius
        y1 = position[1] + radius
        x2 = position[0] + radius * np.cos(position[2])
        y2 = position[1] + radius * np.sin(position[2])
        self.canvas.create_oval(x0, y0, x1, y1, tags='robot', fill="#C2DFFF"),
        self.canvas.create_line(position[0], position[1], x2, y2, tags='line')
        self.canvas.tag_raise('robot')
        self.canvas.tag_raise('line')

    def draw_landmark_lines(self, position):
        for i in range(len(self.room)-1):
            self.canvas.create_line(position[0], position[1], self.room[i + 1], self.room[i + 1], tags=f'landmark{i}', fill='green')

    def update_angle(self, direction_coord):
        # update line indicating direction from passed coordinates
        self.canvas.coords('line', direction_coord[0], direction_coord[1], direction_coord[2], direction_coord[3])
        self.canvas.lift('line')

    def move_agent(self, agent):
        # update all objects in environment
        position = agent.get_position()
        old_position = agent.get_old_position()
        x = position[0] - old_position[0]
        y = position[1] - old_position[1]
        self.canvas.move('robot', x, y)
        self.canvas.tag_raise('robot')
        self.update_angle(agent.get_direction_coords())
        self.update_landmarks_line(position, agent.get_landmarks())
        self.update_trajectory(position, old_position)
        self.update_state(agent.localization.state, agent.localization.old_state)
        if self.steps % 50 == 0:
            self.draw_covariance(agent.localization.state, agent.localization.covariance)
        self.canvas.update()
        time.sleep(0.01)
        self.steps += 1

    def update_trajectory(self, position, old_position):
        self.canvas.create_line(position[0], position[1], old_position[0], old_position[1])

    def update_landmarks_line(self, position, landmarks):
        for i in range(len(self.room) - 1):
            if landmarks[i] != -1:
                self.canvas.coords(f'landmark{i}', position[0], position[1], self.room[i+1][0], self.room[i+1][1])
                self.canvas.itemconfigure(f'landmark{i}', state='normal')
                self.canvas.tag_raise(f'landmark{i}')
            else:
                self.canvas.itemconfigure(f'landmark{i}', state='hidden')

    def update_state(self, state, old_state):
        self.canvas.create_line(state[0], state[1], old_state[0], old_state[1], fill='red')

    def draw_covariance(self, state, covariance):
        # print("update_covariance", state, [covariance[0][0], covariance[1][1]])
        # first draw the ellipse using matplotlib
        # calculate covariance height and width
        # a = covariance[0][0]
        # b = covariance[0][1]
        # c = covariance[1][1]
        # width = (a + c)/2 + math.sqrt(math.pow((a - c)/2, 2) + math.pow(b,2))
        # height = (a + c)/2 - math.sqrt(math.pow((a - c)/2, 2) + math.pow(b,2))
        # if a >= c:
        #     angle = 0
        # else:
        #     angle = 180

        # use covariance values
        width = covariance[0][0]
        height = covariance[1][1]
        ellipse = Ellipse((state[0], state[1]), width * 2 , height *2, covariance[2][2])
        vertices = ellipse.get_verts()  # get the vertices from the ellipse object

        flat_vertices = [item for sublist in vertices for item in sublist]
        # print(flat_vertices)
        # Turn it into a polygon
        self.canvas.create_polygon(*flat_vertices, fill='blue')

