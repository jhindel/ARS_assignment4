from tkinter import *
import time
import numpy as np
from PIL import ImageTk, Image
import matplotlib.pyplot as plt


class World:
    # size of the environment
    min_X = 0
    max_X = 800
    min_Y = 0
    max_Y = 800

    dust_steps = 0

    room = dict(points=[[100, 100, 700, 100, 700, 700, 100, 700], [200, 200, 210, 210], [300, 500, 310, 510],
                        [450, 600, 460, 610], [600, 150, 610, 160], [550, 600, 560, 610], [550, 300, 560, 310],
                        [200, 530, 210, 540], [470, 470, 480, 480], [280, 210, 290, 220]],
                walls=[[100, 100, 700, 100, 700, 700, 100, 700], [100, 500, 200, 500],
                     [500, 300, 700, 300], [400, 500, 400, 700]])

    def __init__(self, GUI, config=0):
        '''
        the configuration of the world is chosen with the parameter config(set in EA)
        GUI is a boolean variable, when set to true the world is displayed. the variable is dynamically changed
        in EA.run_generation
        '''
        self.world_configuration = config
        self.key = list(self.room.keys())[self.world_configuration]
        self.display = GUI
        if GUI:
            self.window = Tk()
            # self.window = Toplevel()
            self.window.title('World simulation')
            self.canvas = Canvas(self.window)
            self.draw_obstacles()
            self.canvas.configure(width=self.max_X, height=self.max_Y)
            self.canvas.pack(fill="both", expand=True)
            self.window.mainloop()
            # print(self.canvas)

    def change_display(self, bool):
        self.display = bool
        if self.display:
            self.__init__(self.display, self.world_configuration)

    def init_agent(self, agent):
        self.canvas.pack()
        self.draw_robot(agent.get_position(), agent.get_radius())
        self.draw_sensors(agent.get_position(), agent.get_sensors())
        self.draw_velocity(agent.get_position())

    def get_obstacles(self):
        return self.room

    def draw_obstacles(self):
        if self.key == 'points':
            for i, obstacle in enumerate(self.room[self.key]):
                if i == 0:
                    self.canvas.create_polygon(obstacle, outline='black', fill='white')
                else:
                    self.canvas.create_oval(obstacle, fill='black')
        else:
            for i, obstacle in enumerate(self.room[self.key]):
                self.canvas.create_polygon(obstacle, outline='black', fill='white')

    def draw_robot(self, position, radius):
        # print(position, radius)
        x0 = position[0] - radius
        y0 = position[1] - radius
        x1 = position[0] + radius
        y1 = position[1] + radius
        x2 = position[0] + radius * np.cos(position[2])
        y2 = position[1] + radius * np.sin(position[2])
        return [self.canvas.create_oval(x0, y0, x1, y1, tags='robot', fill="#C2DFFF"),
                self.canvas.create_line(position[0], position[1], x2, y2, tags='line')]

    def draw_sensors(self, position, sensors):
        for i, [x2, y2, value, _, _, _] in enumerate(sensors):
            # 16/20 at current position and 4/20 at end_point sensors
            self.canvas.create_text((16 / 20 * position[0] + 4 / 20 * x2), (16 / 20 * position[1] + 4 / 20 * y2),
                                    fill="black", font="Times 7",
                                    text=round(value), tags=f'sensor{i}')
            self.canvas.lift(f'sensor{i}')

    def draw_velocity(self, position):
        # draw the velocity values of the the motors
        self.canvas.create_text(position[0], position[1] + 12, fill="black", font="Times 10",
                                text="0", tags='velocity_l')
        self.canvas.create_text(position[0], position[1] - 12, fill="black", font="Times 10",
                                text="0", tags='velocity_r')

    def update_angle(self, direction_coord):
        # update line indicating direction from passed coordinates
        self.canvas.coords('line', direction_coord[0], direction_coord[1], direction_coord[2], direction_coord[3])
        self.canvas.lift('line')

    def update_sensors(self, position, sensors):
        for i, [x2, y2, value, _, _, _] in enumerate(sensors):
            # 16/20 at current position and 4/20 at end_point sensors
            self.canvas.itemconfig(f'sensor{i}', text=np.round(value))
            self.canvas.coords(f'sensor{i}', (16 / 20 * position[0] + 4 / 20 * x2),
                               (16 / 20 * position[1] + 4 / 20 * y2))
            # self.canvas.lift(f'sensor{i}')

    def update_velocity_display(self, position, velocity):
        # velocity left
        # add label 12 pixel from center
        x1 = position[0] + 12 * np.cos(position[2] - np.pi / 2)
        y1 = position[1] + 12 * np.sin(position[2] - np.pi / 2)
        self.canvas.itemconfig('velocity_l', text=np.round(velocity[1], 0))
        self.canvas.coords('velocity_l', x1, y1)
        # self.canvas.lift('velocity_l')
        # velocity right
        # add label 12 pixel from center
        x1 = position[0] + 12 * np.cos(position[2] + np.pi / 2)
        y1 = position[1] + 12 * np.sin(position[2] + np.pi / 2)
        self.canvas.itemconfig('velocity_r', text=np.round(velocity[0], 0))
        self.canvas.coords('velocity_r', x1, y1)
        # self.canvas.lift('velocity_r')

    def move_agent(self, x, y, agent):
        # update all objects in environment
        self.canvas.move('robot', x, y)
        self.update_angle(agent.get_direction_coords())
        self.update_sensors(agent.get_position(), agent.get_sensors())
        self.update_velocity_display(agent.get_position(), agent.get_velocity())
        self.canvas.update_idletasks()
        self.canvas.after(1)
