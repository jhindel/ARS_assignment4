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
    radius = 10

    room = [[100, 100, 700, 100, 700, 700, 100, 700], [200, 200], [300, 500],
                        [450, 600], [600, 150], [550, 600], [550, 300], [200, 530],
                        [470, 470], [280, 210]]

    def __init__(self, GUI, window, config=0):
        '''
        the configuration of the world is chosen with the parameter config(set in EA)
        GUI is a boolean variable, when set to true the world is displayed. the variable is dynamically changed
        in EA.run_generation
        '''
        self.display = GUI
        if GUI:
            self.window = window
            self.canvas = Canvas(self.window)
            self.draw_obstacles()
            self.canvas.configure(width=self.max_X, height=self.max_Y)
            self.canvas.pack(fill="both", expand=True)
            # self.window.mainloop()
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
                                text="0", tags='velocity')
        self.canvas.tag_raise('velocity')

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
        # add label 12 pixel from center
        x1 = position[0] + 12 * np.cos(position[2] - np.pi / 2)
        y1 = position[1] + 12 * np.sin(position[2] - np.pi / 2)
        self.canvas.itemconfig('velocity', text=np.round(velocity, 0))
        self.canvas.coords('velocity', x1, y1)

    def move_agent(self, x, y, agent):
        # update all objects in environment
        self.canvas.move('robot', x, y)
        self.canvas.tag_raise('robot')
        self.update_angle(agent.get_direction_coords())
        self.update_sensors(agent.get_position(), agent.get_sensors())
        self.update_velocity_display(agent.get_position(), agent.get_velocity())
        # self.canvas.update_idletasks()
        # self.canvas.after(1)
        self.canvas.update()
        # necessary otherwise too fast
        time.sleep(0.01)

    def update_trajectory(self, position):
        return
