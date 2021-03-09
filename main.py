"""
author: Diego Di Benedetto, Julia Hindel
"""

from environment import Environment
from robot import Robot
from tkinter import *


def key_pressed(event):
    agent.event_reaction(event.char)

window = Tk()
window.title('World simulation')
window.bind('<Key>', key_pressed)

agent = Robot()
world = Environment(window, agent.get_position(), agent.get_radius(), agent.get_sensors())
while True:
    x, y = agent.update(world.get_obstacles())
    world.move_agent(x, y, agent)


window.mainloop()
