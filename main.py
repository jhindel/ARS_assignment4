"""
author: Diego Di Benedetto, Julia Hindel
"""

from world import World
from robot import Robot
from tkinter import *


def key_pressed(event):
    print("key pressed")
    agent.event_reaction(event.char)

window = Tk()
window.title('World simulation')
window.bind('<Key>', key_pressed)

world = World(window)
agent = Robot(world)

while True:
    agent.update()

# window.mainloop()
