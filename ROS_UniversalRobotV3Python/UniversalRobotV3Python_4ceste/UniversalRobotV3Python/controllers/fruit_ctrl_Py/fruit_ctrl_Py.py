"""fruit_ctrl_Py controller."""

from controller import Supervisor
from controller import Node
from controller import Field
import random

random.seed()

robot = Supervisor()

timestep = 64

i = 0
j = 8
k = 8
l = 8
fr = 1
max = 50


# Initialize camera
camera = robot.getDevice('camera')
camera.enable(timestep)
camera_bin = robot.getDevice('camera_bin')  # Add the new camera
camera_bin.enable(timestep)                 # Enable the new camera

fruit_initial_translation = [0.570002,2.85005,0.349962]

conveyor_belt = robot.getFromDef("conveyor_belt")

if conveyor_belt is None:
    print("Errore: Nodo 'conveyor_belt' non trovato nel file .wbt.")
else:
    # Ottieni il campo 'speed' del nastro
    speed_field = conveyor_belt.getField("speed")
    if speed_field is None:
        print("Errore: Campo 'speed' non trovato nel nodo 'conveyor_belt'.")
        
current_speed = 0.15

# Main loop:
while robot.step(timestep) != -1:

    if robot.getTime() > 7.5:
        if i == 0:
            if fr > 0:
                fr = int(random.choice([1,2]))
                if j == max: fr = 2
                if k == max: fr = 1
                if fr == 1 and j < max:
                    name = f'apple{j:d}'
                    j += 1
                if fr == 2 and k < max:
                    name = f'orange{k:d}'
                    k += 1
                fruit = robot.getFromDef(name)
                fruit_trans_field = Node.getField(fruit, 'translation')
                Field.setSFVec3f(fruit_trans_field, fruit_initial_translation)
                if j == max and k == max: fr = 0
        i += 1
        if i == 120: i = 0
    pass

    current_speed = speed_field.getSFFloat()

