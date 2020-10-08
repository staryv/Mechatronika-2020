"""Missile_LS controller."""

# CODE 1

from math import *
from controller import *
from time import perf_counter 
import csv
import os

# create the Robot instance.
sup = Supervisor()
timestep = int(sup.getBasicTimeStep())
KB = Keyboard()
Keyboard.enable(KB,timestep)
LS1=LightSensor("LS1")
LS2=LightSensor("LS2")
LS3=LightSensor("LS3")
LS4=LightSensor("LS4")
robot_node = sup.getFromDef("Missile")
robot_node1 = sup.getFromDef("ROT1")
trans_field = robot_node.getField("translation")
rot_field = robot_node.getField("rotation")
rot_field1 = robot_node1.getField("rotation")
motor = sup.getMotor("motor")
target_node = sup.getFromDef("Target")
target_field = target_node.getField("translation")
Motor.setPosition(motor, inf) 
Motor.setVelocity(motor, 100)  
display=Display("display")
LightSensor.enable(LS1, timestep)
LightSensor.enable(LS2, timestep)
LightSensor.enable(LS3, timestep)
LightSensor.enable(LS4, timestep)

if os.path.isfile('C:/Users/staryv/Disk Google (1)/CSV/data.csv'):
 os.remove('C:/Users/staryv/Disk Google (1)/CSV/data.csv')

# get the time step of the current world.


# You should insert a getDevice-like function in order to get the
# instance of a device of the robot. Something like:
#  motor = robot.getMotor('motorname')
#  ds = robot.getDistanceSensor('dsname')
#  ds.enable(timestep)

# Main loop:
# - perform simulation steps until Webots is stopping the controller
angle =0.085
angle_azim= -0.025
rot_field.setSFRotation([0,1,0,0])
rot_field1.setSFRotation([0,0,1,0]) 
while sup.step(timestep) != -1:
    # rot_field.setSFRotation([1,0,0,0])
    LS1val = LightSensor.getValue(LS1)
    LS2val = LightSensor.getValue(LS2)
    LS3val = LightSensor.getValue(LS3)
    LS4val = LightSensor.getValue(LS4)
    
    #print(LS1val,LS2val,LS3val,LS4val)
    position = trans_field.getSFVec3f()
    rotation = rot_field.getSFRotation()
    rotation1 = rot_field1.getSFRotation()
    target_position = target_field.getSFVec3f()
    dist = round(sqrt(position[0] * position[0] + position[1] * position[1] + position[2] * position[2]),2)
    target_distance = round(sqrt(target_position[0] * target_position[0] + target_position[1] * target_position[1] + target_position[2] * target_position[2]),2)
    xm=round(position[0],2)
    ym=round(position[1],2)
    zm=round(position[2],2)
    xt=round(target_position[0],2)
    yt=round(target_position[1],2)
    zt=round(target_position[2],2)
#    azim=atan(abs(xt-xm)/abs((zt-zm)))
#    elev=atan((yt-ym)/abs((zt-zm)))
     #print(atan((target_position[2]-position[2])/(target_position[0]-position[0])))
    time=round(perf_counter(),2)
    missile_speed=round(dist/time,2)
    row_data=(time,xm,ym,zm,xt,yt,zt)
    print(position,target_position,"speed=",missile_speed)
    with open('C:/Users/staryv/Disk Google (1)/CSV/data.csv', 'a',newline='') as csvfile:
        writer = csv.writer(csvfile)
        writer.writerow(row_data)
#    print(azim,elev,rotation, rotation1)
    rot_field.setSFRotation([0,1,0,0])
    rot_field1.setSFRotation([0,0,1,0])
        
    Display.drawPixel(display,round(int(xm),2),round(int(ym),2))
    Display.drawPixel(display,round(int(xt),2),round(int(yt),2))
     
    if time <=1.5:
       rot_field.setSFRotation([0,1,0,-0.8])
       rot_field1.setSFRotation([0,0,1,1.2])
    else:
       if LS1val >= LS2val and LS3val >= LS4val:
       rot_field.setSFRotation([0,1,0,(-atan((target_position[2]-position[2])/(target_position[0]-position[0])))+angle_azim])
       rot_field1.setSFRotation([0,0,1,angle+atan((target_position[1]-position[1])/(target_position[0]-position[0]))]) 
       if LS2val > LS1val and LS3val >= LS4val:
           rot_field.setSFRotation([0,1,0,-atan((target_position[2]-position[2])/(target_position[0]-position[0]))])
           #rot_field1.setSFRotation([0,0,1,rotation1[3]+angle])
           rot_field1.setSFRotation([0,0,1,angle+atan((target_position[1]-position[1])/(target_position[0]-position[0]))]) 
       if LS1val >= LS2val and LS3val < LS4val:
           rot_field.setSFRotation([0,1,0,-atan((target_position[2]-position[2])/(target_position[0]-position[0]))])
           rot_field1.setSFRotation([0,0,1,atan((target_position[1]-position[1])/(target_position[0]-position[0]))])       
       if LS2val > LS1val and LS3val < LS4val:
           rot_field.setSFRotation([0,1,0,-atan((target_position[2]-position[2])/(target_position[0]-position[0]))])
           rot_field1.setSFRotation([0,0,1,atan((target_position[1]-position[1])/(target_position[0]-position[0]))])        
 

        if abs((xt-xm)) <=1:
            if abs((yt-ym)) <=1:
                if abs((zt-zm)) <=1:
                    print("Hit!","Distance",(target_distance - dist),"m,","time to hit=",time,"s")
                    break
        if time >= 7:
            print("Miss")
            break
