# 6-DOF Arm Robot Kinematics

This project is created to help intermediate to amateur robotics developers to understand more about kinematics in 6-DOF robot arm. There is a python tkinter based app to give an input (Angle or Position) to the STM32F401CCU6 microcontroler that I used in this project. All the mathematical calculation is done by the STM32, it can helps when you want to directly simulate it with actuator like stepper or servo motors.

## Python APP Interface
1. Serial com tab
   In this tab you can connect the STM32 to the python app via USB and you can configure the 'Com Port' and 'Baud Rate'
3. Forward kinematics tab
   In this tab you can give some joint angle input to get the tool position of the robot
5. Inverse kinematics tab
   In this tab you can give some toll position input to get the joint angle of the robot

## DH Parameter
1. There are some example of
