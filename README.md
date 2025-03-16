# CoppeliaSim Mobile Manipulation

## Project Summary

This project focuses on implementing mobile manipulation using the **youBot**, a mobile manipulator featuring a four-Mecanum-wheel base and a 5R robotic arm. The primary goal is to control the robot to pick up a block from a specified location, transport it to a target position, and place it down accurately. Key tasks include generating a reference trajectory for the end-effector, performing odometry as the chassis moves, and applying feedback control to ensure precise motion execution.

## Cases Overview

The project includes three distinct cases: **Best Case**, **Overshoot**, and **NewTask**. Each case has its own script and corresponding CSV files located in the project folder. To run a case, download the entire code folder and execute the individual case script in your environment.

### Best Case

In the **Best Case** scenario, the youBot executes the task with optimal trajectory tracking and minimal error, successfully picking, transporting, and placing the block as intended.

![Demo](https://github.com/Allenwu1122/CoppeliaSim-Mobile-Manipulation/tree/main/recording/Best_Recording.gif)

### Overshoot

The **Overshoot** case demonstrates a scenario where the robot overshoots the target due to excessive joint velocities or inadequate feedback tuning, resulting in positioning errors during the task.

### NewTask

The **NewTask** case introduces a modified or alternative manipulation task, showcasing the adaptability of the control system to new objectives or configurations.

## Setup Instructions

1. Download the entire `CoppeliaSim-Mobile-Manipulation` folder.
2. Open your simulation environment (e.g., CoppeliaSim) and MATLAB.
3. Run the desired case script (e.g., `BestCase.m`, `Overshoot.m`, or `NewTask.m`) to simulate the corresponding scenario.
