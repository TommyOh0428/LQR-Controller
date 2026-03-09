# CMPT 419 Project Proposal

## Project Title

**Make LQR Controller a Nav2 Controller Plugin**

## Team Members and Roles

| Name (* indicates project contact) | Email |
|---|---|
| Tommy Oh | koa18@sfu.ca |
| Ansh Aggarwal* | aaa275@sfu.ca |
| Daniel Senteu | dms26@sfu.ca |
| JunHang Wu | jwa337@sfu.ca |

## 1. Abstract

This project implements a Linear Quadratic Regulator (LQR) as a C++ Nav2 Controller Server plugin for ROS 2 navigation. The controller will track the global path by locally stabilizing the robot around a reference trajectory using a linearized kinematic/dynamic model, producing smooth velocity commands while respecting basic limits. We will run a systematic experimental study on planner-controller compatibility and dynamics sensitivity by evaluating two robot models/dynamics configurations. We will benchmark against standard Nav2 DWB controllers with the NavFN planner plugin in simulation on a TurtleBot3 in Gazebo, using metrics such as path-tracking error, smoothness, success rate, and time-to-goal, and provide a reproducible evaluation pipeline and tuning guidelines that clarify when LQR improves navigation performance and where it breaks down.

## 2. Deliverables

### Interim Deliverables

Tasks to be completed by the end of week 4 (**Mar. 20**):

- Fully finished setup for the Nav2 controller
- Basic implementation of the LQR in C++ / LQR controller that can be compiled, but does not have to work
- Basic documentation of the current process and implementation

### Final Deliverables

Tasks to be completed by the end of week 8 (**Apr. 17**):

- Fully functional LQR Nav2 plugin
- A Gazebo simulation environment setup configured to test the controller
- The project report also compares the performance of the LQR controller and the default DWB controller in terms of control smoothness and computational time
- Well-documented repository and installation instructions with source code on GitHub

## References

- Optimal Control for Linear Dynamical Systems and Quadratic Cost (UC Berkeley)
  - https://people.eecs.berkeley.edu/~pabbeel/cs287-fa15/slides/lecture5-LQR.pdf
  - https://www.youtube.com/watch?v=S5LavPCJ5vw&t=2s
- Create Custom Plugins for ROS2 Navigation
  - https://www.youtube.com/watch?v=X128gB2lVF0
- Create Custom Controller Integration with Nav2
  - https://www.youtube.com/watch?v=OEkIWiLqCsk&start=445
- Official Nav2 Documentation
  - https://docs.nav2.org/
- Underactuated Robotics by Russ Tedrake, Chapter 8: Linear Quadratic Regulators
  - https://underactuated.csail.mit.edu/lqr.html
