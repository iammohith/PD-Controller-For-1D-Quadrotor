# PD-Controller-For-1D-Quadrotor
This project implements a Proportional-Derivative (PD) control system for a one-dimensional (1D) quadrotor. The objective is to stabilize the quadrotor's position along a single axis while counteracting disturbances and ensuring smooth motion. The PD controller computes corrective actions based on the position error (difference between desired and current positions) and its rate of change, allowing for precise and responsive control. This project is ideal for understanding control theory concepts, experimenting with feedback mechanisms, and applying them in a simulated environment.

## Introduction  
The goal of this project is to get familiar with working with a quadrotor simulator and implementing a Proportional-Derivative (PD) controller. The PD controller is designed to manage the vertical motion of the quadrotor by adjusting thrust to achieve stability and respond to specific height requirements.  

### Technical Details  

Physical properties of quadrotor, mass of quadrotor is 0.18($kg$) and length of arm of the quadrotor is 0.086($m$).

![Quadrotor Schematic](Quadrotor_Schematic.png)

The dynamic equation governing the motion of the quadrotor in the vertical ($z$) direction is given by:  

$$
\ddot{z} = \frac{u}{m} - g
$$  

Where:  
- $\ddot{z}$: Acceleration in the $z$-direction. $(m/s^2)$  
- $u$: Control input (thrust force). $(N)$  
- $m$: Mass of the quadrotor. $(kg)$
- $g$: Gravitational acceleration. $(m/s^2)$
- $u_{min}$ is 0 $(N)$
- $u_{max}$ is $1.2mg$ $(N)$  

#### PD Controller  
The control input $u$ for the Proportional-Derivative (PD) controller is defined as:  

$$
u = m(\ddot{z}_{\text{des}} + K_p e + K_v \dot{e} + g)
$$  

Where:  
- $\ddot{z}_{\text{des}}$: Desired acceleration in the $$z$$-direction. $(m/s^2)$
- $K_p$: Proportional gain.  
- $K_v$: Derivative gain.  
- $e = z_{\text{des}} - z$: Position error (difference between desired and current height).  
- $\dot{e} = \dot{z}_{\text{des}} - \dot{z}$: Velocity error. 

In this project, I have implemented a custom PD controller to control the height of a 1D quadrotor and tuned its gains $K_p$ and $K_v$ for optimal performance. The controller is tested with two distinct cases:  

1. **Stabilization at Zero Height**:  
   The quadrotor needs to maintain stability at a height of 0 meters.  

2. **Step Input Test**:  
   The quadrotor is given a step input to rise to a height of 1 meter and maintain this position. 

## Features  
- **PD Controller Implementation**:  
  Implements a custom Proportional-Derivative (PD) controller to manage the quadrotor's vertical position.  

- **Simulation Environment**:  
  Provides a realistic simulation of the quadrotor's dynamics under the control of the PD system.  

- **Graphical User Interface (GUI)**:  
  Includes an interactive GUI for:  
  - Simulating the quadrotor's motion.  
  - Visualizing the controller's response over time.

## Requirements
- MATLAB (preferably R2018b or later)

## Usage
Clone this repository and run the `simulation.m` file
```matlab
params - A structure containing the following fields:
   params.gravity    - Gravitational constant (m/s^2)
   params.mass       - Mass of the quadrotor (kg)
   params.arm_length - Length of the quadrotor arm (m)
   params.u_min      - Minimum control input (thrust)
   params.u_max      - Maximum control input (thrust)
```

## **File Structure**  
The project consists of the following MATLAB functions:

- **`simulation.m`**:  
   The main function that initiates the simulation. It connects the desired trajectory, PD controller, and dynamics, and visualizes the quadrotor's height control performance.

- **`fixed_set_point.m`**:  
   Generates the desired state trajectory for the system. For all times $t > 0$, the desired state remains constant at $[z_{\text{des}}; 0]$, where $z_{\text{des}}$ is the desired position along the $z$-axis.

- **`controller.m`**:  
   Implements a Proportional-Derivative (PD) controller to regulate the height of the quadrotor. The controller computes the thrust required based on position and velocity errors.

- **`height_control.m`**:  
   Simulates the closed-loop height control system of the quadrotor. It applies control inputs, integrates the system's equations of motion, and updates the state over time.

- **`libs/`**:  
   Contains helper functions for the simulation:  
   - **`QuadPlot.m`**:  
     Visualizes the quadrotor's position and trajectory over time, including motor positions and historical movement.  
   - **`QuatToRot.m`**:  
     Converts a unit quaternion into its corresponding $3 \times 3$ rotation matrix. Useful for orientation computations.  
   - **`quad_pos.m`**:  
     Calculates the quadrotor's position in the world frame based on its state.  
   - **`simStateToQuadState.m`**:  
     Converts simulation state data into the 13-element state vector format:  
     $[x, y, z, \dot{x}, \dot{y}, \dot{z}, q_w, q_x, q_y, q_z, p, q, r]$.  
   - **`sys_eom.m`**:  
     Defines the equations of motion (EOM) for the quadrotor system. For 1D motion, it simplifies to:  
     $m\ddot{z} = F_{\text{thrust}} - mg$  
     where $F_{\text{thrust}}$ is the thrust generated by the controller.  
   - **`sys_params.m`**:  
     Sets system parameters, such as the quadrotor's mass $m$, gravitational acceleration $g$, controller gains ($K_p$, $K_d$), and simulation time step.
