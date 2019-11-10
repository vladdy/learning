## Intro

This is a reflection on my solution to the CarND Term 2 Model Predictive Control (MPC) Project

## The Model

The vehicle model uses the following parameters
* state described by coordinates (x, y) and orientation angle (psi)
* velocity (v)
* cross-track error (cte) and psi error (epsi)
* the actuator of vehicle with angle(delta) and acceleration (a)

The model uses the state and actuations from the current time (t) and calculates the next time (t+1) as follows:

![model](model.png)

## Timestep Length and Elapsed Duration (N & dt)

I ended up with N=10 and the elapsed duration dt=0.1 with reference velocity ref_v=70 as reasonable parameters
allowing me to achieve the goal in this solution. The elapsed duration is the same as the connection latency. 
The parameter space I chose from was N = [5; 10; 15; 25] and dt = [0.05; 0.1; 0.2; 0.25]. For parameters I discared, the model produced 
either unstable trajectory with the vehicle getting off the road or had no difference from the final 
ones consuming more resources (that happened for larger N).

## Polynomial Fitting and MPC Preprocessing

Preprocessing included a transformation of the waypoints from the global coordinates to the vehicle coordinates 
(see main.cpp, lines 104-112). Then, the waypoints were approximated by a 3rd degree Polynomial Fitting 
(see file main.ccp, line 114). As a result, the waypoints in the vehicle's coordinates use the origin x=0 and y=0 and 
the orientation angle is psi=-90 degrees.

## Dealing 100ms latency

The model uses the next predicted value for the state (see main.cpp, lines 119-130). When the time difference between
the fist state and that predicted state is equal to the "artificial" delay from this lab (d = 0.1s) emulating the
connection latency, the vehicle is able to stay within the allowed area on the road. Other differences, were not that
good causing it to move out of the road.
