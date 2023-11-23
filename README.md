# Introduction to Self-Driving Cars
The goal of the project is to design longitudinal and lateral controllers for a vehicle on a race track. The track trajectory is given as triples of (x, y, speed).

## Longitudinal Controller
A high-level PID controller was designed that receives desired speed as input and outputs desired acceleration. A low-level controllerwhich requires engine dynamics and torque-speed curves was not designed for this project.
Therefore, the acceleration output was directly taken as throttle/brake values. 

## Pure-Pursuit Lateral Controller 
A pure-pursuit lateral controller was tuned and employed to determine steering angles. The following is the formulation for pure-pursuit control:
$$\delta = atan\left(\frac{2 \\, L \\, sin(\alpha)}{l_d}\right)$$
where $\delta$ is the steering angle, $L$ is the wheel-base length, $\alpha$ is the look-ahead angle and $l_d = k_{dd} \\, v$ is the look-ahead distance set to 
a multiple of vehicle speed. 

![image](https://github.com/nimasarli/intro-to-self-driving-cars/assets/5862494/c56690d5-da80-469c-90f7-01b2dbf712c9)

## Stanley Lateral Controller 
A Stanley lateral controller was also tuned and employed to determine steering angles. The Stanely controller has two terms: (1) correction for heading error, 
(2) correction for cross-track error. \
Here is the formulation:
$$\delta = \psi_{des} - \psi + atan\left(\frac{k \\, e}{v_f+\epsilon}\right)$$
where $\psi_{des}$ and $\psi$ are desired (track) and current (vehicle) heading, $k$ is a gain, $e$ is the cross-track error, $v_f$ is the front-wheel speed and $\epsilon$ is a small scalar to prevent
division by zero.

![image](https://github.com/nimasarli/intro-to-self-driving-cars/assets/5862494/72d3b7c0-435f-4f6d-81ec-4a7ece19f4fb)


## Results 
The following shows the results for both lateral controllers in CARLA simulator. Note that steering angles are bounded to $[-70\degree, +70\degree]$. 
The longitudinal controller was tuned to have a short rise time and low settling time and steady-state error. As you can see in the speed plots, the controller is performing satisfactorily.  
As for lateral controllers, you'll notice that Stanley controller has sligtly improved performance in terms of cross-track error (it may be difficult to notice due to low video quality)

https://github.com/nimasarli/intro-to-self-driving-cars/assets/5862494/90b20a0c-4d87-488e-8133-5d3ef23f62c8


https://github.com/nimasarli/intro-to-self-driving-cars/assets/5862494/a7080448-8466-4593-aa9c-2783b31cabc7

