# CarND-Controls-PID
Self-Driving Car Engineer Nanodegree Program

---

# Intro
The PID class developed for this project includes a few features that allows the car to drive around the track safely up to 35mph. 
Two PID objects have been used, one for the steering control and another one for the throttle.

# PID
The hyperparameters have been adjusted experimentally. During this phase of tuning, there were a found few challenges that the control had to face:

* The differential of the error explained in Sebastian's video did not take into account the differential time between two iterations. By using the `chrono` library, I noticed that the time between two control actions was not constant. For this reason, I saw that the control output was jittering as I increased the KD. This was corrected by dividing the d_error by the dt.

* The Integral (I) is always adding up. To avoid the wind-up effect, the error has been limited to a value once the I term of the PID reaches 0.1. Also, the Integral is reseted once the CTE changes its sign. This avoids the car continuing commanding to one direction even when the error is 0.

* The Differential (D) also has been limited to 0.1 to avoid jumps in the control output.

* Also, to avoid the jittering effect due to jumps in the differential of the error, a `LowPassFilter` library has been added [https://github.com/overlord1123/LowPassFilter]. This low-pass filter smoothes the d_error before it is multiplied by the Kd gain. This filter has been tuned manually and it produces a good results with a cut-off frequency of aproximate 95Hz.

* As the car get faster, it is strongly recommendable to lower the gains. To do this, a simple linear gain scheduler has been added in the `main.cpp` file. This scheduler simply reduces the gain from a known value to another one as the car increases speed.

* The throttle control takes the CTE and decreases the car speed from a target value to 0 as the position error increases. Due to the engine slow dynamics in the simulator, the controller did not need a Differential term.

# Known issues and future work
* The car still oscillates in some sections of the track.

* The twiddle algorithm for autotunning was found difficult to implement with the simulation enviornment. If the PID changes its parameters as the car drives, there could be some sections of the track that the car needs 'more gain' than in others. To do it properly, it may be necessary to calculate the average error and oscillations (bandwidth) for a whole lap and then repeat the algorithm again. Instead of using a 'real time' simulator, a more analytic tool like MATLAB would help to speed up the autotunning process.

* The throttle PID shall also take into consideration the speed of the car to avoid making the car throttle command oscillate as the CTE changes.

* During corners, the PID control is not robust enough to keep the car on track without oscillations. A more complex control architecture that predicts the corner curvature would help to make the cornering smoother.

* If the car is driving at more than 35 mph, the PID control is not running fast enough to make the proper corrections to the steering.

