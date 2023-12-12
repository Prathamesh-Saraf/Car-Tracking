# Car-Tracking
Project for course ECE 272B - Filtering and Estimation

## Given

* Two radar with positions (0, 0) and (0, -1000) are given. We are given 61 radar measurements. The interval between the radar measurements is 5 seconds. The range error of the radar is 95m and bearing error is 30 milliradian.
* We are also provided with speed ($(v_x, v_y)$). We are provided the map of the path which the object to be tracked takes.
* The start point of the object is (-500, -400).

## Notes:

* Piecewise EKF seems to be working well. But the question is how can we genralise it.
* Models to be compared:
  * Piecewise EKF where we know when the car will turn and model manually.
  * Piecewise EKF where the turns will be incorporated in the motion model.
  * Average the radar readings and get a trajectory.
  * Use the constant velocity model and generate trajectory. We'll use the radar readings to get parameters for velocity (for nominal velocity given).
  * Velocity model where we calculate the velocity using the radar data.
  * One EKF applied throughout the implementation (Although this does not makes sense mathematically).
* Four models should suffice:
  * Radar data average
  * Velocity model with no filtering
  * EKF
  * Peicewise EKF
* We could mention during presentation other models which were mentioned but did not put in the presentation for the sake of completeness.
* For generalising the piecewise EKF, we could use the information of the map. Since, we know the covariance from EKF. If the gaussian ellipse includes the intersection region in the map we could use a transition probability matrix in the motion model.
* Now, the question would be how can we update in the covariance in this case. 
* State space options:
  * $(x, y, v_x, v_y)$ :
    * Here if we use nominal velocity in the motion model, we can use the difference in the radar measurement as observation model.
    * covarinace matrix can be built using the radar measurement.
    * But would we use the velocities in estimating the observation wrt states Intuitively this feels like we would have one more positional information to take into account. But the Kalman is devised as bayesian filter with markov assumptions. So, this doesn't seem like it would help.
    * If we are able to appropriately formulate it then this would be the best.
  * $(x,y)$:
    * This seems like best choice mathematically. We would only need to appropriately calculate initial covariance.
    * The only problem is that we need to use the nominal velocity. 
    * Experiment with using the velocity from radar.
* Corners:
  * (-500, -400), (-500, -200), (-350, -200), (-350, -800), (-950, -800), (-950, 200), (-550, 200), (-550, -400), (-750, -400)
* Now will assign the intersection directions for each:

|Coordinate|Intersection|Heading|
|----------|------------|-------|
|(-500, -400)|North (start)|North|
|(-500, -200)|North, East|East|
|(-350, -200)|North, South|South|
|(-350, -800)|South, West|West|
|(-750, -800)|South, North, West|West|
|(-950, -800)|South, North|North|
|(-950, -400)|North, East|North|
|(-950, 200)|North, East|East|
|(-750, 200)|North, South, East|East|
|(-550, 200)|North, South|South|
|(-550, -400)|South, West|West|
|(-750, -400)|South, North, West|North|
--------------------------------






## TODO:

* Implement the piecewise one piece for constant velocity model with nominal velocity - Done
* Implement the piecewise one piece for constant velocity model with velocity from radar - Done; Results reasonable but not as good as previous one; also not properly formulated as the Q is calculated from the given data (could make this to calculate on the fly; also think about using the velocity incremental std with this) - HALF Done
* Verify mathematical formualtion for velocity state vector - Yet to do
* Implement piecewise completely - yet to do
* Get the calculation of the initial covariance for the position - yet to do