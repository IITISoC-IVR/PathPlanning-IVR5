**Abstract:** Autonomous racing is a research field gaining large popularity, as it pushes autonomous driving algorithms to their limits and serves as a catalyst for general autonomous driving. The basic idea for our PS is to generate an optimal trajectory which when followed by the Race car with a certain velocity profile will result in reduced lap times. However, in order to reach this objective, the driver has to keep the car between the track limits while trying to get the best out of the car based on the power and grip conditions.
It is to be noted that for this project we will consider 1/10th model of an actual race car since they are mechanically very close to full-scale cars.
The first part is to generate optimised trajectories which then has to be followed by the vehcile to minimize the lap time. The first method is the Shortest Path approach in which we minimize the path travelled by the car. The results of the MATLAB code for this method are shown in the figure below:

![SilverStone_Short](https://github.com/rakshitjangid22/PathPlanning-IVR5/assets/116636404/bb939786-57ec-4644-b986-eda212466598)
![Austin_Short](https://github.com/rakshitjangid22/PathPlanning-IVR5/assets/116636404/69873362-7702-4755-9321-8d744f9ccc04)

Another approach is the Min Curvature method which minimizes the curvature of the path in each segment of the Race track and results in higher speeds. The results for the same are shown in the figure below:
![SilverStone_min](https://github.com/rakshitjangid22/PathPlanning-IVR5/assets/116636404/c18c8c56-b646-463b-9d1c-7881cb5a1a7d)
![Austin_min](https://github.com/rakshitjangid22/PathPlanning-IVR5/assets/116636404/2b04dd4b-9b12-4a1e-bb12-21306e9ad696)

Now, The Optimal Trajectory is the combination of both the above approaches since Shortest Path contributes to least travelled distance and Mininmum Curvature contributes to higher speeds. Hence we create a new optimisation function which is the combination of both the approaches: O<sup>2</sup> = (1- &epsilon;)S<sup>2</sup> + &epsilon;C<sup>2</sup>

![SilverStone_final](https://github.com/rakshitjangid22/PathPlanning-IVR5/assets/116636404/143f7b24-0923-4971-8098-9186da79ff06)
![Austin_final](https://github.com/rakshitjangid22/PathPlanning-IVR5/assets/116636404/bf13594c-1b7b-4a69-986a-886287bff94a)

The we made a Vehicle-Driver model in Simulink, which takes into account all the forces and Vehicle Dynamics Equations and comprises a Lateral Control- Pure Pursuit Controller(our original code is not working in the Simulink file and hence we have replaced this code with a much simpler code and the original one is attached in the repo by the name- PurePursuitOwn) and a Longitudinal Control. 
We aim to fix the errors in the Simulink File and make this model work.

Following is the table in which we have added the Full forms of the abbrevations used:
|ShortForms| Name|
|---|---|
|GR1|Gear Ratio 1|
|GR2| Gear Ratio 2|
|I_wheel| Moment of Inertia of Car about the z-axis|
|l_wb|length of wheel base|
|mCar| mass of car|
|reffWheel| effective radius of the wheel|
