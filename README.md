# CarND-Path-Planning-Project
Self-Driving Car Engineer Nanodegree Program

![](./images/pic1.JPG)

# Reflection

This code provides path planning for use with the term3 simulator and is written in C++ contained in the `main.cpp` source.  All reference to lines are in `src/main.cpp`.  

Initially the code accepts from the simulator the current car information x, y, s, d, yaw, car-speed along with the remaining previous path that was not used since the simulator invoked the planning code.  It also recieves sensor data about the other cars on the road i.e. the id, x, y, vx, vy, s, d of each car (See lines 396-414). 

## Same Lane Frontal Collision Avoidance ##
The first for loop starting on line 425 goes through all the other cars on the road and determines if one of them in my lane is on a collision course.  Determining if the other car is in same lane is done by calculating the lane range on line 434 through 438 based on the assumption of a 4 meter lane width.
```
float lbound = 2+4*lane-2;
float rbound = 2+4*lane+2;
```
The other car's s is then calculated out in to the future to match my car's s value which was set to the last value in the previous path...
```
if( prev_size > 0 ) {
   car_s = end_path_s;
}
...
check_car_s+=((double)prev_size*.02*check_speed)
```
The difference in the calculated position of the other car and my car is used to determine one of three levels of decleration on lines 446-456. Following this there is also code to determine if a car in another lane is crossing in to my lane and could cause a collision.  The deceleration level will be set to 4 causing maximum deceleration to avoid the collision (See lines 461-465). 

Once out of the loop that processes each car on the road, the reference velocity is set based on the level set above (lines 491-512). 


## Path planner/lane changer ##
The path planner function called `lane_changer()` will be invoked (line 520) if any of the deceleration levels were set, thus only looking for a new path if the current path was hindered.  The lane_changer() function goes through the loop of all the cars on the road and determines best routes of either stay in the current lane, change left, or change right.  This function will add cost to changing lanes as it goes. The variables kcost (stay in lane), rcost (lane change right), and lcost (lange change left) are increased as the situations are assessed.

The first check makes the cost very high too go left if the car is already in the far left or to go right and the car is already in the far right lanes.
The second check is done on whether my car could safely change to the left or right lane by looking for any cars beside my car (lines 222-243).

Another check is done to see if there is an accelerating car comming from behind in either side lane.  A calculation based on future location in 30 steps on all cars behind mine is done to see if my car and the other car will collide thus making the cost high to perform that action (See lines 290-316).

A choice, only if my car is in the center lane, is made between left and right lanes if the keep-lane cost is high; it chooses the lane right or left lane with the lowest cost (See lines 321-325).



## Spline ##
Now the code needs to determine what path points to send back to the simulator.  First, points are set up to be fed in to a spline class created by Tino Kluge to be used as the future projection for the simulator.  The first two points for the spline are set up in a way that will smoothly mesh with the previous end points (See lines 549-575). Three more points (x,y) will be added by using the `getXY()` function provided to map points 30, 60, and 90 "s" meters in the distance (line 578-580). These 5 points to be splined are shifted and rotated around the origin and then fed into the `set_points()` spline class method on line 601 to create our continous projection to follow in the future.


determined based on the current state of the vehicle (x, y, yaw), the desired velocity, desired lane, and project out 1 second (50 steps of 0.02 seconds) in the future

The points will be determined based on the current state of the vehicle (x, y, yaw), the desired velocity, desired lane, and project out 1 second (50 steps of 0.02 seconds) in the future.  These points will then be fed into a spline function provided by Tino Kluge which will provide a continous path.  Previous points not used by the simulator will be sent back to us and reused i.e. only points needed to fill the 50 point array will need to calculated and smoothly added to the end of the array.


The spacing of the points will determine velocity.

![](./images/Pic3.JPG)
![](./images/Pic4.JPG)
## Ticker ##

## Discussion ##
Although this planner works very well, many improvements could be made to this planner.  Use of a trained gaussian classifier predictive probably would have been a better choice for planning lanes but in the interest of time I went with a brute force approach which took a lot of tinkering to get it right.  

More consideration for crossing over to the far lane should have been done to get better timings.

It would have saved a lot of time to be able to control the other cars in the simulator while it is running to provide scenarios on demand for testing instead having to wait and see if it happens naturally by the simulator.



   

