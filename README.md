# Flight-simulator
Build tiny flight simulator and control it with washout algorithm. 

Hardware
----
* Control unit: Arduino Mega 2560 Rev3
* PC: Acer Swift3
* Flight data source: X-plane 10
* Attitude detection: MPU6050

Bottom platform
----
<img width="500" src=https://github.com/827983519/Flight-simulator/blob/master/440182283.jpg>

Upper platform platform
----
<img width="500" src=https://github.com/827983519/Flight-simulator/blob/master/1919900530.jpg>


Tiny stewart platform
----
<img width="300" src=https://github.com/827983519/Flight-simulator/blob/master/148566244.jpg>

Simple flight simulator
----
<img width="500" src=https://github.com/827983519/Flight-simulator/blob/master/1182037841.jpg>



Process
----
1. PC runs X-plane and output flight data via series port. 
2. Arduino get flight data and compute attitude angle.
3. Arduino control Servo to move upper platform.


outcome
----
### Start the plane, and see the change of acceleration as well as the output of washout algorithm
#### The acceleration of plane

<img width="500" src=https://github.com/827983519/Flight-simulator/blob/master/1182037841.jpg>








