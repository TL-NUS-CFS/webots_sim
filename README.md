# webots_sim
## SGBA simulation 
1) Run the teste_puck.wbt world project
2) Choose the SGBA_Controller for the Khepera IV robot if not already chosen

![image](https://github.com/TL-NUS-CFS/webots_sim/assets/77558792/5047ae5b-c01c-4eaa-9474-fb7a5c3d2ec9)

## Outstanding bugs

* Some of the below bugs are because of the way the motor activation is coded. Need to amend the motor activation to better reflect the intended action of SGBA algorithm.  
 * Crashes if encounter wall at an angle
 * If lose wall, will keep turning indefinitely (also behaviour observed IRL) (see closer)
 * Turning is not optimal (also due to being non holonomic)
 * Sometimes will get stuck at wall due to crash
 * ~~Often loses corner when turning around wall~~
 * going forward, if near side wall, it will think it is encountering wall at angle

## To be implemented
* Loop detection + Reheading
* ~~Dynamic desired heading~~
* ~~Clockwise/AntiClockwise WallFollowing~~ + CA Highway


## To be checked
* Simulation, distance, robot scale
