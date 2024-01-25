# webots_sim
## SGBA simulation 
1) Run the teste_puck.wbt world project
2) Choose the SGBA_Controller for the Khepera IV robot if not already chosen

## Outstanding bugs
* Khepera is holonomic
 * Cannot do Y adjustment like crazyflie. Drifts when following long wall.
 * Cannot turn on the spot

* Some of the below bugs are because of the way the motor activation is coded. Need to amend the motor activation to better reflect the intended action of SGBA algorithm.  
 * Crashes if encounter wall at an angle
 * If lose wall, will keep turning indefinitely
 * Turning is not optimal (also due to being non holonomic)