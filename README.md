## Payload release dynamics with UAV not moving in the direction of wind.

The following documentation describes the payload trajectory when the payload is dropped from
moving UAV. This documentation assumes that the UAV is not moving along the direction of
wind.
The data we know beforehand:

- Velocity of wind in north direction
- Velocity of wind in east direction
- Current coordinate of UAV
- Target coordinate of UAV

The code you should see is **payload_drop_relative.m**

### The output of payload_drop_relative.m is shown below

#### MATLAB PLOT RESULTS

The uav direction is NORTH EAST and under different cases of wind direction, the release point and target point is plotted
1) North 0 m/s , East 0 m/s

![N0E0](/results/N0E0.jpg) 
![N0E0G](/results/N0E0G.jpg)

2) North 10 m/s , East 0 m/s

![N10E0](/results/N10E0.jpg)
![N10E0G](/results/N10E0G.jpg)

3) North 0 m/s , East 10 m/s

![N0E10](/results/N0E10.jpg)
![N0E10G](/results/N0E10G.jpg)

4) North 10 m/s , East 10 m/s

![N10E10](/results/N10E10.jpg)
![N10E10G](/results/N10E10G.jpg)

5) North -10 m/s , East 0 m/s

![N-10E0](/results/N_10E0.jpg)
![N-10E0G](/results/N_10E0G.jpg)

6) North 0 m/s , East -10 m/s

![N0E_10](/results/N0E_10.jpg)
![N0E_10G](/results/N0E_10G.jpg)

7) North _10 m/s , East _10 m/s

![N_10E_10](/results/N_10E_10.jpg)
![N_10E_10G](/results/N_10E_10G.jpg)
