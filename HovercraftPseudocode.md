
- stablity in x direction going forward: the angle of the thrust fans (yaw) should counter the angle of the body so that it goes straignt (corrects direction)
- only stops and turns sensor (scans) when a certain distance away from a wall (stopDistance)
- we should develop an accurate coordinate system for body and inertial reference frames (to not backtrack etc.)
- PI controller for yaw angle
- always following path to longest distance
#### Scanning
- stop lift and thrust fans before scanning (put US on servo)
**deciding where to turn**:
- var maxDistance
- var angle
- maxDistance and angle are overwritten every time the sensor detects a further distance while scanning (open space to move into)
- at end of scan it returns the angle that has the maxDistance (open path indicator)
- if distance on both sides is the same, goCrazy() {thrust fan left and right until it gets unstuck// one distance is larger than the other}
**deciding how much to turn**:
- keep turning until distance spikes, at that point stabillize (compensate for oversteering)

- interrupt for scanning, whenever linear acceleration in all directions = 0 (stuck)


#### Edge cases:
- Same distance on both sides of hovercraft
- Out of range (too close) for US, blocked by wall
- Turning 180º and turning back 
