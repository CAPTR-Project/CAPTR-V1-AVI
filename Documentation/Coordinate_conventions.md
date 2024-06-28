# Sign and coordinate conventions

## Coordinate frames

![](bodycs.gif?raw=true)

### Local frame
Uses FRD (Front Right Down) coordinate frame, BUT modified to accomodate the orientation of the rocket. **"Front" is actually pointing UP**. 

#### If the rocket is sitting on the pad, **X is Up, Y is Right, Z is Front.**

## Rotation

Euler angles should use 3-2-1 (Yaw-Pitch-Roll) convention. Yaw is about the Z axis, Pitch is about the Y axis, Roll is about the X axis. CCW is +ve when viewing the axis straight-on.

![](rotations.gif?raw=true)

Quaternions are standard representation


### Global frame
Uses East North Up coordinate frame. X is East, Y is North, Z is up. **Note that this is very different from the local frame.**