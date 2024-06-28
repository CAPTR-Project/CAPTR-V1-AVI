# Sign and coordinate conventions

## Coordinate frames

![](bodycs.gif?raw=true)

### Local frame
Uses FLU (Front Left Up) coordinate frame. X is front, Y is Left, Z is up.

## Rotation

Euler angles should use 3-2-1 (Yaw-Pitch-Roll) convention. Yaw is about the Z axis, Pitch is about the Y axis, Roll is about the X axis. CCW is +ve when viewing the axis straight-on.

![](rotations.gif?raw=true)

Quaternions are standard representation


### Global frame
Uses East North Up coordinate frame. **Note that this will be perceived as "rotated" 90 degrees clockwise from the local frame, since us humans like to think north as forward.**