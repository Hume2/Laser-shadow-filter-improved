[back to index](index.md)

# Tradr Laser Shadow Filter
This filter removes laser shadows from the scan. When the distances from laser data ascend rapidly, it's most likely a laser shadow. There are two angles, one is applied for far objects and the other is applied for the close objects on the sides of the robot.

## Parameters
* `min_angle_close` (in radians) - The minium ascend angle of the shadow. This one is applied for the close objects on the sides of the robot.
* `min_angle_far` (in radians) - The maximum ascend angle of the shadow. This one is applied for the rest objects.
* `far_distance_threshold` (in metres) - This is the threshold between the `min_angle_close` and `min_angle_far`. When the point is farther than this or the object is in the front, the `min_angle_far` is applied. And when the object is on the side and is closer than this, the `min_angle_close` is applied.

## Input and Output
The input is the data from a laser scanner. It should already pass through the [Near Filter](NearFilter.md).

The output is the filtered laser data. It should also pass some filters to remove the robot itself from the scans.
