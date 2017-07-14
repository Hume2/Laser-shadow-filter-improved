[back to index](index.md)

# Island Filter

This filter cuts off all lonely island, those are probably left from the previous filters. It always passes through the island, looks whether the island is small enough and then it might get deleted. An island is an isolated cluster of data that is not connected to the neighbour data. There are no valid data outside the island, or the data outside the island is much farther that the island. There is also a *skip cone* in front of the robot, where no islands can be filtered.

When the island isn't in the skip cone, at least one of those conditions must be met to delete the island:
* The count of points of the island is less than `max_count`.
* The count of points in the previous frame on the smae position is less than `max_big_rise`.

## Parameters
* `max_distance` (in metres) - The maximum distance of an island to be filtered. Farther islands are not filtered.
* `min_wall_distance` (in metres) - The minimum distance between the island and the background. When the background is closer, the island is not considered to be an island. When there is no background or the background is farther than `max_distance`, the island is always considered as an island. The island must be always in front of the background.
* `skip_cone` (in radians) - The angle between the laser scan start and the start of the *skip cone*. The end of the *skip cone* is symetric. No islands are filtered inside the *skip cone*.
* `max_count` - The maximum count of points of the island to be filtered. The island might be still filtered even when it contains more points, but there were less points in the previous frame. There is an extra paragraph about this.
* `max_big_rise` - The maximum count of points in the previous frame to the current island be filtered. This allows a bigger islands to be filtered when it looks like to be a fake. There is an extra paragraph about this.

## Input and Output
The input is a data set which has passed all the filters before, but there are still minor glitches arround the robot.

The output is a data set without the minor glitches. Some glitches might still appear, but they never appear on the same place twice. Therefore the data is then ready for evaluating.

## max_count and max_big_rise
There are two types of islands, the small ones and the big ones. The point count of small ones are always less than `max_count` and the big islands are the rest islands. The idea is that the small islands are fake, so they can be cut off. However, setting the `max_count` might be problematic. There are even quite big islands those are fake, but there are also many small islands those are real, mainly in the ground.

This results in setting the `max_count` to a quite big number, because some fake islands are really big. This is a problem, because it reduces the robot's perception rapidly for the close objects. Imagine that there is a vertical pole near the robot. When the `max_count` is set to a big value, the robot can never see that pole, because it is considered to be a fake island in all of the scans. However, the fake islands are usually just a few frames thick. Therefore the filter always compares the data with the data from the previous frame. When there is a big island and there was a big island even in the frame before, it's considered to be a real island. And when there were just several points (less than `max_big_rise`), it's considered to be a fake. The `max_big_rise` says, how many points there can be before the big island to be filtered.

## Setting the parameters
Setting the parameters for the [Island Filter](IslandFilter.md) is always finding a good compromise. Setting the parameters to low values results in more fake islands to be left in the data, while the high values result in reducing the robot's perception for close objects rapidly. By default the `max_count` is set to 8 and the `max_big_rise` to 8. This makes the scans very clean and there are still many real points left near the robot.

Also note that other filters might scatter the real data to small islands, so it also depends on the settings of the other used filters.

## Example Configuration
```yaml
    -   name: IslandFilter
        type: nifti_laser_filtering/IslandFilter
        params:
            max_distance: 0.8 # metres
            min_wall_distance: 0.05 # metres
            skip_cone: 1.9 # radians
            max_count: 8
            max_big_rise: 8
```
