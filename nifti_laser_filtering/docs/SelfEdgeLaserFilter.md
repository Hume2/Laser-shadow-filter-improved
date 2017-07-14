[back to index](index.md)

# Self Edge Laser Filter
This filter removes the shadows that the robot casts. It is expected that the robot has no sharp edges. It always finds an edge and tries to remove the trail as is possible.

## Parameters
* `max_edge_distance` (in metres) - The maximum distance of an edge to be proccessed. Note that this is only the distance of the edge. When the shadow continues behind this value, it's still filtered.
* `min_filter_angle` (in radians) - The minimum angle of the edge. Note that the angle might look bigger in the cross-section.
* `max_filter_angle` (in radians) - The maximum angle of the edge. Note that the angle might look bigger in the cross-section.
* `delta_threshold` (in metres) - It's expected that the shadows are lines. However, there is also some noise, so this value says, how much the points can differ.
* `min_valid_distance` (in metres) - There is another function of this filter. It also removes the points those are very close. When the distance is less than `min_valid_distance`, the point gets cut off.
* `after_trail_points` - This is the count of points those get removed after the trail. This was added because sometimes a point is out of the trail, but the trail still continues.
* `use_vertical_filtering` (True or False) - This option allows to consider the data from previous frames. It filters a bit more points when it's on. Vertical filtering makes the same as the normal (horizontal) filtering, but vertically.

## Input and Output
The input is the data from a laser scan. It's advertised to use the [Tradr Shadow Filter](TradrShadowFilter.md) first.

The output is the filtered laser scan data. It still doesn't remove everything, it's advertised to use also the *Island Filter*.

## Example Configuration
```yaml
    -   name: SelfEdgeLaserFilter
        type: nifti_laser_filtering/SelfEdgeLaserFilter
        params:
            max_edge_distance: 0.5  # metres
            min_filter_angle: 2.450  # radians
            max_filter_angle: 2.850  # radians
            delta_threshold: 0.01  # metres
            min_valid_distance: 0.2 # metres
            after_trail_points: 10
            use_vertical_filtering: True
```
