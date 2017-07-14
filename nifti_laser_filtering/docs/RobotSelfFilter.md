[back to index](index.md)

# Robot Self Filter
This is a filter that removes the robot itself from the laser scan.

## Parameters
* `robot_frame` (string) - The frame of the robot body, usually `base_link`. It has to be usable as a fixed frame wrt the laser frame. Especially, it can't be laser_frame itself in case the laser is rotating.
* `min_distance` (in metres) - The minimum distance of points from the laser to keep them.
* `max_distance` (in metres) - The maximum distance of points from the body origin to apply this filter on.
* `inflation_scale` - A scale that is applied to the collision model for the purposes of collision checking (1.0 = no scaling). Every collision element is scaled individually with the scaling center in its origin.
* `inflation_padding` - A constant padding to be added to the collision model for the purposes of collision checking (in meters). It is added individually to every collision element.
* `robot_description_param` (string) - This is the name of the parameter where the robot model is stored.
* `wait_for_transform` (in seconds)
* `compute_bounding_sphere` (True or False) - This toggles wheter the bounding sphere is going to be calculated.
* `compute_bounding_box` (True or False) - This toggles wheter the bounding box is going to be calculated.
* `compute_debug_bounding_box` (True or False) - This toggles wheter the debug bounding box is going to be calculated.

## Input and Output
The input is the data from a laser scan. Shadows should be already filtered, because it might be impossible to filter the shadows after applying this filter.

The output is the filtered data. The remaining glitches can be filtered using the [Island Filter](IslandFilter.md).

## Example Configuration
```
    -   name: RobotSelfFilter
        type: robot_self_filter/RobotSelfFilter
        params:
            robot_frame: 'base_link'
            min_distance: 0.0
            max_distance: 2.0
            inflation_scale: 1.15
            inflation_padding: 0.01
            robot_description_param: '/nifti_robot_description'
            wait_for_transform: 0.5
            compute_bounding_sphere: True
            compute_bounding_box: True
            compute_debug_bounding_box: False
```
