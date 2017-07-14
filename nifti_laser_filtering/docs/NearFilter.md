[back to index](index.md)

# Near Filter
The points close to the robot appear to be closer than they really are. This filter simply fixes the deformation. All values are hard-set, so this filter doesn't have any parameters.

## Parameters
*none*

## Input and Output
The input is the raw data from the laser scanner.

The output is the data where the deformation is eliminated.

## Example Configuration
```
    -   name: NearFilter
        type: nifti_laser_filtering/NearFilter
```
