# Nifti Laser Filter Documentation

This is a set of ROS filters used for filtering the laser scan input. It filters the laser shadows, the robot itself and some other things those aren't the real part of the environment. The input is simply the data from the laser scanner and the output is the filtered data.

Althrough this filter removes the major part of fake measurements from the data set, some minor glitches might still occur. However, they appear always only once on the given position, so they shouldn't make any problems for evaluating the data set.

**This set of filter consists of these filters:**

* [Island Filter](IslandFilter.md) - It's a filter that cuts off the lonely islands of data, those are probably fake.
* [Near Filter](NearFilter.md) - The close points appear to be closer than they really are, so this filter fixes that.
* [Peak Filter](PeakFilter.md) - (not used by default) It's a simple noise blanker which flatterns sharp peaks. It's limited to three points, so it can't make any harm for the real environment.
* [Robot Self Filter](RobotSelfFilter.md) - This filter cuts off the robot's body from the data set, because the robot can also see itself.
* [Self Edge Laser Filter](SelfEdgeLaserFilter.md) - The robot's own body also casts shadows, so this filter removes them.
* [Tradr Laser Shadow Filter](TradrLaserShadowFilter.md) - This is a simple filter that removes laser shadows, but doesn't work well for close shadows.
