#ifndef ISLANDFILTER_H
#define ISLANDFILTER_H

#include <filters/filter_base.h>
#include <sensor_msgs/LaserScan.h>

namespace nifti_laser_filtering {

/**
     */
class IslandFilter : public filters::FilterBase<sensor_msgs::LaserScan> {
  public:
    virtual ~IslandFilter();

    //! Read config parameters loaded by FilterBase::configure(string, NodeHandle)
    bool configure();

    //! Apply the filter.
    bool update(const sensor_msgs::LaserScan& input_scan, sensor_msgs::LaserScan& filtered_scan);

    //! Proccess one point
    bool proccess_point(sensor_msgs::LaserScan& filtered_scan, bool& last_valid, int& delete_big,
                        double& last_dist, int& island_points, int& num_filtered_points, int i, int sgn);

  protected:
    //! The maximum distance of island to be filtered.
    double max_distance;

    //! The maximum count of points per filtered island.
    int max_count;

    //! The maximum count of points detected before a big island to filter it.
    int max_big_rise;

    //! The angle where the skip cone starts. The skip sone is a cone on the front
    //! of the robot where no islands are going to be filtered.
    double skip_cone;

    //! The minimum distance between island and a background wall
    double min_wall_distance;

    //! The laser data from previous two scans
    static bool* buffer;
};

}

#endif // ISLANDFILTER_H

