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
    bool proccess_point(sensor_msgs::LaserScan& filtered_scan, bool& last_valid,
                        int& island_points, int& num_filtered_points, int i, int sgn);

  protected:
    //! The maximum distance of island to be filtered.
    double max_distance;

    //! The maximum count of points per filtered island.
    int max_count;
};

}

#endif // ISLANDFILTER_H

