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
                        int& island_points, int& num_filtered_points, int i, int sgn);

  protected:
    //! The maximum distance of island to be filtered.
    double max_distance;

    //! The maximum count of points per filtered island.
    int max_count;

    //! The maximum count of points detected before a big island to filter it.
    int max_big_rise;

    //! The count of frames to compare
    int buffer_depth;

    //! The laser data from previous two scans
    static bool** buffer;

    //! The current buffer to be written to
    static int current_buffer;

    //! The width of one buffer
    static int buffer_width;
};

}

#endif // ISLANDFILTER_H

