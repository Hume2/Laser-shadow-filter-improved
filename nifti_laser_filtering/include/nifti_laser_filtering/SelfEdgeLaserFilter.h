#ifndef SELFEDGELASERFILTER_H
#define SELFEDGELASERFILTER_H

#include <filters/filter_base.h>
#include <sensor_msgs/LaserScan.h>

namespace nifti_laser_filtering {

struct LaserData {
    double r1, r2, dr2;
    int c;
};

/**
 * \brief filter to remove the shadows of robot's own body
 *
 * \author Leonard Mentzl
 */
class SelfEdgeLaserFilter : public filters::FilterBase<sensor_msgs::LaserScan> {
  public:
    virtual ~SelfEdgeLaserFilter();

    //! Read config parameters loaded by FilterBase::configure(string, NodeHandle)
    bool configure();

    //! Apply the filter.
    bool update(const sensor_msgs::LaserScan& input_scan, sensor_msgs::LaserScan& filtered_scan);

  protected:
    //! The maximum distance of robot's edge
    double max_edge_dist;

    //! The cosine of maximum angle at the trail's start
    double cos_min;

    //! The cosine of minimum angle at the trail's start
    double cos_max;

    //! The maximum distance of filtered points from the trail's line
    double delta_threshold;

    //! The laser data from previous two scans
    static LaserData* buffer;

    //! Cut all that is closer than this distance
    double min_valid_distance;

    //! How many points can be filtered after the trail
    int after_trail_points;
};

}

#endif // SELFEDGELASERFILTER_H

