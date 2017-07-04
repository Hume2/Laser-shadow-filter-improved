#ifndef SELFEDGELASERFILTER_H
#define SELFEDGELASERFILTER_H

#include <filters/filter_base.h>
#include <sensor_msgs/LaserScan.h>

namespace nifti_laser_filtering {

    /**
     */
    class SelfEdgeLaserFilter : public filters::FilterBase<sensor_msgs::LaserScan> {
    public:
        virtual ~SelfEdgeLaserFilter();

        //! Read config parameters loaded by FilterBase::configure(string, NodeHandle)
        bool configure();

        //! Apply the filter.
        bool update(const sensor_msgs::LaserScan& input_scan, sensor_msgs::LaserScan& filtered_scan);
    };

}

#endif // SELFEDGELASERFILTER_H

