#ifndef _NIFTI_LASER_FILTERING_NEARFILTER_H_
#define _NIFTI_LASER_FILTERING_NEARFILTER_H_

#include <filters/filter_base.h>
#include <sensor_msgs/LaserScan.h>

namespace nifti_laser_filtering {

    /**
     */
    class NearFilter : public filters::FilterBase<sensor_msgs::LaserScan> {
    public:
        virtual ~NearFilter();

        //! Read config parameters loaded by FilterBase::configure(string, NodeHandle)
        bool configure();

        //! Apply the filter.
        bool update(const sensor_msgs::LaserScan& input_scan, sensor_msgs::LaserScan& filtered_scan);
    };

}


#endif //_NIFTI_LASER_FILTERING_NEARFILTER_H_
