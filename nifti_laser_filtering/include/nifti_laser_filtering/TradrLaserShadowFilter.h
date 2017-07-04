#ifndef _NIFTI_LASER_FILTERING_TRADRLASERSHADOWFILTER_H_
#define _NIFTI_LASER_FILTERING_TRADRLASERSHADOWFILTER_H_

#include <filters/filter_base.h>
#include <sensor_msgs/LaserScan.h>

namespace nifti_laser_filtering {

    /**
     * \brief A filter to do laser shadow filtering.
     *
     * The shadow filtering integrated into ROS doesn't give the results we need, so we use this custom one.
     * The computation is almost the same, so maybe the detected shadow handling differs.
     */
    class TradrLaserShadowFilter : public filters::FilterBase<sensor_msgs::LaserScan> {
    public:
        virtual ~TradrLaserShadowFilter();

        //! Read config parameters loaded by FilterBase::configure(string, NodeHandle)
        /**
         * Accepted parameters are:
         * min_angle: the filter applies to points with angle lower than min_angle or larger than pi-min_angle
         *      default 0.14 rad
         *      let r0 be the endpoint of the first ray, r1 the endpoint of the consequent ray, and l the position
         *      of laser; then the angle r0-r1-l is the constrained one
         */
        bool configure();

        //! Apply the filter.
        bool update(const sensor_msgs::LaserScan& input_scan, sensor_msgs::LaserScan& filtered_scan);

    protected:
        //! Tangens of (pi/2 - min_angle) to do shadow filtering of close points.
        double tan_shadow_filter_min_angle_close;
        //! Tangens of (pi/2 - min_angle) to do shadow filtering of far points.
        double tan_shadow_filter_min_angle_far;
        //! Distance (in meters) from which the points should be considered far.
        double far_distance_threshold;
    };
}

#endif //_NIFTI_LASER_FILTERING_TRADRLASERSHADOWFILTER_H_