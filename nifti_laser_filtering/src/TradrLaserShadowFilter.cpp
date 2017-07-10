#include "nifti_laser_filtering/TradrLaserShadowFilter.h"

#include "pluginlib/class_list_macros.h"

namespace nifti_laser_filtering {

    TradrLaserShadowFilter::~TradrLaserShadowFilter() {
    }

    bool TradrLaserShadowFilter::configure() {
        double min_angle_close;
        if (!filters::FilterBase<sensor_msgs::LaserScan>::getParam("min_angle_close", min_angle_close)) {
            ROS_WARN("TradrLaserShadowFilter was not given min_angle_close, assuming 0.14 rad.");
            min_angle_close = 0.14;
        } else {
            ROS_DEBUG("TradrLaserShadowFilter: found param min_angle_close: %.5f rad", min_angle_close);
        }
        this->tan_shadow_filter_min_angle_close = tan(M_PI / 2 - min_angle_close);

        double min_angle_far;
        if (!filters::FilterBase<sensor_msgs::LaserScan>::getParam("min_angle_far", min_angle_far)) {
            ROS_WARN("TradrLaserShadowFilter was not given min_angle_far, assuming 0.5 rad.");
            min_angle_far = 0.5;
        } else {
            ROS_DEBUG("TradrLaserShadowFilter: found param min_angle_far: %.5f rad", min_angle_far);
        }
        this->tan_shadow_filter_min_angle_far = tan(M_PI / 2 - min_angle_far);

        if (!filters::FilterBase<sensor_msgs::LaserScan>::getParam("far_distance_threshold", far_distance_threshold)) {
            ROS_WARN("TradrLaserShadowFilter was not given far_distance_threshold, assuming 0.7 meters.");
            far_distance_threshold = 0.7;
        } else {
            ROS_DEBUG("TradrLaserShadowFilter: found param far_distance_threshold: %.5f rad", far_distance_threshold);
        }

        ROS_INFO("TradrLaserShadowFilter: Successfully configured.");
        return true;
    }

    void TradrLaserShadowFilter::proccess_point(int i, float r0, float r1,
                                                const sensor_msgs::LaserScan& input_scan,
                                                sensor_msgs::LaserScan& filtered_scan,
                                                unsigned int& num_filtered_points) {
        const float sin_gamma = sin(input_scan.angle_increment);
        const float cos_gamma = cos(input_scan.angle_increment);
        float x, y;
        const float r_min = input_scan.range_min;
        const float r_max = input_scan.range_max;

        if ((r1 >= r_min) && (r1 <= r_max)) {
            x = r0 * sin_gamma;
            y = fabs(r1 - r0 * cos_gamma);

            // the min_angle_close is used for points that are close to the robot and to its left/right (not in front of it).
            const double min_angle = ((i < 120 || i > 420) && r1 <= this->far_distance_threshold) ?
                                     this->tan_shadow_filter_min_angle_close :
                                     this->tan_shadow_filter_min_angle_far;
            if (y > x * min_angle) {
                filtered_scan.ranges[i - 1] = std::numeric_limits<float>::quiet_NaN();
                filtered_scan.ranges[i] = std::numeric_limits<float>::quiet_NaN();
                num_filtered_points += 1;
            }
            //r0 = r1;
        }
    }

    bool TradrLaserShadowFilter::update(const sensor_msgs::LaserScan &input_scan, sensor_msgs::LaserScan &filtered_scan) {
        //const float sin_gamma = sin(input_scan.angle_increment);
        //const float cos_gamma = cos(input_scan.angle_increment);
        float r0, r1;
        //const float r_min = input_scan.range_min;
        //const float r_max = input_scan.range_max;

        unsigned int num_filtered_points = 0;

        // go through the scan from one side to the other, and when two neighboring points form a very obtuse
        // triangle together with the sensor origin, discard the points
        filtered_scan = input_scan;
        r0 = input_scan.ranges[0];
        for (unsigned int i = 1; i < input_scan.ranges.size(); i++) {
            r1 = input_scan.ranges[i];
            proccess_point(i, r0, r1, input_scan, filtered_scan, num_filtered_points);
            r0 = r1;
        }

        r0 = input_scan.ranges[input_scan.ranges.size() - 1];
        for (unsigned int i = input_scan.ranges.size() - 2; i >= 1; i--) {
            r1 = input_scan.ranges[i];
            proccess_point(i, r0, r1, input_scan, filtered_scan, num_filtered_points);
            r0 = r1;
        }

       ROS_DEBUG("Shadow filtering removed %u points.", num_filtered_points);

        /*
         * a (alpha) is the angle of interest
         * tan (90-a) = y/x = (r1-r0*cos g)/(r0*sin g)
         *
         *      _/|
         *    _/ a|y=r1-r0*cos g
         *   /    |
         *  /90-a |
         * <------|
         *  \  x  |  x=r0*sin g
         * r0\    |
         *    \   |r1
         *     \  |
         *      \g|
         *       \|
         *     laser
         */

        return true;
    }
}

PLUGINLIB_EXPORT_CLASS(nifti_laser_filtering::TradrLaserShadowFilter, filters::FilterBase<sensor_msgs::LaserScan>);
