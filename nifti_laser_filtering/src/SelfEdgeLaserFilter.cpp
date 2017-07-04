#include <math.h>

#include "nifti_laser_filtering/SelfEdgeLaserFilter.h"

#include "pluginlib/class_list_macros.h"

namespace nifti_laser_filtering {

    SelfEdgeLaserFilter::~SelfEdgeLaserFilter() {
    }

    bool SelfEdgeLaserFilter::configure() {
        ROS_INFO("NearFilter: Successfully configured.");
        return true;
    }

    bool SelfEdgeLaserFilter::update(const sensor_msgs::LaserScan &input_scan, sensor_msgs::LaserScan &filtered_scan) {
        const float max_edge_dist = 0.5;
        const float cos_min = -0.98;
        const float cos_max = 0.10;
        const float delta_treshold = 0.05;

        const float cos_gamma = cos(input_scan.angle_increment);
        const float cos_2gamma = cos(2 * input_scan.angle_increment);

        float a1, a2, a3, r1, r2, r3, ra, rb, dr, dr2, cos_edge;
        int num_filtered_points = 0;
        int sgn;

        filtered_scan = input_scan;
        for (unsigned int i = 0; i < input_scan.ranges.size(); i++) {
            r1 = r2;
            r2 = r3;
            r3 = filtered_scan.ranges[i];
            if (i < 2) { //get at least three points
              continue;
            }

            if ((r1 != r1) || (r2 != r2) || (r3 != r3)) { //some points already filtered
              continue;
            }

            if (r2 > max_edge_dist) { //edge too far
              continue;
            }

            if (r2 > r1 || r2 > r3) { //edge not convex
              continue;
            }

            a1 = r1*r1 + r2*r2 - 2*r1*r2*cos_gamma;
            a2 = r2*r2 + r3*r3 - 2*r1*r2*cos_gamma;
            a3 = r1*r1 + r3*r3 - 2*r1*r2*cos_gamma;

            cos_edge = (a3 - a1 - a2) / (2 * sqrt(a1*a2));

            if ((cos_edge < cos_max) || (cos_edge > cos_min)) {
              //Let's remove the trail.
              ra = r2;
              if (a1 < a3) { //decide which way the trail continues
                sgn = -1;
                i -= 2;
              } else {
                sgn = 1;
              }

              while (i >= 0 && i < input_scan.ranges.size()) {
                rb = ra;
                ra = filtered_scan.ranges[i];
                dr2 = rb - ra;
                if (fabs(dr2) < delta_treshold) {
                  dr = dr2;
                  filtered_scan.ranges[i] = std::numeric_limits<float>::quiet_NaN();
                  num_filtered_points += 1;
                } else {
                  break;
                }
                i += sgn;
              }
              //filtered_scan.ranges[i] = std::numeric_limits<float>::quiet_NaN();
              //num_filtered_points += 1;
            }
        }

        ROS_DEBUG("Self edge shadow filtering removed %u points.", num_filtered_points);

        return true;
    }
}

PLUGINLIB_EXPORT_CLASS(nifti_laser_filtering::SelfEdgeLaserFilter, filters::FilterBase<sensor_msgs::LaserScan>);

