#include "nifti_laser_filtering/NearFilter.h"

#include "pluginlib/class_list_macros.h"

namespace nifti_laser_filtering {

    NearFilter::~NearFilter() {
    }

    bool NearFilter::configure() {
        ROS_INFO("NearFilter: Successfully configured.");
        return true;
    }

    bool NearFilter::update(const sensor_msgs::LaserScan &input_scan, sensor_msgs::LaserScan &filtered_scan) {
        // experimentally determined constants
        const double max = 0.4;
        const double p00 =    -0.05064;
        const double p10 =       -3.36;
        const double p01 =    0.006448;
        const double p20 =        4.02;
        const double p11 =     0.02588;
        const double p02 =   -5.26e-05;
        const double p30 =      -19.92;
        const double p21 =     0.01529;
        const double p12 =  -9.746e-05;
        const double p03 =   1.652e-07;
        const double p40 =       10.35;
        const double p31 =     0.04989;
        const double p22 =  -8.034e-05;
        const double p13 =   1.735e-07;
        const double p04 =  -2.294e-10;
        const double p50 =      -7.719;
        const double p41 =    0.007997;
        const double p32 =  -5.798e-05;
        const double p23 =   8.971e-08;
        const double p14 =  -1.175e-10;
        const double p05 =   1.189e-13;

        double c, x, x2, x3, x4, y, y2, y3, y4;

        filtered_scan = input_scan;
        for (unsigned int i = 0; i < input_scan.ranges.size(); i++) {
            if (input_scan.ranges[i] > input_scan.range_min && input_scan.ranges[i] <= max) {
                x = filtered_scan.ranges[i];
                y = filtered_scan.intensities[i];
                x2 = x*x;
                x3 = x2*x;
                x4 = x3*x;
                y2 = y*y;
                y3 = y2*y;
                y4 = y3*y;
                // The poly55 function
                c = p00 + p10*x + p01*y + p20*x2 + p11*x*y + p02*y2 + p30*x3
                    + p21*x2*y + p12*x*y2 + p03*y3 + p40*x4 + p31*x3*y
                    + p22*x2*y2 + p13*x*y3 + p04*y4 + p50*x4*x + p41*x4*y
                    + p32*x3*y2 + p23*x2*y3 + p14*x*y4 + p05*y4*y;

                filtered_scan.ranges[i] += c;
            }
        }

        return true;
    }
}

PLUGINLIB_EXPORT_CLASS(nifti_laser_filtering::NearFilter, filters::FilterBase<sensor_msgs::LaserScan>);
