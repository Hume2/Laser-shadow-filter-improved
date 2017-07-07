#include <math.h>

#include "nifti_laser_filtering/PeakFilter.h"

#include "pluginlib/class_list_macros.h"

namespace nifti_laser_filtering {

PeakFilter::~PeakFilter() {
}

bool PeakFilter::configure() {
  double max_filter_angle;
  if (!filters::FilterBase<sensor_msgs::LaserScan>::getParam("max_filter_angle", max_filter_angle)) {
    ROS_WARN("PeakFilter was not given max_filter_angle, assuming 0.524 rad.");
    max_filter_angle = 0.524;
  } else {
    ROS_DEBUG("PeakFilter: found param max_filter_angle: %.5f rad", max_filter_angle);
  }
  cos_min = cos(max_filter_angle);

  if (!filters::FilterBase<sensor_msgs::LaserScan>::getParam("passes", passes)) {
    ROS_WARN("PeakFilter was not given passes, assuming 5.");
    passes = 5;
  } else {
    ROS_DEBUG("PeakFilter: found param passes: %.5f rad", max_filter_angle);
  }

  ROS_INFO("PeakFilter: Successfully configured.");
  return true;
}

void PeakFilter::make_pass(sensor_msgs::LaserScan& filtered_scan, int& num_filtered_points) {
  const double cos_gamma = cos(filtered_scan.angle_increment);
  const double cos_2gamma = cos(2 * filtered_scan.angle_increment);

  double a1, a2, a3, r1, r2, r3, cos_edge;

  for (unsigned int i = 0; i < filtered_scan.ranges.size(); i++) {
    r1 = r2;
    r2 = r3;
    r3 = filtered_scan.ranges[i];
    if (i < 2) { //get at least three points
      continue;
    }

    if ((r1 != r1) || (r2 != r2) || (r3 != r3)) { //some points already filtered
      continue;
    }

    a1 = r1*r1 + r2*r2 - 2*r1*r2*cos_gamma;
    a2 = r2*r2 + r3*r3 - 2*r1*r2*cos_gamma;
    a3 = r1*r1 + r3*r3 - 2*r1*r2*cos_2gamma;

    cos_edge = (a3 - a1 - a2) / (2 * sqrt(a1*a2));

    if (cos_edge > cos_min) { //filter the peak
      r2 = (r1 + r3)/2;
      filtered_scan.ranges[i] = r2;
      num_filtered_points++;
    }
  }
}

bool PeakFilter::update(const sensor_msgs::LaserScan& input_scan, sensor_msgs::LaserScan& filtered_scan) {
  int num_filtered_points = 0;
  filtered_scan = input_scan;

  for (int i = passes; i; i--) {
    make_pass(filtered_scan, num_filtered_points);
  }

  ROS_DEBUG("Peak filter corrected %u peaks.", num_filtered_points);

  return true;
}

/*              S
 *             /|\
 *            /g|g\
 *           /  |  \
 *          /   |   \
 *       r1/  r2|    \r3
 *        /     |     \
 *       /      0      \
 *      /     _/ \_     \
 *     /    _/\ ? /\_    \
 *    /   _/   "-"   \_   \
 *   /  _/ a1       a2 \_  \
 *  / _/        |        \_ \
 * /_/__________1__________\_\
 *             a3
 *
 *              S
 *             /|\
 *            /g|g\
 *           /  |  \
 *          /   |   \
 *       r1/  r2|    \r3
 *        /     |     \
 *       /      |      \
 *      /       |       \
 *     /        |        \
 *    /         |         \
 *   /          |          \
 *  /           |           \
 * /____________1____________\
 * \_           |   a3      _/
 *   \_         |         _/
 *     \_       |       _/
 *    a1 \_    ___    _/ a2
 *         \__/   \__/
 *           \_ ? _/
 *             \_/
 *              0
 *
 * When the angle marked as '?' is too sharp, it's assumed to be a peak.
 * After that it moves the point marked '0' to point marked '1'.*/
}

PLUGINLIB_EXPORT_CLASS(nifti_laser_filtering::PeakFilter, filters::FilterBase<sensor_msgs::LaserScan>);


