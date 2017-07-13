#include <math.h>

#include "nifti_laser_filtering/SelfEdgeLaserFilter.h"

#include "pluginlib/class_list_macros.h"

namespace nifti_laser_filtering {

LaserData* SelfEdgeLaserFilter::buffer = NULL;

SelfEdgeLaserFilter::~SelfEdgeLaserFilter() {
  if (buffer) {
    delete[] buffer;
  }
}

bool SelfEdgeLaserFilter::configure() {
  if (!filters::FilterBase<sensor_msgs::LaserScan>::getParam("max_edge_distance", max_edge_dist)) {
    ROS_WARN("SelfEdgeLaserFilter was not given max_edge_distance, assuming 0.5 meters.");
    max_edge_dist = 0.5;
  } else {
    ROS_DEBUG("SelfEdgeLaserFilter: found param max_edge_distance: %.5f meters", max_edge_dist);
  }

  double min_filter_angle;
  if (!filters::FilterBase<sensor_msgs::LaserScan>::getParam("min_filter_angle", min_filter_angle)) {
    ROS_WARN("SelfEdgeLaserFilter was not given min_filter_angle, assuming 1.466 rad.");
    min_filter_angle = 1.466;
  } else {
    ROS_DEBUG("SelfEdgeLaserFilter: found param min_filter_angle: %.5f rad", min_filter_angle);
  }
  cos_max = cos(min_filter_angle);

  double max_filter_angle;
  if (!filters::FilterBase<sensor_msgs::LaserScan>::getParam("max_filter_angle", max_filter_angle)) {
    ROS_WARN("SelfEdgeLaserFilter was not given max_filter_angle, assuming 2.932 rad.");
    max_filter_angle = 2.932;
  } else {
    ROS_DEBUG("SelfEdgeLaserFilter: found param max_filter_angle: %.5f rad", max_filter_angle);
  }
  cos_min = cos(max_filter_angle);

  if (!filters::FilterBase<sensor_msgs::LaserScan>::getParam("delta_threshold", delta_threshold)) {
    ROS_WARN("SelfEdgeLaserFilter was not given delta_threshold, assuming 0.01 meters.");
    delta_threshold = 0.01;
  } else {
    ROS_DEBUG("SelfEdgeLaserFilter: found param delta_threshold: %.5f rad", delta_threshold);
  }

  if (!filters::FilterBase<sensor_msgs::LaserScan>::getParam("min_valid_distance", min_valid_distance)) {
    ROS_WARN("SelfEdgeLaserFilter was not given min_valid_distance, assuming 0.15 meters.");
    delta_threshold = 0.15;
  } else {
    ROS_DEBUG("SelfEdgeLaserFilter: found param min_valid_distance: %.5f rad", min_valid_distance);
  }

  if (!filters::FilterBase<sensor_msgs::LaserScan>::getParam("after_trail_points", after_trail_points)) {
    ROS_WARN("SelfEdgeLaserFilter was not given after_trail_points, assuming 5.");
    after_trail_points = 5;
  } else {
    ROS_DEBUG("SelfEdgeLaserFilter: found param after_trail_points: %i", after_trail_points);
  }

  if (!filters::FilterBase<sensor_msgs::LaserScan>::getParam("use_vertical_filtering", use_vertical_filtering)) {
    use_vertical_filtering = true;
    ROS_DEBUG("SelfEdgeLaserFilter: found param use_vertical_filtering: %i", use_vertical_filtering);
  } else {
    ROS_DEBUG("SelfEdgeLaserFilter: found param use_vertical_filtering: %i", use_vertical_filtering);
  }

  ROS_INFO("SelfEdgeLaserFilter: Successfully configured.");
  return true;
}

bool SelfEdgeLaserFilter::update(const sensor_msgs::LaserScan &input_scan, sensor_msgs::LaserScan &filtered_scan) {
  if (!buffer) {
    buffer = new LaserData[input_scan.ranges.size()];
    for (unsigned int i = 0; i < filtered_scan.ranges.size(); i++) {
      buffer[i].r1 = std::numeric_limits<float>::quiet_NaN();
      buffer[i].r2 = std::numeric_limits<float>::quiet_NaN();
      buffer[i].dr2 = std::numeric_limits<float>::quiet_NaN();
    }
  }

  const double cos_gamma = cos(input_scan.angle_increment);
  const double cos_2gamma = cos(2 * input_scan.angle_increment);

  int num_filtered_points = 0;

  filtered_scan = input_scan;

  double r1, r2, r3, a1, a2, a3, dr, cos_edge;

  // horizontal filtering
  double ra, rb, dr2;
  int sgn;

  for (unsigned int i = 0; i < filtered_scan.ranges.size(); i++) {
    r1 = r2;
    r2 = r3;
    r3 = filtered_scan.ranges[i];

    //check whether the value is really valid
    if (r3 < input_scan.range_min || r3 > input_scan.range_max || r3 < min_valid_distance) {
      filtered_scan.ranges[i] = std::numeric_limits<float>::quiet_NaN();
      r3 = std::numeric_limits<float>::quiet_NaN();
      num_filtered_points += 1;
    }

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
    a2 = r2*r2 + r3*r3 - 2*r2*r3*cos_gamma;
    a3 = r1*r1 + r3*r3 - 2*r1*r3*cos_2gamma;

    cos_edge = (a3 - a1 - a2) / (2 * sqrt(a1*a2));

    if ((cos_edge < cos_max) && (cos_edge > cos_min)) {
      //ROS_INFO("cos_max = %.5f, cos_edge = %.5f, cos_min = %.5f", cos_max, cos_edge, cos_min);
      int j = i;
      int c = after_trail_points;
      //Let's remove the trail.
      ra = r2;
      if (r1 > r3) { //decide which way the trail continues
        sgn = -1;
        j -= 2;
      } else {
        sgn = 1;
      }

      while (j >= 0 && j < filtered_scan.ranges.size() && c >= 0) {
        rb = ra;
        ra = filtered_scan.ranges[j];
        dr2 = rb - ra;
        if (fabs(dr2 - dr) < delta_threshold) {
          dr = dr2;
        } else {
          c--;
        }

        filtered_scan.ranges[j] = std::numeric_limits<float>::quiet_NaN();
        num_filtered_points += 1;

        j += sgn;
      }
    }
  }

  // vertical filtering
  if (use_vertical_filtering) {
    for (unsigned int i = 0; i < filtered_scan.ranges.size(); i++) {
      r1 = buffer[i].r1;
      r2 = buffer[i].r2;
      r3 = filtered_scan.ranges[i];

      buffer[i].r1 = r2;
      buffer[i].r2 = r3;

      if (buffer[i].dr2 == buffer[i].dr2) {
        dr = r3 - r2;
        if (buffer[i].c >= 0) {
          if (fabs(buffer[i].dr2 - dr) < delta_threshold) {
            //buffer[i].dr2 = dr;
          } else {
            buffer[i].c--;
          }
          filtered_scan.ranges[i] = std::numeric_limits<float>::quiet_NaN();
          num_filtered_points += 1;
          continue;
        } else {
          buffer[i].dr2 = std::numeric_limits<float>::quiet_NaN();
        }
      }

      if ((r1 != r1) || (r3 != r3)) { //some points already filtered
        continue;
      }

      if (r2 > max_edge_dist) { //edge too far
        continue;
      }

      if (r2 > r1 || r2 > r3) { //edge not convex
        continue;
      }

      a1 = r1*r1 + r2*r2 - 2*r1*r2*cos_gamma;
      a2 = r2*r2 + r3*r3 - 2*r2*r3*cos_gamma;
      a3 = r1*r1 + r3*r3 - 2*r1*r3*cos_2gamma;

      cos_edge = (a3 - a1 - a2) / (2 * sqrt(a1*a2));

      if (r1 > r3) { //We have already missed the edge.
        continue;
      }

      if ((cos_edge < cos_max) && (cos_edge > cos_min)) {
        filtered_scan.ranges[i] = std::numeric_limits<float>::quiet_NaN();
        num_filtered_points += 1;
        buffer[i].dr2 = r3 - r2;
        buffer[i].c = after_trail_points;
      }
    }
  }


  ROS_DEBUG("Self edge shadow filtering removed %u points.", num_filtered_points);

  return true;
}

/*
 *                            _S
 *                         _-"//
 *                      _-" g/g|
 *                   _-"    / /
 *             r1 _-"      /  |
 *             _-"        /  /
 *          _-"        r2/   |
 *       _-"            /   /
 *    _-"              /    |
 * _-"     a1         /    /r3
 * ------------------+     |
 *  \             | ?|    /
 *   "\            \_|    |
 *     "\            |   /
 *       "\          |a2 |
 *         "\a3      |  /
 *           "\      |  |
 *             "\    | /
 *               "\  | |
 *                 "\|/
 *
 * If the angle marked '?' is in given bounds, the trail is detected.
 * After that it goes along the trail and removes the point while the
 * points are along the detected trail.
 */

}

PLUGINLIB_EXPORT_CLASS(nifti_laser_filtering::SelfEdgeLaserFilter, filters::FilterBase<sensor_msgs::LaserScan>);

