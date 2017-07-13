#include <math.h>

#include "nifti_laser_filtering/IslandFilter.h"

#include "pluginlib/class_list_macros.h"

namespace nifti_laser_filtering {

bool* IslandFilter::buffer = NULL;

IslandFilter::~IslandFilter() {
  if (buffer) {
    delete[] buffer;
  }
}

bool IslandFilter::configure() {
  if (!filters::FilterBase<sensor_msgs::LaserScan>::getParam("max_distance", max_distance)) {
    ROS_WARN("IslandFilter was not given max_distance, assuming 0.4 m.");
    max_distance = 0.4;
  } else {
    ROS_DEBUG("IslandFilter: found param max_distance: %.5f m", max_distance);
  }

  if (!filters::FilterBase<sensor_msgs::LaserScan>::getParam("min_wall_distance", min_wall_distance)) {
    ROS_WARN("IslandFilter was not given min_wall_distance, assuming 0.05 m.");
    min_wall_distance = 0.05;
  } else {
    ROS_DEBUG("IslandFilter: found param min_wall_distance: %.5f m", min_wall_distance);
  }

  if (!filters::FilterBase<sensor_msgs::LaserScan>::getParam("skip_cone", skip_cone)) {
    ROS_WARN("IslandFilter was not given skip_cone, assuming 1.571 rad.");
    skip_cone = 1.571;
  } else {
    ROS_DEBUG("IslandFilter: found param skip_cone: %.5f rad", skip_cone);
  }

  if (!filters::FilterBase<sensor_msgs::LaserScan>::getParam("max_count", max_count)) {
    ROS_WARN("IslandFilter was not given max_count, assuming 10.");
    max_count = 10;
  } else {
    ROS_DEBUG("IslandFilter: found param max_count: %i", max_count);
  }

  if (!filters::FilterBase<sensor_msgs::LaserScan>::getParam("max_big_rise", max_big_rise)) {
    ROS_WARN("IslandFilter was not given max_big_rise, assuming 10.");
    max_big_rise = 10;
  } else {
    ROS_DEBUG("IslandFilter: found param max_big_rise: %i", max_big_rise);
  }

  ROS_INFO("IslandFilter: Successfully configured.");
  return true;
}

bool IslandFilter::proccess_point(sensor_msgs::LaserScan& filtered_scan, bool& last_valid, int& delete_big,
                    double& last_dist, int& island_points, int& num_filtered_points, int i, int sgn) {
  bool result = false;
  double r = filtered_scan.ranges[i];

  //detect the island even when there is a big jummp
  if (last_dist == last_dist) {
    double dr = r - last_dist;
    if (dr >= min_wall_distance) {
      last_valid = false;
    } else if (dr <= -min_wall_distance) {
      r = max_distance + 1.0;
    }
  }

  if (r == r && r < max_distance) {
    //we are on an island, so let's count the points
    island_points++;
    if (buffer[i]) {
      delete_big++;
    }
  } else {
    if (last_valid) {
      // The island ends right there, so let's decide whether it should be kept or removed.
      if (island_points < max_count || delete_big < max_big_rise) {
        //remove the island
        for (unsigned int j = i - island_points*sgn; j != i; j += sgn) {
          filtered_scan.ranges[j] = std::numeric_limits<float>::quiet_NaN();
          num_filtered_points += 1;
        }
        result = true;
      }
    }
    // The variables should be reinitialised for the next island.
    island_points = 0;
    delete_big = 0;
  }

  // Write the current data into the buffers.
  last_valid = r == r && r < max_distance;
  buffer[i] = last_valid;
  last_dist = r;
  return result;
}

bool IslandFilter::update(const sensor_msgs::LaserScan& input_scan, sensor_msgs::LaserScan& filtered_scan)
{
  if (!buffer) { // Initialise the buffer
    buffer = new bool[input_scan.ranges.size()];
    for (unsigned int i = 0; i < filtered_scan.ranges.size(); i++) {
      buffer[i] = false;
    }
  }

  int num_filtered_points = 0;
  filtered_scan = input_scan;

  int island_points = 0;
  bool last_valid = false;
  double last_dist = 0;
  int delete_big = 0; //count how many points were at the place of island in the previous frame

  int skip_min = skip_cone / input_scan.angle_increment;
  int skip_max = filtered_scan.ranges.size() - skip_min;

  for (unsigned int i = 0; i < filtered_scan.ranges.size(); i++) {
    if (i > skip_min && i < skip_max) {
      continue;
    }
    if (proccess_point(filtered_scan, last_valid, delete_big, last_dist,
                       island_points, num_filtered_points, i, 1)) {
      //The count of filtered islands might be limited in the future.
    }
  }

  // This is ready for the backward pass
  /*for (unsigned int i = filtered_scan.ranges.size()-1; i >= 0; i--) {
    if (proccess_point(filtered_scan, last_valid, island_points, num_filtered_points, i, -1)) {
      break;
    }
  }*/

  ROS_DEBUG("Island filter filtered %u points.", num_filtered_points);

  return true;
}

}

PLUGINLIB_EXPORT_CLASS(nifti_laser_filtering::IslandFilter, filters::FilterBase<sensor_msgs::LaserScan>);



