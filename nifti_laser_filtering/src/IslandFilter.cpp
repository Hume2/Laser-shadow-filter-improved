#include <math.h>

#include "nifti_laser_filtering/IslandFilter.h"

#include "pluginlib/class_list_macros.h"

namespace nifti_laser_filtering {

IslandFilter::~IslandFilter() {
}

bool IslandFilter::configure() {
  if (!filters::FilterBase<sensor_msgs::LaserScan>::getParam("max_distance", max_distance)) {
    ROS_WARN("IslandFilter was not given max_distance, assuming 0.4 m.");
    max_distance = 0.4;
  } else {
    ROS_DEBUG("IslandFilter: found param max_distance: %.5f m", max_distance);
  }

  if (!filters::FilterBase<sensor_msgs::LaserScan>::getParam("max_count", max_count)) {
    ROS_WARN("IslandFilter was not given max_count, assuming 10.");
    max_count = 10;
  } else {
    ROS_DEBUG("IslandFilter: found param max_count: %i", max_count);
  }

  ROS_INFO("IslandFilter: Successfully configured.");
  return true;
}

bool IslandFilter::proccess_point(sensor_msgs::LaserScan& filtered_scan, bool& last_valid,
                    int& island_points, int& num_filtered_points, int i, int sgn) {
  bool result = false;
  double r = filtered_scan.ranges[i];

  if (r == r && r < max_distance) {
    island_points++;
    //summa += r;
  } else {
    if (last_valid) {
      //average = summa / island_points;
      if (island_points < max_count) {
        //remove the island
        for (unsigned int j = i - island_points*sgn; j != i; j += sgn) {
          filtered_scan.ranges[j] = std::numeric_limits<float>::quiet_NaN();
          num_filtered_points += 1;
        }
        result = true;
        //i -= island_points;
      }
    }
    //summa = 0;
    island_points = 0;
  }
  last_valid = r == r && r < max_distance;
  return result;
}

bool IslandFilter::update(const sensor_msgs::LaserScan& input_scan, sensor_msgs::LaserScan& filtered_scan) {
  int num_filtered_points = 0;
  filtered_scan = input_scan;

  int island_points = 0;
  //double average;
 // double summa = 0;
  bool last_valid = false;

  for (unsigned int i = 0; i < filtered_scan.ranges.size(); i++) {
    if (proccess_point(filtered_scan, last_valid, island_points, num_filtered_points, i, 1)) {
      //break;
    }
  }

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


