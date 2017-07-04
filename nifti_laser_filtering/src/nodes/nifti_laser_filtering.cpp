#include "nifti_laser_filtering/nodes/nifti_laser_filtering.h"

using namespace std;
using namespace boost;
using namespace sensor_msgs;

template<typename T>
T getParam(ros::NodeHandle &n, const string &name, const T &defaultValue) {
	T v;
	if (n.getParam(name, v)) {
		ROS_INFO_STREAM("Found parameter: " << name << ", value: " << v);
		return v;
	}
	else
		ROS_WARN_STREAM("Cannot find value for parameter: " << name << ", assigning default: " << defaultValue);
	return defaultValue;
}

namespace nifti_laser_filtering {

	NiftiLaserFiltering::NiftiLaserFiltering() :
			privateNodeHandle("~"),
			laser_filter_chain("sensor_msgs::LaserScan") {

		time_offset = ros::Duration(getParam<double>(privateNodeHandle, "time_offset", -0.025));

		// laser scan subscriber
		laser_scan_subscriber = nodeHandle.subscribe<LaserScan>(
				"scan", 250, bind(&NiftiLaserFiltering::scan_callback, this, _1));

		// setup laser scan filters - should have been loaded from
		// config/nifti_laser_filters.yaml in launch file
		laser_filter_chain.configure("nifti_laser_filtering/scan_filter_chain", nodeHandle);

		// filtered scan publisher
		scan_filtered_publisher = nodeHandle.advertise<LaserScan>("scan_filtered", 50);
	}

	NiftiLaserFiltering::~NiftiLaserFiltering() {
	}

	void NiftiLaserFiltering::time_correct(LaserScan &scan) {
		// add time_offset to the scan's timestamp
		// (actually, the value is supposed to be negative, so we're subtracting)

		scan.header.stamp = scan.header.stamp + time_offset;
	}

	void NiftiLaserFiltering::scan_callback(const LaserScan::ConstPtr &scan) {
		ROS_DEBUG("A new laser scan is going to be processed.");

		// create a non-const copy of the scan
		LaserScan scan_time_corrected = LaserScan(*scan);
		// correct the message time
		time_correct(scan_time_corrected);

		// apply the filter chain
		LaserScan scan_filtered;

		if (laser_filter_chain.update(scan_time_corrected, scan_filtered)) {

			ROS_DEBUG("Laser filtering chain has been applied.");

			// publish the filtered result
			scan_filtered_publisher.publish(scan_filtered);

			ROS_DEBUG("Filtered scan has been published.");

		} else {
			ROS_INFO_THROTTLE(3, "Filtering the scan from time %i.%i failed.", scan_time_corrected.header.stamp.sec, scan_time_corrected.header.stamp.nsec);
		}
	}

}

int main(int argc, char **argv)
{
	ros::init(argc, argv, "nifti_laser_filtering");

	nifti_laser_filtering::NiftiLaserFiltering nlf;

	ros::spin();

	return 0;
}
