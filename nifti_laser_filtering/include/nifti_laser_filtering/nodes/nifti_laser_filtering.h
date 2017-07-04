#ifndef NIFTI_LASER_FILTERING_NIFTI_LASER_FILTERING_H
#define NIFTI_LASER_FILTERING_NIFTI_LASER_FILTERING_H

#include <ros/ros.h>
#include <sensor_msgs/LaserScan.h>
#include <filters/filter_chain.h>

using namespace std;
using namespace boost;

namespace nifti_laser_filtering {
    /*
     * \brief Laser filtering's task is to alter the raw laser scans to increase their usability.
     * As of now, the filtering does the following tasks:
     * - shift scan timestamp a little bit back (to account for processing delays)
     * - undistort near points (the laser is normally good only for points >20 cm far)
     * - discard all points detected on the robot's body
     * - discard all points classified as laser shadow (when the laser hit a sharp edge)
     *
     * Configuration of the filters is stored in nifti_laser_filtering/config/nifti_laser_filters.yaml.
     * The filters are loaded as pluginlib plugins.
     *
     * To have the robot body filtering working, the robot_description parameter must contain
     * an up-to-date URDF model of the robot with all additional parts (e.g. the arm), and proper
     * TFs have to be available.
     */
    class NiftiLaserFiltering {

    public:
        //! Read ROS parameters, initialize publishers/subscribers, initialize other class members. ROS::init() is assumed to have been called before.
        NiftiLaserFiltering();

        virtual ~NiftiLaserFiltering();

    protected:

        //! public NodeHandle
        ros::NodeHandle nodeHandle;

        //! private NodeHandle
        ros::NodeHandle privateNodeHandle;

        //! Time offset to add to scan timestamp (ROS parameter) (default: -0.025 sec)
        ros::Duration time_offset;

        //! Subscriber to laser scans (default topic: "/scan")
        ros::Subscriber laser_scan_subscriber;

        //! The chain of laser filters to apply to the incoming scans.
        filters::FilterChain<sensor_msgs::LaserScan> laser_filter_chain;

        //! Publisher for the filtered scans (default topic: "/scan_filtered")
        ros::Publisher scan_filtered_publisher;

        /** \brief Time correction of the laser scan.
         *
         * The idea is that it takes some time for the scan data to bubble through all the
         * firmware/drivers, so when ROS constructs the scan message, the time has gone forward
         * a bit in the meantime.
         * We need the most precise time of scan acquisition to take laser rotation into account.
         */
        void time_correct(sensor_msgs::LaserScan &scan);

        //! Scan callback function.
        void scan_callback(const sensor_msgs::LaserScan::ConstPtr &scan);
    };
}

#endif //NIFTI_LASER_FILTERING_NIFTI_LASER_FILTERING_H
