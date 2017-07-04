#ifndef _NIFTI_LASER_FILTERING_ROBOTSELFFILTER_H_
#define _NIFTI_LASER_FILTERING_ROBOTSELFFILTER_H_

#include <set>

#include <boost/scoped_ptr.hpp>
#include <boost/thread.hpp>

#include <Eigen/Core>
#include <Eigen/Geometry>

#include <pcl/filters/crop_box.h>

#include <ros/ros.h>
#include <filters/filter_base.h>
#include <sensor_msgs/LaserScan.h>
#include <moveit/point_containment_filter/shape_mask.h>
#include <moveit/occupancy_map_monitor/occupancy_map_updater.h>
#include <urdf/model.h>
#include <laser_geometry/laser_geometry.h>
#include <geometric_shapes/mesh_operations.h>
#include <geometry_msgs/Point32.h>
#include <geometry_msgs/PolygonStamped.h>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>
#include <dynamic_reconfigure/Config.h>
//#include <stamped_msgs/Float32.h>
#include <geometry_msgs/PointStamped.h>
#include <visualization_msgs/MarkerArray.h>

using namespace std;
using namespace boost;

namespace robot_self_filter {
    /**
	 * \brief Just a helper structure holding together a link, one of its collision elements,
     * and the index of the collision element in the collision array of the link.
     *
     * All of this is done to be able to use collision links as indices in a map.
	 */
    struct CollisionBodyWithLink {
        boost::shared_ptr<urdf::Collision> collision;
        boost::shared_ptr<urdf::Link> link;
        size_t index_in_collision_array;
        Eigen::Vector3d shape_extents;

        CollisionBodyWithLink() :
                index_in_collision_array(0) {
        }

        CollisionBodyWithLink(boost::shared_ptr<urdf::Collision> const &collision,
                boost::shared_ptr<urdf::Link> const &link,
                size_t index_in_collision_array,
                const Eigen::Vector3d& shape_extents):

                collision(collision), link(link), index_in_collision_array(index_in_collision_array),
                shape_extents(shape_extents)
        {
        }

        //! Return a unique cache key for the collision
        string getCacheKey() const {
            ostringstream stream;
            stream << link->name << "-" << index_in_collision_array;
            return stream.str();
        }
    };

    /**
     * \brief Watch if some unreachable TFs haven't become reachable.
     *
     * Intended to be run in a separate thread.
     *
     * \author Martin Pecka
     */
    class UnreachableFramesWatchdog {
    public:
        UnreachableFramesWatchdog(string robot_frame, set<string>& reachable_frames,
            map<point_containment_filter::ShapeHandle, CollisionBodyWithLink>& shapes_to_links,
            ros::Duration TF_BUFFER_LENGTH, boost::shared_ptr<boost::mutex> model_mutex);

        /** Start the updater using a thread.
         *
         * \param tf_wait_timeout Timeout for canTransform() (in seconds)
         */
        void start(double tf_wait_timeout);

        /** Run the updater (should be run in a separate thread).
         *
         * \param tf_wait_timeout Timeout for canTransform() (in seconds)
         */
        void run(double tf_wait_timeout);

        /**
         * \brief Pause thread execution.
         */
        void pause();

        /**
         * \brief Unpause thread execution.
         */
        void unpause();

        /**
         * \brief Clear shapes_to_links, reachable_frames and tf_buffer.
         */
        void clear();

    protected:
        //! The target frame of all watched transforms.
        string robot_frame;
        //! List of source frames for which TFs to robot_frame are available.
        set<string>& reachable_frames;
        /// A map that correlates shapes in robot_shape_mask to collision links in URDF.
        /** Used to know the set of all frames to be considered and watched. */
        map<point_containment_filter::ShapeHandle, CollisionBodyWithLink>& shapes_to_links;

        //! If true, this thread is paused.
        bool paused;

        //! tf client
        tf2_ros::Buffer tf_buffer;
        //! tf listener
        tf2_ros::TransformListener tf_listener;

        //! A mutex that has to be locked in order to work with reachable_frames, shapes_to_links or tf_buffer.
        boost::shared_ptr<boost::mutex> model_mutex;
    private:
        thread this_thread;
    };

    /**
     * \brief Axis-aligned bounding box.
     */
    class AABB {
    protected:
        //! Coordinates of the lowest corner.
        Eigen::Vector3d min;
        //! Coordinates of the highest corner.
        Eigen::Vector3d max;

    public:
        /**
         * \brief Coordinates of the lowest corner.
         * \return Coordinates of the lowest corner.
         */
        const Eigen::Vector3d& minCorner() const {return this->min;}

        /**
         * \brief Coordinates of the highest corner.
         * \return Coordinates of the highest corner.
         */
        const Eigen::Vector3d& maxCorner() const {return this->max;}

        /**
         * \brief Merge this AABB with a box given by extents and transformed by the given transform.
         *
         * The given box is assumed to have its center in the midpoint of the given extents.
         *
         * \param transform Transform of the box to merge with.
         * \param extents Dimensions of the box to merge with.
         */
        inline void update(const Eigen::Affine3d& transform, const Eigen::Vector3d& extents) {
            Eigen::Vector3d half = extents / 2.0;
            const Eigen::Vector3d max = transform * half;
            half = -half;
            const Eigen::Vector3d min = transform * half;

            // now we walk around all the box's 8 vertices
            Eigen::Vector3d corner = max;
            this->updateFromPoint(corner);
            corner[0] = min[0];
            this->updateFromPoint(corner);
            corner[1] = min[1];
            this->updateFromPoint(corner);
            corner[2] = min[2];  // corner == min
            this->updateFromPoint(corner);

            corner[1] = max[1];
            this->updateFromPoint(corner);
            corner[0] = max[0];
            this->updateFromPoint(corner);
            corner[1] = min[1];
            this->updateFromPoint(corner);
            corner[2] = max[2];
            this->updateFromPoint(corner);
        }

        /**
         * \brief Insert a point to the AABB and update it to contain that point.
         * \param point The point to insert.
         */
        inline void updateFromPoint(const Eigen::Vector3d& point) {
            if (this->min[0] > point[0])
                this->min[0] = point[0];
            if (this->min[1] > point[1])
                this->min[1] = point[1];
            if (this->min[2] > point[2])
                this->min[2] = point[2];

            if (this->max[0] < point[0])
                this->max[0] = point[0];
            if (this->max[1] < point[1])
                this->max[1] = point[1];
            if (this->max[2] < point[2])
                this->max[2] = point[2];
        }

        /**
         * \brief Reset the AABB to be prepared to accept new bounding boxes.
         */
        inline void reset() {
            this->min = Eigen::Vector3d::Ones() * 1000.0;
            this->max = Eigen::Vector3d::Ones() * (-1000.0);
        }
    };

    /**
     * \brief Filter to remove robot's own body from laser scans.
     *
     * \author Martin Pecka
     */
    class RobotSelfFilter : public filters::FilterBase<sensor_msgs::LaserScan> {
    public:
        RobotSelfFilter();
        virtual ~RobotSelfFilter();

        //! Read config parameters loaded by FilterBase::configure(string, NodeHandle)
        /**
         * robot_frame: The frame of the robot body
         *      default: base_link
         * min_distance: The minimum distance of points from the laser to keep them (in meters)
         *      default: 0.0 m
         * inflation_scale: A scale that is applied to the collision model for the purposes of collision checking
         *      default: 1.0
         * inflation_padding: Padding to be added to the collision model for the purposes of collision checking (meters)
         *      default: 0.0 m
         * robot_description_param: Name of the parameter where the robot model can be found
         *      default: /robot_description
         */
        bool configure();

        //! Apply the filter.
        bool update(const sensor_msgs::LaserScan &input_scan, sensor_msgs::LaserScan &filtered_scan);

    protected:

        //! Handle of the node this filter runs in.
        ros::NodeHandle nodeHandle;

        /// The frame of the robot body.
        /** Usually base_link. It has to be usable as a fixed frame wrt the laser frame.
         * Especially, it can't be laser_frame itself in case the laser is rotating. */
        string robot_frame;

        //! The minimum distance of points from the laser to keep them (in meters).
        double min_distance;

        //! The maximum distance of points from the body origin to apply this filter on.
        double max_distance;

        /// A scale that is applied to the collision model for the purposes of collision checking (1.0 = no scaling).
        /** Every collision element is scaled individually with the scaling center in its origin. */
        double inflation_scale;

        /// A constant padding to be added to the collision model for the purposes of collision checking (in meters).
        /** It is added individually to every collision element. */
        double inflation_padding;

        //! Name of the parameter where the robot model can be found.
        string robot_description_param;

        //! Subscriber for nifti_robot_description updates.
        ros::Subscriber robot_description_updates_listener;

        //! Publisher of robot radius (relative to robot_center).
        ros::Publisher robot_radius_publisher;
        //! Publisher of robot center (relative to base_link).
        ros::Publisher robot_center_publisher;
        //! Publisher of robot bounding box (relative to base_link).
        ros::Publisher robot_aabb_publisher;
        //! Publisher of the debug bounding box markers.
        ros::Publisher debug_marker_publisher;
        //! Publisher of scan_point_cloud with robot bounding box cut out.
        ros::Publisher scan_point_cloud_no_bbox_publisher;

        //! Whether to compute bounding sphere of the robot.
        bool compute_bounding_sphere;
        //! Whether to compute bounding box of the robot.
        bool compute_bounding_box;
        //! Whether to compute debug bounding box of the robot.
        bool compute_debug_bounding_box;

        //! The robot-bounding-box cropper.
        pcl::CropBox<pcl::PCLPointCloud2> cropBox;
        //! Input PCL cloud for bbox cropping.
        pcl::PCLPointCloud2::Ptr bbox_crop_input;
        //! Output ROS PointCloud from bbox cropping.
        sensor_msgs::PointCloud2Ptr bbox_crop_output;

        //! Prepared message instance
        geometry_msgs::PointStamped robotRadius;
        //! Prepared message instance
        geometry_msgs::PointStamped robotCenter;
        //! Prepared message instance
        geometry_msgs::PolygonStamped robotAabbMsg;
        //! Prepared message instance
        visualization_msgs::MarkerArray aabbDebugMsg;

        //! The axis-aligned bounding box of the robot (in robot_frame).
        AABB aabb;

        double wait_for_transform;

        //! A mutex that has to be locked in order to work with reachable_frames, shapes_to_links or tf_buffer.
        boost::shared_ptr<boost::mutex> model_mutex;

        //! tf buffer length
        const ros::Duration TF_BUFFER_LENGTH;
        //! tf client
        tf2_ros::Buffer tf_buffer;
        //! tf listener
        tf2_ros::TransformListener tf_listener;
        //! TF frames that have recently been reachable (e.g. canTransform() returns almost immediately)
        /** Only these TFs may be tried to be updated in the update() method. Updating unreachable TFs leads to waiting
         * until canTransform() times out, which rapidly decreases the rate at which the update() method can be run. */
        set<string> reachable_frames;

        //! Watchdog for unreachable frames.
        boost::shared_ptr<UnreachableFramesWatchdog> unreachable_frames_watchdog;

        //! The time when the filter configuration has finished.
        ros::Time time_configured;

        //! The helper used to project laser scans to point clouds.
        laser_geometry::LaserProjection laser_projector;

        //! Tool for masking out 3D bodies out of point clouds.
        scoped_ptr<point_containment_filter::ShapeMask> robot_shape_mask;

        /// A map that correlates shapes in robot_shape_mask to collision links in URDF.
        /** Keys are shape handles from robot_shape_mask. */
        map<point_containment_filter::ShapeHandle, CollisionBodyWithLink> shapes_to_links;

        //! Caches any_link->robot_frame transforms after a scan message is received. Is queried by robot_shape_mask. Keys are link names.
        map<string, boost::shared_ptr<Eigen::Affine3d> > transform_cache;

        /** \brief Return the latest cached transform for the link corresponding to the given shape handle.
         *
         * You should call updateTransformCache before calling this function.
         *
         * \param handle The handle of the shape for which we want the transform. The handle is from robot_shape_mask.
         * \param[out] transform Transform of the corresponding link (wrt robot_frame).
         * \return If the transform was found.
         */
        bool getShapeTransform(point_containment_filter::ShapeHandle handle, Eigen::Affine3d &transform) const;

        /** \brief Update robot_shape_mask with the given URDF model.
         *
         * \param urdf_model The robot's URDF loaded as a string.
         */
        void addRobotMaskFromUrdf(string urdf_model);

        /**
         * \brief Remove all parts of the robot mask and clear internal shape and TF buffers.
         *
         * Make sure no filtering happens when executing this function.
         */
        void clearRobotMask();

        /** \brief Update the cache of link transforms relative to robot_frame.
         *
         * \param time The time to get transforms for.
         */
        void updateTransformCache(const ros::Time time);

        /**
         * \brief Callback handling update of the robot_description parameter using dynamic reconfigure.
         *
         * \param newConfig The updated config.
         */
        void robotDescriptionUpdated(dynamic_reconfigure::ConfigConstPtr newConfig);

        /** \brief Consruct a masking shape out of the given URDF geometry.
         *
         * Just a helper function to convert urdf::Geometry to the corresponding shapes::Shape.
         *
         * Limitation: non-uniformly scaled meshes are not supported (and are scaled uniformly by
         * the first scaling coefficient). Should be overcome once shapes::Shape supports non-uniform scaling.
         *
         * \param geometry The URDF geometry object to convert.
         */
        shapes::Shape *constructShape(const urdf::Geometry *geometry);

        /**
         * \brief Get the extents of the AABB of the shape.
         *
         * \param shape The shape to get extents of.
         * \return Size of the AABB.
         */
        Eigen::Vector3d getShapeExtents(const shapes::Shape& shape);

        //! Helper function to convert URDF pose to Eigen transform.
        /**
         * \param pose The pose to convert.
         */
        inline Eigen::Affine3d urdfPose2EigenTransform(const urdf::Pose &pose);

        //! Helper function to convert geometry_msgs::Point32 to Eigen::Vector3d
        /**
         * \param point The point to convert.
         */
        inline Eigen::Vector3d point32ToEigen(const geometry_msgs::Point32 &point);
    };
}

#endif //_NIFTI_LASER_FILTERING_ROBOTSELFFILTER_H_
