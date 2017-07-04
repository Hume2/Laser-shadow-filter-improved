/* HACK HACK HACK */
/* We use it to access mesh bounding box. */
#define protected public
#include <geometric_shapes/bodies.h>
#undef protected

/* HACK HACK HACK */
/* We use it so that we don't need to duplicate bounding spheres computations. */
#define private public
#include <moveit/point_containment_filter/shape_mask.h>
#undef private

#include "nifti_laser_filtering/RobotSelfFilter.h"

#include "pluginlib/class_list_macros.h"

#include <geometric_shapes/body_operations.h>

#include <sensor_msgs/point_cloud_conversion.h>
#include <eigen_conversions/eigen_msg.h>
#include <sensor_msgs/point_cloud2_iterator.h>

#include <pcl_conversions/pcl_conversions.h>
#include <tf2_sensor_msgs/tf2_sensor_msgs.h>

using namespace std;
using namespace boost;
using namespace sensor_msgs;
using namespace filters;

namespace robot_self_filter {

    /**
     * @brief remainingTime Return remaining time to timeout from the query time.
     * @param query The query time, e.g. of the tf transform.
     * @param maxTimeout Maximum time to wait from the query time onwards.
     * @return
     */
    ros::Duration remainingTime(const ros::Time &query, const double timeout) {
        const double past = (ros::Time::now() - query).toSec();
        return ros::Duration(std::max(0.0, timeout - past));
    }

    inline void strip_leading_slash(std::string& s)
    {
        if (s.length() > 0 && s[0] == '/') {
            s.erase(0, 1);
        }
    }

    RobotSelfFilter::RobotSelfFilter():
            TF_BUFFER_LENGTH(ros::Duration(60.0)),
            tf_buffer(TF_BUFFER_LENGTH),
            tf_listener(tf_buffer) {

        this->model_mutex.reset(new boost::mutex());

        this->robotAabbMsg.polygon.points.resize(2);

        this->cropBox.setNegative(true);

        this->bbox_crop_input.reset(new pcl::PCLPointCloud2);
        this->bbox_crop_output.reset(new PointCloud2);
    }

    RobotSelfFilter::~RobotSelfFilter() {
    }

    bool RobotSelfFilter::configure() {
        // clear the TF buffer (useful if calling configure() after receiving old TF data)
        tf_buffer.clear();

        if (FilterBase<LaserScan>::getParam("robot_frame", robot_frame)) {
            ROS_DEBUG("RobotSelfFilter: found param robot_frame: %s", robot_frame.c_str());
        } else {
            ROS_INFO("RobotSelfFilter was not given robot_frame, assuming base_link.");
            robot_frame = "base_link";
        }

        if (FilterBase<LaserScan>::getParam("min_distance", min_distance)) {
            ROS_DEBUG("RobotSelfFilter: found param min_distance: %.5f m", min_distance);
        } else {
            ROS_INFO("RobotSelfFilter was not given min_distance, assuming 0.0 m.");
            min_distance = 0.0;
        }

        if (FilterBase<LaserScan>::getParam("max_distance", max_distance)) {
            ROS_DEBUG("RobotSelfFilter: found param max_distance: %.5f m", max_distance);
        } else {
            ROS_INFO("RobotSelfFilter was not given max_distance, assuming 100.0 m.");
            min_distance = 100.0;
        }

        if (FilterBase<LaserScan>::getParam("inflation_padding", inflation_padding)) {
            ROS_DEBUG("RobotSelfFilter: found param inflation_padding: %.5f m", inflation_padding);
        } else {
            ROS_INFO("RobotSelfFilter was not given inflation_padding, assuming 0.01 m.");
            inflation_padding = 0.01;
        }

        if (FilterBase<LaserScan>::getParam("inflation_scale", inflation_scale)) {
            ROS_DEBUG("RobotSelfFilter: found param inflation_scale: %.5f", inflation_scale);
        } else {
            ROS_INFO("RobotSelfFilter was not given inflation_scale, assuming 1.15 .");
            inflation_scale = 1.15;
        }

        if (FilterBase<LaserScan>::getParam("robot_description_param", robot_description_param)) {
            ROS_DEBUG("RobotSelfFilter: found param robot_description_param: %s", robot_description_param.c_str());
        } else {
            ROS_INFO("RobotSelfFilter was not given robot_description_param, assuming /robot_description.");
            robot_description_param = "/robot_description";
        }

        if (FilterBase<LaserScan>::getParam("wait_for_transform", wait_for_transform)) {
            ROS_DEBUG("RobotSelfFilter: found param wait_for_transform: %f s", wait_for_transform);
        } else {
            wait_for_transform = 1.0;
            ROS_INFO("RobotSelfFilter was not given wait_for_transform, assuming %f s.", wait_for_transform);
        }

        if (FilterBase<LaserScan>::getParam("compute_bounding_sphere", compute_bounding_sphere)) { // update your ROS if build fails here!
            ROS_DEBUG("RobotSelfFilter: found param compute_bounding_sphere: %i", compute_bounding_sphere);
        } else {
            compute_bounding_sphere = true;
            ROS_INFO("RobotSelfFilter was not given compute_bounding_sphere, assuming %i.", compute_bounding_sphere);
        }

        if (FilterBase<LaserScan>::getParam("compute_bounding_box", compute_bounding_box)) { // update your ROS if build fails here!
            ROS_DEBUG("RobotSelfFilter: found param compute_bounding_box: %i", compute_bounding_box);
        } else {
            compute_bounding_box = true;
            ROS_INFO("RobotSelfFilter was not given compute_bounding_box, assuming %i.", compute_bounding_box);
        }

        if (FilterBase<LaserScan>::getParam("compute_debug_bounding_box", compute_debug_bounding_box)) { // update your ROS if build fails here!
            ROS_DEBUG("RobotSelfFilter: found param compute_debug_bounding_box: %i", compute_debug_bounding_box);
        } else {
            compute_debug_bounding_box = false;
            ROS_INFO("RobotSelfFilter was not given compute_debug_bounding_box, assuming %i.", compute_debug_bounding_box);
        }

        // subscribe for robot_description param changes
        robot_description_updates_listener = this->nodeHandle.subscribe(
                "dynamic_robot_model_server/parameter_updates", 10, &RobotSelfFilter::robotDescriptionUpdated, this);

        if (this->compute_bounding_sphere) {
            robot_radius_publisher = this->nodeHandle.advertise<stamped_msgs::Float32>("robot_radius", 100);
            robot_center_publisher = this->nodeHandle.advertise<geometry_msgs::PointStamped>("robot_center", 100);
        }

        if (this->compute_bounding_box) {
            robot_aabb_publisher = this->nodeHandle.advertise<geometry_msgs::PolygonStamped>("robot_bounding_box", 100);
            scan_point_cloud_no_bbox_publisher = this->nodeHandle.advertise<sensor_msgs::PointCloud2>("scan_point_cloud_no_bbox", 100);
        }

        if (this->compute_debug_bounding_box) {
            debug_marker_publisher = this->nodeHandle.advertise<visualization_msgs::MarkerArray>(
                    "robot_bounding_box_debug", 100);
        }

        // initialize the 3D body masking tool
        robot_shape_mask.reset(new point_containment_filter::ShapeMask());
        robot_shape_mask->setTransformCallback(bind(&RobotSelfFilter::getShapeTransform, this, _1, _2));

        { // initialize the robot body to be masked out
            string robot_urdf("");
            while (!this->nodeHandle.getParam(robot_description_param, robot_urdf) || robot_urdf.length() == 0) {
                if (!ros::ok())
                    return false;

                ROS_ERROR("RobotSelfFilter: %s is empty or not set. Please, provide the robot model. Waiting 1s.",
                          robot_description_param.c_str());
                ros::Duration(1.0).sleep();
            }

            this->addRobotMaskFromUrdf(robot_urdf);
        }


        unreachable_frames_watchdog.reset(new UnreachableFramesWatchdog(robot_frame, reachable_frames, shapes_to_links,
                                                                        TF_BUFFER_LENGTH, this->model_mutex));
        unreachable_frames_watchdog->start(0.2);

        ROS_INFO("RobotSelfFilter: Successfully configured.");

        this->time_configured = ros::Time::now();

        return true;
    }

    bool RobotSelfFilter::update(const LaserScan &input_scan, LaserScan &filtered_scan) {
        const ros::Time scan_time = input_scan.header.stamp;

        if (!this->configured_) {
            ROS_DEBUG("Ignore scan from time %u.%u - filter not yet initialized.",
                      scan_time.sec, scan_time.nsec);
            return false;
        } else if ((scan_time < time_configured) && (scan_time >= (time_configured - TF_BUFFER_LENGTH))) {
            ROS_DEBUG("Ignore scan from time %u.%u - filter not yet initialized.",
                      scan_time.sec, scan_time.nsec);
            return false;
        } else if ((scan_time < time_configured) && (scan_time < (time_configured - TF_BUFFER_LENGTH))) {
            ROS_WARN("Old TF data received. Clearing TF buffer and reconfiguring laser filter.");
            this->configure();
            return false;
        }

        const clock_t stopwatch_overall = clock();

        // tf2 doesn't like frames starting with slash
        string scan_frame = input_scan.header.frame_id;
        strip_leading_slash(scan_frame);

        // create the output copy of the input scan
        filtered_scan = input_scan;
        filtered_scan.header.frame_id = scan_frame;
        filtered_scan.range_min = max(input_scan.range_min, (float) min_distance);

        { // acquire the lock here, because we work with the tf_buffer all the time
            boost::lock_guard<boost::mutex> guard(*this->model_mutex);

            { // make sure we have all the tfs between laser frame and robot_frame during the time of scan acquisition
                const float scan_duration = input_scan.ranges.size() * input_scan.time_increment;
                const ros::Time after_scan_time = scan_time + ros::Duration().fromSec(scan_duration);

                string err;
                if (!tf_buffer.canTransform(robot_frame, scan_frame, scan_time,
                                            remainingTime(scan_time, wait_for_transform), &err) ||
                    !tf_buffer.canTransform(robot_frame, scan_frame, after_scan_time,
                                            remainingTime(after_scan_time, wait_for_transform), &err)) {
                    ROS_ERROR_THROTTLE(3,
                                       "RobotSelfFilter: Cannot transform laser scan to robot frame. Something's wrong "
                                               "with TFs: %s", err.c_str());

                    return false;
                }
            }

            // The point cloud will have fields x, y, z, intensity (float32) and index (int32)
            sensor_msgs::PointCloud2 point_cloud;
            { // project the scan measurements to a point cloud in the robot_frame
                const int channel_options = laser_geometry::channel_option::Default;
                // here we should use input_scan instead of filtered_scan, but it has frame_id /laser which breaks tf2
                // and is const; on the other hand, filtered_scan hasn't been changed yet except the corrected frame_id
                laser_projector.transformLaserScanToPointCloud(
                        robot_frame, filtered_scan, point_cloud, tf_buffer, -1, channel_options);

                // the projected point cloud can omit some measurements if they are out of the defined scan's range;
                // for this case, the second channel ("index") contains indices of the point cloud's points into the scan

                point_cloud.header.frame_id = robot_frame;
                // according to LaserProjector, the point cloud is created so that it corresponds to the time of the first
                // measurement
                point_cloud.header.stamp = scan_time;
            }

            // update transforms cache, which is then used in maskContainment call
            updateTransformCache(scan_time);

            vector<int> point_mask;
            {// compute a mask of point indices for points from point_cloud that tells if they are inside or outside robot
                const Eigen::Vector3d distance_measuring_center; // we want to measure distances from the robot body's center

                // updates shapes according to tf cache (by calling getShapeTransform for each shape)
                // and masks contained points
                robot_shape_mask->maskContainment(point_cloud, distance_measuring_center, 0.0, max_distance,
                                                  point_mask);
                
                if (this->compute_bounding_sphere)
                {
                    // compute the robot radius (aka a very coarse filter for those who want it)
                    bodies::BoundingSphere bound;
                    {
                        boost::mutex::scoped_lock _(robot_shape_mask->shapes_lock_);
                        // HACK bspheres_ is normally private, but we steal it :)
                        bodies::mergeBoundingSpheres(robot_shape_mask->bspheres_, bound);
                    }
                    
                    robotRadius.header.stamp = scan_time;
                    robotRadius.header.frame_id = robot_frame;
                    robotRadius.data = bound.radius;
                    robot_radius_publisher.publish(robotRadius);
                    
                    robotCenter.header.stamp = scan_time;
                    robotCenter.header.frame_id = robot_frame;
                    robotCenter.point.x = bound.center[0];
                    robotCenter.point.y = bound.center[1];
                    robotCenter.point.z = bound.center[2];
                    robot_center_publisher.publish(robotCenter);
                }

                if (this->compute_bounding_box) {
                    // the box is updated in updateTransformCache()

                    robotAabbMsg.header.stamp = scan_time;
                    robotAabbMsg.header.frame_id = robot_frame;

                    robotAabbMsg.polygon.points[0].x = static_cast<float>(aabb.minCorner().x());
                    robotAabbMsg.polygon.points[0].y = static_cast<float>(aabb.minCorner().y());
                    robotAabbMsg.polygon.points[0].z = static_cast<float>(aabb.minCorner().z());

                    robotAabbMsg.polygon.points[1].x = static_cast<float>(aabb.maxCorner().x());
                    robotAabbMsg.polygon.points[1].y = static_cast<float>(aabb.maxCorner().y());
                    robotAabbMsg.polygon.points[1].z = static_cast<float>(aabb.maxCorner().z());

                    robot_aabb_publisher.publish(robotAabbMsg);
                }
            }

            { // remove points that are inside the model (not outside or clipped)
                const float INVALID_POINT_VALUE = std::numeric_limits<float>::quiet_NaN();
                try {
                    sensor_msgs::PointCloud2Iterator<int> index_it(point_cloud, "index");

                    for (vector<int>::const_iterator it = point_mask.begin();
                         it != point_mask.end(); ++it, ++index_it) {
                        const int mask_value = *it;
                        if (mask_value == point_containment_filter::ShapeMask::INSIDE) {
                            const size_t index_in_scan = *index_it;
                            filtered_scan.ranges[index_in_scan] = INVALID_POINT_VALUE;
                        }
                    }
                }
                catch (std::runtime_error) {
                    ROS_ERROR("RobotSelfFilter: point_cloud_from_scan doesn't have field called 'index',"
                                      " but the algorithm relies on that.");
                    return false;
                }
            }

            if (this->compute_bounding_box) { // compute (and publish!) the scan_point_cloud with robot bounding box removed

                pcl_conversions::moveToPCL(point_cloud, *(this->bbox_crop_input));  // point_cloud.data is invalid from now on!
                this->cropBox.setInputCloud(this->bbox_crop_input);

                this->cropBox.setMin(Eigen::Vector4f((float)this->aabb.minCorner().x(),
                                                     (float)this->aabb.minCorner().y(),
                                                     (float)this->aabb.minCorner().z(),
                                                     0.0f));
                this->cropBox.setMax(Eigen::Vector4f((float)this->aabb.maxCorner().x(),
                                                     (float)this->aabb.maxCorner().y(),
                                                     (float)this->aabb.maxCorner().z(),
                                                     0.0f));

                pcl::PCLPointCloud2 pcl_output;
                this->cropBox.filter(pcl_output);

                sensor_msgs::PointCloud2 ros_output;
                pcl_conversions::moveFromPCL(pcl_output, ros_output);

                this->tf_buffer.transform(ros_output, *this->bbox_crop_output, "laser");

                this->scan_point_cloud_no_bbox_publisher.publish(bbox_crop_output);
            }

            ROS_DEBUG("RobotSelfFilter: Scan filtering run time is %.5f secs.", double(clock()-stopwatch_overall) / CLOCKS_PER_SEC);
        }

        return true;
    }

    bool RobotSelfFilter::getShapeTransform(point_containment_filter::ShapeHandle shape_handle, Eigen::Affine3d &transform) const {
        // make sure you locked this->model_mutex

        // check if the given shape_handle has been registered to a link during addRobotMaskFromUrdf call.
        if (shapes_to_links.find(shape_handle) == shapes_to_links.end()) {
            ROS_ERROR_STREAM("RobotSelfFilter: Invalid shape handle: " << lexical_cast<string>(shape_handle));
            return false;
        }

        CollisionBodyWithLink collision = shapes_to_links.at(shape_handle);
        string collision_cache_key = collision.getCacheKey();

        // find the latest transform for the link
        if (transform_cache.find(collision_cache_key) == transform_cache.end()) {
            // do not log the error because unreachable frame watchdog would do it for us
            return false;
        }

        transform = *transform_cache.at(collision_cache_key);

        return true;
    }

    void RobotSelfFilter::updateTransformCache(const ros::Time time) {
        // make sure you locked this->model_mutex

        // clear the cache so that maskContainment always uses only these tf data and not some older
        transform_cache.clear();

        if (compute_bounding_box || compute_debug_bounding_box) {
            aabb.reset();
        }

        if (compute_debug_bounding_box) {
            aabbDebugMsg.markers.clear();
        }

        string err;
        // iterate over all links corresponding to some masking shape and update their cached transforms relative
        // to robot_frame
        for (map<point_containment_filter::ShapeHandle, CollisionBodyWithLink>::const_iterator it = shapes_to_links.begin();
             it != shapes_to_links.end(); ++it) {

            boost::shared_ptr<urdf::Collision> collision = it->second.collision;
            boost::shared_ptr<urdf::Link> link = it->second.link;

            // here we assume the tf frames' names correspond to the link names
            string link_frame = link->name;

            // only update TF cache for reachable frames
            if (reachable_frames.find(link_frame) == reachable_frames.end())
                continue;

            if (!tf_buffer.canTransform(robot_frame, link_frame, time, remainingTime(time, wait_for_transform), &err)) {
                ROS_WARN_THROTTLE(3, "RobotSelfFilter: Could not get transform %s->%s! Laser filtering may fail. Cause: %s",
                         robot_frame.c_str(), link_frame.c_str(), err.c_str());
                err.clear();
                // if we couldn't get TF for this reachable frame, mark it unreachable
                reachable_frames.erase(link_frame);
                continue;
            }

            geometry_msgs::TransformStamped link_transform_tf =
                    tf_buffer.lookupTransform(robot_frame, link_frame, time, remainingTime(time, wait_for_transform));

            Eigen::Affine3d link_transform_eigen;
            tf::transformMsgToEigen(link_transform_tf.transform, link_transform_eigen);

            // the collision object may have a different origin than the visual, we need to account for that
            Eigen::Affine3d collision_static_transform = urdfPose2EigenTransform(collision->origin);
            Eigen::Affine3d transform = link_transform_eigen * collision_static_transform;

            string transform_cache_key = it->second.getCacheKey();
            transform_cache[transform_cache_key] = boost::make_shared<Eigen::Affine3d>(transform);

            if (compute_bounding_box || compute_debug_bounding_box) {
                aabb.update(transform, it->second.shape_extents);
            }

            if (compute_debug_bounding_box) {
                visualization_msgs::Marker msg;
                msg.header.stamp = time;
                msg.header.frame_id = robot_frame;

                msg.scale.x = it->second.shape_extents[0];
                msg.scale.y = it->second.shape_extents[1];
                msg.scale.z = it->second.shape_extents[2];

                msg.pose.position.x = transform.translation()[0];
                msg.pose.position.y = transform.translation()[1];
                msg.pose.position.z = transform.translation()[2];

                Eigen::Quaterniond q(transform.rotation());
                msg.pose.orientation.x = q.x();
                msg.pose.orientation.y = q.y();
                msg.pose.orientation.z = q.z();
                msg.pose.orientation.w = q.w();

                msg.color.g = 1.0;
                msg.color.a = 0.5;
                msg.type = visualization_msgs::Marker::CUBE;
                msg.action = visualization_msgs::Marker::ADD;
                msg.ns = transform_cache_key;
                msg.frame_locked = true;

                aabbDebugMsg.markers.push_back(msg);
            }
        }

        if (compute_debug_bounding_box) {
            debug_marker_publisher.publish(aabbDebugMsg);
        }
    }

    void RobotSelfFilter::addRobotMaskFromUrdf(string urdf_model) {
        if (urdf_model == "") {
            ROS_ERROR("RobotSelfFilter: Empty string passed as robot model to addRobotMaskFromUrdf. "
                             "Robot body filtering is not going to work.");
            return;
        }

        // parse the URDF model
        urdf::Model parsed_urdf_model;
        bool urdf_parse_succeeded = parsed_urdf_model.initString(urdf_model);
        if (!urdf_parse_succeeded) {
            ROS_ERROR("RobotSelfFilter: The URDF model given in robot_description cannot be parsed. "
                              "See urdf::Model::initString for debugging, or try running 'gzsdf my_robot.urdf'.");
            return;
        }

        // add all model's collision links as masking shapes
        for (map<string, boost::shared_ptr<urdf::Link> >::const_iterator it = parsed_urdf_model.links_.begin();
             it != parsed_urdf_model.links_.end(); ++it) {

            boost::shared_ptr<urdf::Link> link = it->second;

            // every link can have multiple collision elements
            size_t collision_index = 0;
            for (vector<boost::shared_ptr<urdf::Collision> >::const_iterator collision_it = link->collision_array.begin();
                 collision_it != link->collision_array.end(); ++collision_it) {

                boost::shared_ptr<urdf::Collision> collision = (*collision_it);

                if (collision->geometry == NULL) {
                    ROS_WARN("RobotSelfFilter: Collision element without geometry found in link %s of robot %s. "
                            "This collision element will not be filtered out from laser scans.",
                            link->name.c_str(), parsed_urdf_model.getName().c_str());
                    continue;
                }

                shapes::Shape *collision_shape = constructShape(collision->geometry.get());
                const Eigen::Vector3d shape_extents = getShapeExtents(*collision_shape) * inflation_scale +
                        inflation_padding * Eigen::Vector3d::Ones();

                // add the collision shape to robot_shape_mask; the inflation parameters come into play here
                point_containment_filter::ShapeHandle shape_handle =
                        robot_shape_mask->addShape(boost::shared_ptr<shapes::Shape const>(collision_shape),
                                inflation_scale, inflation_padding);
                shapes_to_links[shape_handle] = CollisionBodyWithLink(collision, link, collision_index, shape_extents);

                ++collision_index;
            }

            if (collision_index == 0) {
                ROS_INFO("RobotSelfFilter: No collision element found for link %s of robot %s. This link will not be filtered out "
                        "from laser scans.", link->name.c_str(), parsed_urdf_model.getName().c_str());
            }
        }
    }

    void RobotSelfFilter::clearRobotMask() {
        {
            boost::lock_guard<boost::mutex> guard(*this->model_mutex);

            for (map<point_containment_filter::ShapeHandle, CollisionBodyWithLink>::const_iterator it = shapes_to_links.begin();
                 it != shapes_to_links.end(); ++it) {
                point_containment_filter::ShapeHandle handle = it->first;

                robot_shape_mask->removeShape(handle);
            }
            shapes_to_links.clear();
            transform_cache.clear();
            reachable_frames.clear();
            tf_buffer.clear();
        }

        unreachable_frames_watchdog->clear();
    }

    void RobotSelfFilter::robotDescriptionUpdated(dynamic_reconfigure::ConfigConstPtr newConfig) {
        std::string urdf(newConfig->strs[0].value);

        ROS_INFO("RobotSelfFilter: Reloading robot model because of dynamic_reconfigure update. Filter operation stopped.");

        unreachable_frames_watchdog->pause();
        configured_ = false;

        clearRobotMask();
        addRobotMaskFromUrdf(urdf);

        unreachable_frames_watchdog->unpause();
        time_configured = ros::Time::now();
        configured_ = true;

        ROS_INFO("RobotSelfFilter: Robot model reloaded, resuming filter operation.");
    }

    /////////////////////
    // HELPER FUNCTIONS//
    /////////////////////

    shapes::Shape *RobotSelfFilter::constructShape(const urdf::Geometry *geometry) {
        ROS_ASSERT(geometry);
        shapes::Shape *result = NULL;

        switch (geometry->type) {
            case urdf::Geometry::SPHERE: {
                result = new shapes::Sphere(dynamic_cast<const urdf::Sphere *>(geometry)->radius);
                break;
            }
            case urdf::Geometry::BOX: {
                urdf::Vector3 dim = dynamic_cast<const urdf::Box *>(geometry)->dim;
                result = new shapes::Box(dim.x, dim.y, dim.z);
                break;
            }
            case urdf::Geometry::CYLINDER: {
                result = new shapes::Cylinder(dynamic_cast<const urdf::Cylinder *>(geometry)->radius,
                        dynamic_cast<const urdf::Cylinder *>(geometry)->length);
                break;
            }
            case urdf::Geometry::MESH: {
                const urdf::Mesh *mesh = dynamic_cast<const urdf::Mesh *>(geometry);
                if (!mesh->filename.empty()) {
                    result = shapes::createMeshFromResource(mesh->filename);

                    // TODO watch https://github.com/ros-planning/geometric_shapes/issues/29 if it is solved
                    if (mesh->scale.x != mesh->scale.y || mesh->scale.y != mesh->scale.z || mesh->scale.x != mesh->scale.z) {
                        ROS_WARN_STREAM("Nonuniform mesh scaling not supported in meshes for laser filtering." <<
                                " Using X scale as the general scale. The problematic mesh: " << mesh->filename);
                    }

                    result->scale(mesh->scale.x);
                } else
                    ROS_WARN("Empty mesh filename");
                break;
            }
            default: {
                ROS_ERROR("Unknown geometry type: %d", (int) geometry->type);
                break;
            }
        }
        return result;
    }

    Eigen::Vector3d RobotSelfFilter::getShapeExtents(const shapes::Shape& shape) {
        switch (shape.type) {
            case shapes::BOX: {
                return Eigen::Vector3d(dynamic_cast<const shapes::Box&>(shape).size);
            }

            case shapes::SPHERE: {
                const shapes::Sphere& sphere = dynamic_cast<const shapes::Sphere&>(shape);
                const double width = 2 * sphere.radius;
                return Eigen::Vector3d(width, width, width);
            }

            case shapes::CYLINDER: {
                const shapes::Cylinder& cylinder = dynamic_cast<const shapes::Cylinder&>(shape);
                const double width = 2 * cylinder.radius;
                return Eigen::Vector3d(width, width, cylinder.length);
            }

            case shapes::CONE: {
                const shapes::Cone& cone = dynamic_cast<const shapes::Cone&>(shape);
                const double width = 2 * cone.radius;
                return Eigen::Vector3d(width, width, cone.length);
            }
            case shapes::MESH: {
                const shapes::Mesh& mesh = dynamic_cast<const shapes::Mesh&>(shape);
                const bodies::ConvexMesh meshBody(&mesh);
                // HACK bounding_box_ is normally protected, but we steal it :)
                return Eigen::Vector3d(meshBody.bounding_box_.getDimensions().data());
            }

            case shapes::UNKNOWN_SHAPE:  // intentional fall-through
            case shapes::PLANE: // intentional fall-through
            case shapes::OCTREE: {
                ROS_ERROR("Cannot process shape of type %d", (int) shape.type);
                return Eigen::Vector3d();
            }
        }
    }

    inline Eigen::Affine3d RobotSelfFilter::urdfPose2EigenTransform(const urdf::Pose &pose) {
        return Eigen::Translation3d(pose.position.x, pose.position.y, pose.position.z) *
                Eigen::Quaterniond(pose.rotation.w, pose.rotation.x, pose.rotation.y, pose.rotation.z);
        // who the fuck decided that W is put first in Eigen::Quaternion constructor???
    }

    inline Eigen::Vector3d RobotSelfFilter::point32ToEigen(const geometry_msgs::Point32 &point) {
        Eigen::Vector3d point_eigen;
        point_eigen[0] = point.x; point_eigen[1] = point.y; point_eigen[2] = point.z;
        return point_eigen;
    }

    UnreachableFramesWatchdog::UnreachableFramesWatchdog(string robot_frame, set<string> &reachable_frames,
                                                         map<point_containment_filter::ShapeHandle, CollisionBodyWithLink> &shapes_to_links,
                                                         ros::Duration TF_BUFFER_LENGTH, boost::shared_ptr<boost::mutex> model_mutex):
            robot_frame(robot_frame),
            reachable_frames(reachable_frames),
            shapes_to_links(shapes_to_links),
            tf_buffer(TF_BUFFER_LENGTH),
            tf_listener(tf_buffer),
            paused(true),
            model_mutex(model_mutex) {
    }

    void UnreachableFramesWatchdog::start(double tf_wait_timeout) {
        this_thread = thread(&UnreachableFramesWatchdog::run, this, tf_wait_timeout);
        this->unpause();
    }

    void UnreachableFramesWatchdog::run(double tf_wait_timeout) {
        string err;
        ros::Rate rate(1.0);
        ros::Time time;
        ros::Duration tf_timeout(tf_wait_timeout);

        while (ros::ok()) {
            if (!this->paused) { // the thread is paused when we want to change the stuff protected by model_mutex.
                time = ros::Time::now();

                // detect all unreachable frames
                // we can't join this loop with the following one, beacase we need to lock the model_mutex, but
                // canTransform can hang for pretty long, which would result in deadlocks

                set<string> unreachable_frames;
                {
                    boost::lock_guard<boost::mutex> guard(*this->model_mutex);

                    for (map<point_containment_filter::ShapeHandle, CollisionBodyWithLink>::const_iterator it = shapes_to_links.begin();
                         it != shapes_to_links.end(); ++it) {
                        boost::shared_ptr<urdf::Link> link = it->second.link;
                        // here we assume the tf frames' names correspond to the link names
                        string link_frame = link->name;

                        if (reachable_frames.find(link_frame) == reachable_frames.end()) {
                            unreachable_frames.insert(link_frame);
                        }
                    }
                }

                // now, the mutex is unlocked and we try to get transforms to all of the unreachable links... that
                // could take a while
                for (set<string>::const_iterator it = unreachable_frames.begin();
                     it != unreachable_frames.end(); ++it) {
                    if (this->paused) {
                        break;
                    }

                    string link_frame = *it;
		            err.clear();
                    if (tf_buffer.canTransform(robot_frame, link_frame, time, tf_timeout, &err)) {
                        // only lock the mutex if we really need to update the protected data
                        boost::lock_guard<boost::mutex> guard(*this->model_mutex);
                        reachable_frames.insert(link_frame);
                        ROS_DEBUG("RobotSelfFilter: Frame became reachable: %s, %i.%i", link_frame.c_str(), time.sec, time.nsec);
                    } else {
                        ROS_WARN_THROTTLE(3,
                                          "RobotSelfFilter: Could not get transform %s->%s! Laser filtering may fail. Cause: %s",
                                          robot_frame.c_str(), link_frame.c_str(), err.c_str());
                    }
                }
            }
            rate.sleep();
        }
    }

    void UnreachableFramesWatchdog::pause() {
        this->paused = true;
    }

    void UnreachableFramesWatchdog::unpause() {
        this->paused = false;
    }

    void UnreachableFramesWatchdog::clear() {
        boost::lock_guard<boost::mutex> guard(*this->model_mutex);
        shapes_to_links.clear();
        tf_buffer.clear();
        reachable_frames.clear();
    }
}

PLUGINLIB_EXPORT_CLASS(robot_self_filter::RobotSelfFilter, filters::FilterBase<sensor_msgs::LaserScan>);
