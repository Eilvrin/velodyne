/*
 *  Copyright (C) 2009, 2010 Austin Robot Technology, Jack O'Quin
 *  Copyright (C) 2011 Jesse Vera
 *  Copyright (C) 2012 Austin Robot Technology, Jack O'Quin
 *  License: Modified BSD Software License Agreement
 *
 *  $Id$
 */

/** @file
    This class transforms raw Velodyne 3D LIDAR packets to PointCloud2
    in a given frame of reference.
    @author Jack O'Quin
    @author Jesse Vera
*/

#include "transform.h"

#include <pcl_conversions/pcl_conversions.h>

namespace velodyne_pointcloud
{
  /** @brief Constructor. */
  Transform::Transform(ros::NodeHandle node, ros::NodeHandle private_nh):
    tf_prefix_(tf::getPrefixParam(private_nh)),
    data_(new velodyne_rawdata::RawData())
  {
    data_->setup(private_nh, &listener_);

    // Advertise output point cloud before subscribing to input data.
    output_ =
      node.advertise<sensor_msgs::PointCloud2>("velodyne_points", 10);

    // Subscribe to VelodyneScan packets using transform listener.
    // The target frame of the transform listener will be set in a reconfigure_callback.
    velodyne_scan_.subscribe(node, "velodyne_packets", 10);
    tf_filter_ = new tf::MessageFilter<velodyne_msgs::VelodyneScan>(velodyne_scan_, listener_, "", 10);

    // Set up dynamic reconfiguration.
    srv_ = boost::make_shared<dynamic_reconfigure::Server<velodyne_pointcloud::TransformNodeConfig> >(private_nh);
    dynamic_reconfigure::Server<velodyne_pointcloud::TransformNodeConfig>::CallbackType f;
    f = boost::bind (&Transform::reconfigure_callback, this, _1, _2);
    srv_->setCallback (f);

    // Register message filter at last.
    tf_filter_->registerCallback(boost::bind(&Transform::processScan, this, _1));
  }


  void Transform::reconfigure_callback(
      velodyne_pointcloud::TransformNodeConfig &config, uint32_t level)
  {
    ROS_INFO_STREAM("Reconfigure request.");

    const std::string frame_id = tf::resolve(tf_prefix_, config.frame_id);
    tf_filter_->setTargetFrames(std::vector<std::string>(1, frame_id));
    data_->setParameters(config.min_range, config.max_range,
                         config.view_direction, config.view_width,
                         frame_id, config.fixed_frame_id);
  }


  /// Callback for raw scan messages.
  /// Transforms the scan message to a point cloud and publishes it.
  void Transform::processScan(const velodyne_msgs::VelodyneScan::ConstPtr &scanMsg)
  {
    if (output_.getNumSubscribers() == 0)         // no one listening?
      return;                                     // avoid much work

    // allocate an output point cloud with same time as raw data
    VPointCloud::Ptr outMsg(new VPointCloud());

    // unpack the raw data
    data_->unpack(scanMsg, *outMsg);

    // publish the cloud message
    ROS_DEBUG_STREAM("Publishing " << outMsg->width << " x " << outMsg->height
                     << " Velodyne points, time: " << outMsg->header.stamp);
    output_.publish(outMsg);
  }

} // namespace velodyne_pointcloud

