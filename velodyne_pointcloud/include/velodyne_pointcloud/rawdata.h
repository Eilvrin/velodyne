/* -*- mode: C++ -*-
 *
 *  Copyright (C) 2007 Austin Robot Technology, Yaxin Liu, Patrick Beeson
 *  Copyright (C) 2009, 2010, 2012 Austin Robot Technology, Jack O'Quin
 *
 *  License: Modified BSD Software License Agreement
 *
 *  $Id$
 */

/** @file
 *
 *  @brief Interfaces for interpreting raw packets from the Velodyne 3D LIDAR.
 *
 *  @author Yaxin Liu
 *  @author Patrick Beeson
 *  @author Jack O'Quin
 */

#ifndef __VELODYNE_RAWDATA_H
#define __VELODYNE_RAWDATA_H

#include <errno.h>
#include <stdint.h>
#include <string>
#include <boost/format.hpp>
#include <math.h>
 #include <fstream>

#include <ros/ros.h>
#include <tf/transform_listener.h>
#include <pcl_ros/point_cloud.h>
#include <velodyne_msgs/VelodyneScan.h>
#include <velodyne_pointcloud/point_types.h>
#include <velodyne_pointcloud/calibration.h>

namespace velodyne_rawdata
{
  // Shorthand typedefs for point cloud representations
  typedef velodyne_pointcloud::PointXYZIR VPoint;
  typedef pcl::PointCloud<VPoint> VPointCloud;

  /** Log rate for throttled warnings and errors in seconds */
  static const double LOG_PERIOD_ = 1.0;

  /**
   * Raw Velodyne packet constants and structures.
   */
  static const int SIZE_BLOCK = 100;
  static const int RAW_SCAN_SIZE = 3;
  static const int SCANS_PER_BLOCK = 32;
  static const int BLOCK_DATA_SIZE = (SCANS_PER_BLOCK * RAW_SCAN_SIZE);

  static const float ROTATION_RESOLUTION      =     0.01f;  // [deg]
  static const uint16_t ROTATION_MAX_UNITS    = 36000u;     // [deg/100]
  static const float DISTANCE_RESOLUTION      =     0.002f; // [m]

  /** @todo make this work for both big and little-endian machines */
  static const uint16_t UPPER_BANK = 0xeeff;
  static const uint16_t LOWER_BANK = 0xddff;

  /** Special Defines for VLP16 support **/
  static const int    VLP16_FIRINGS_PER_BLOCK =   2;
  static const int    VLP16_SCANS_PER_FIRING  =  16;
  static const float  VLP16_BLOCK_TDURATION   = 110.592f;   // [µs]
  static const float  VLP16_DSR_TOFFSET       =   2.304f;   // [µs]
  static const float  VLP16_FIRING_TOFFSET    =  55.296f;   // [µs]


  /** \brief Raw Velodyne data block.
   *
   *  Each block contains data from either the upper or lower laser
   *  bank.  The device returns three times as many upper bank blocks.
   *
   *  use stdint.h types, so things work with both 64 and 32-bit machines
   */
  typedef struct raw_block
  {
    uint16_t header;        ///< UPPER_BANK or LOWER_BANK
    uint16_t rotation;      ///< 0-35999, divide by 100 to get degrees
    uint8_t  data[BLOCK_DATA_SIZE];
  } raw_block_t;

  /** used for unpacking the first two data bytes in a block
   *
   *  They are packed into the actual data stream misaligned.  I doubt
   *  this works on big endian machines.
   */
  union two_bytes
  {
    uint16_t uint;
    uint8_t  bytes[2];
  };

  static const int PACKET_SIZE = 1206;
  static const int BLOCKS_PER_PACKET = 12;
  static const int PACKET_STATUS_SIZE = 4;
  static const int SCANS_PER_PACKET = (SCANS_PER_BLOCK * BLOCKS_PER_PACKET);

  /** \brief Raw Velodyne packet.
   *
   *  revolution is described in the device manual as incrementing
   *    (mod 65536) for each physical turn of the device.  Our device
   *    seems to alternate between two different values every third
   *    packet.  One value increases, the other decreases.
   *
   *  \todo figure out if revolution is only present for one of the
   *  two types of status fields
   *
   *  status has either a temperature encoding or the microcode level
   */
  typedef struct raw_packet
  {
    raw_block_t blocks[BLOCKS_PER_PACKET];
    uint16_t revolution;
    uint8_t status[PACKET_STATUS_SIZE];
  } raw_packet_t;

  inline float packet_interp_time(const uint32_t time) {
      // Check endiasness
      int x = 1;
      char *y = (char*)&x;
      uint32_t time_hex;
      if(*y) // Little endian
      {
        // Then do nothing, since the time stamp are transmitted least significant byte first.
        time_hex = time;
      } else { // Big endian
        // Then reverse byte order
        time_hex =  ((time>>24)&0xff)    |   // move byte 3 to byte 0
                    ((time<<8)&0xff0000) |   // move byte 1 to byte 2
                    ((time>>8)&0xff00)   |   // move byte 2 to byte 1
                    ((time<<24)&0xff000000); // byte 0 to byte 3
      }
      float time_dec;
      std::stringstream stream;
      stream << time_hex;
      stream >> std::hex >> time_dec;
      return time_dec;
    }

  typedef struct raw_packet_vlp16
  {
    raw_block_t blocks[BLOCKS_PER_PACKET];
    uint32_t time;        // Time in microseconds from startup. Probably overshoots each hour.
    uint8_t return_type;  // 37 - strongest return; 38 - last return; 39 - dual return
    uint8_t data_source;  // 21 for HDL-32E or 22 for VLP-16
  } raw_packet_vlp16_t;

  /** \brief Velodyne data conversion class */
  class RawData
  {
  public:

    RawData();
    ~RawData() {}

    /** \brief Set up for data processing.
     *
     *  Perform initializations needed before data processing can
     *  begin:
     *
     *    - read device-specific angles calibration
     *    - specify transform listener
     *
     *  @param private_nh private node handle for ROS parameters
     *  @param tf_listener_ transform listener for transforming points
     *  @returns 0 if successful;
     *           errno value for failure
     */
    int setup(ros::NodeHandle private_nh, tf::TransformListener* tf_listener = NULL);

    /** @brief convert raw Velodyne message to point cloud
     *
     *  @param scanMsg raw Velodyne scan message
     *  @param pc shared pointer to organized point cloud
     */
    void unpack(const velodyne_msgs::VelodyneScan::ConstPtr &scanMsg, VPointCloud &pc);

    void setParameters(double min_range, double max_range, double view_direction,
                       double view_width, const std::string& frame_id = "", const std::string& fixed_frame_id = "");

  private:

    /** configuration parameters */
    typedef struct {
      std::string calibrationFile;     ///< calibration file name
      double max_range;                ///< maximum range to publish
      double min_range;                ///< minimum range to publish
      int min_angle;                   ///< minimum angle to publish
      int max_angle;                   ///< maximum angle to publish
      std::string frame_id;            ///< frame into which to transform points
      std::string fixed_frame_id;     ///<  fixed frame for tf transform

      double tmp_min_angle;
      double tmp_max_angle;
    } Config;
    Config config_;

    /**
     * Calibration file
     */
    velodyne_pointcloud::Calibration calibration_;
    float sin_rot_table_[ROTATION_MAX_UNITS];
    float cos_rot_table_[ROTATION_MAX_UNITS];

    tf::TransformListener* tf_listener_;

    std::ofstream file_;

    /** @brief convert raw VLP16 message to point cloud
     *
     *  @param scanMsg raw Velodyne scan message
     *  @param pc shared pointer to point cloud (points are appended)
     */
    void unpack_vlp16(const velodyne_msgs::VelodyneScan::ConstPtr &scanMsg,
                      VPointCloud &pc);

    /** in-line test whether a point is in range */
    bool pointInRange(float range)
    {
      return (range >= config_.min_range
              && range <= config_.max_range);
    }
  };

} // namespace velodyne_rawdata

#endif // __VELODYNE_RAWDATA_H
