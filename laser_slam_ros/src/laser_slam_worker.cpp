#include "laser_slam_ros/laser_slam_worker.hpp"

#include "laser_slam/benchmarker.hpp"

//TODO clean
#include <Eigen/Eigenvalues>
#include <geometry_msgs/Pose.h>
#include <geometry_msgs/PoseStamped.h>
#include <gtsam/nonlinear/NonlinearFactorGraph.h>
#include <laser_slam/benchmarker.hpp>
#include <laser_slam/common.hpp>
#include <laser_slam_ros/common.hpp>
#include <laser_slam_ros/laser_slam_worker.hpp>
#include <nav_msgs/Odometry.h>
#include <nav_msgs/Path.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/common/transforms.h>
#include <pcl/io/pcd_io.h>
#include <pointmatcher_ros/point_cloud.h>
#include <pointmatcher_ros/transform.h>
#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <std_msgs/String.h>
#include <tf_conversions/tf_eigen.h>
#include <tf/transform_broadcaster.h>
#include <tf/transform_listener.h>
#include <visualization_msgs/Marker.h>
#include <visualization_msgs/MarkerArray.h>

#include <cmath>

namespace laser_slam_ros {

using namespace laser_slam;

LaserSlamWorker::LaserSlamWorker() { }

LaserSlamWorker::~LaserSlamWorker() { }

void LaserSlamWorker::init(
    ros::NodeHandle& nh, const LaserSlamWorkerParams& params,
    std::shared_ptr<laser_slam::IncrementalEstimator> incremental_estimator,
    unsigned int worker_id) {
  params_ = params;
  incremental_estimator_ = incremental_estimator;
  worker_id_ = worker_id;

  // Get the LaserTrack object from the IncrementalEstimator.
  laser_track_ = incremental_estimator_->getLaserTrack(worker_id);

  // Setup subscriber.  
  if (params_.VoxelNet_format)
  {
    scan_sub_ = nh.subscribe(params_.assembled_cloud_sub_topic, kScanSubscriberMessageQueueSize,
                          &LaserSlamWorker::scanCallback_VoxelNetFormat, this); 
  }
  else if (params_.ARGOS_format)
  {
    // for the labeled KITTI dataset
    {
      labeled_points_sub = new message_filters::Subscriber<laser_slam::LabeledPointCloud> (nh, "/labeled_points", 1);
      imu_sub = new message_filters::Subscriber<sensor_msgs::Imu> (nh, "/kitti/oxts/imu", 1);
      gps_sub = new message_filters::Subscriber<sensor_msgs::NavSatFix> (nh, "/kitti/oxts/gps/fix", 1);

      sync.reset(new Sync(MySyncPolicy(10), *labeled_points_sub, *imu_sub, *gps_sub)); 
      sync->registerCallback(boost::bind(&LaserSlamWorker::scanCallback_ARGOS_Format, this, _1, _2, _3));
    }

    // SR dataset
    // {
    //   labeled_points_sub = new message_filters::Subscriber<laser_slam::LabeledPointCloud> (nh, "/labeled_points", 1);
    //   imu_sub = new message_filters::Subscriber<sensor_msgs::Imu> (nh, "/imu/data", 1);
    //   gps_sub = new message_filters::Subscriber<sensor_msgs::NavSatFix> (nh, "/fix", 1);

    //   sync.reset(new Sync(MySyncPolicy(1000), *labeled_points_sub, *imu_sub, *gps_sub)); 
    //   sync->registerCallback(boost::bind(&LaserSlamWorker::scanCallback_ARGOS_Format, this, _1, _2, _3));
    // }

    // for double lidars
    // {
    //   scan_sub1 = new message_filters::Subscriber<sensor_msgs::PointCloud2> (nh, "/lidar1/velodyne_points", 1);
    //   scan_sub2 = new message_filters::Subscriber<sensor_msgs::PointCloud2> (nh, "/lidar2/velodyne_points", 1);
    //   gps_sub = new message_filters::Subscriber<sensor_msgs::NavSatFix> (nh, "/fix", 1);
    //   imu_sub = new message_filters::Subscriber<sensor_msgs::Imu> (nh, "/imu/data", 1);

    //   sr_format_double_lidar_sync.reset(new SR_Format_Double_Lidar_Sync(SR_Format_Double_Lidar_SyncPolicy(10), 
    //                           *scan_sub1, *scan_sub2, *gps_sub, *imu_sub));
    //   sr_format_double_lidar_sync->registerCallback(boost::bind(&LaserSlamWorker::scanCallback_ARGOS_Format_double_lidars, this, _1, _2, _3, _4)); 
    // }
  }
  else if (params_.IRAP_format)
  {
    scan_sub1 = new message_filters::Subscriber<sensor_msgs::PointCloud2> (nh, params_.assembled_cloud_sub_topic, 1);
    scan_sub2 = new message_filters::Subscriber<sensor_msgs::PointCloud2> (nh, params_.assembled_cloud_sub2_topic, 1);
    gps_sub = new message_filters::Subscriber<sensor_msgs::NavSatFix> (nh, "/gps/fix", 1);
    imu_sub_IRAP = new message_filters::Subscriber<laser_slam::imu> (nh, "/xsens_imu_data", 1);

    irap_format_sync.reset(new IRAP_Format_Sync(IRAP_Format_SyncPolicy(10), 
                            *scan_sub1, *scan_sub2, *gps_sub, *imu_sub_IRAP));
    irap_format_sync->registerCallback(boost::bind(&LaserSlamWorker::scanCallback_IRAP_Format, this, _1, _2, _3, _4));  
  }
  else
  {
    scan_sub_ = nh.subscribe(params_.assembled_cloud_sub_topic, kScanSubscriberMessageQueueSize,
                          &LaserSlamWorker::scanCallback, this);
  }

  // Setup publishers.
  trajectory_pub_ = nh.advertise<nav_msgs::Path>(params_.trajectory_pub_topic, kPublisherQueueSize, true);

  if (params_.publish_semantic_local_map) {
    semantic_local_map_pub_ = nh.advertise<sensor_msgs::PointCloud2>(params_.semantic_single_scan_pub_topic,
                                                            kPublisherQueueSize);
    if (params_.publish_semantic_full_map) {
      semantic_full_map_pub_ = nh.advertise<sensor_msgs::PointCloud2>(params_.semantic_full_map_pub_topic,
                                                              kPublisherQueueSize);
    }
  }

  // Setup services.
  get_laser_track_srv_ = nh.advertiseService(
      "get_laser_track",
      &LaserSlamWorker::getLaserTracksServiceCall, this);
  export_trajectory_srv_ = nh.advertiseService(
      "export_trajectory",
      &LaserSlamWorker::exportTrajectoryServiceCall, this);

  voxel_filter_.setLeafSize(params_.voxel_size_m, 
                            params_.voxel_size_m,
                            params_.voxel_size_m);
  voxel_filter_.setMinimumPointsNumberPerVoxel(params_.minimum_point_number_per_voxel);
  

  // Set the first world to odom transform.
  Eigen::Matrix<float, Eigen::Dynamic, Eigen::Dynamic> matrix;
  matrix.resize(4, 4);
  matrix = Eigen::Matrix<float, 4,4>::Identity();
  world_to_odom_ = PointMatcher_ros::eigenMatrixToStampedTransform<float>(
      matrix, params_.world_frame, params_.odom_frame, ros::Time::now());

  // TODO reactivate or rm.
  //  odometry_trajectory_pub_ = nh_.advertise<nav_msgs::Path>(params_.odometry_trajectory_pub_topic,
  //
  //  if (params_.publish_distant_map) {
  //    distant_map_pub_ = nh_.advertise<sensor_msgs::PointCloud2>(params_.distant_map_pub_topic,
  //                                                               kPublisherQueueSize);
  //  }
  //  if (params_.publish_full_map) {
  //    point_cloud_pub_ = nh_.advertise<sensor_msgs::PointCloud2>(params_.full_map_pub_topic,
  //                                                               kPublisherQueueSize);
  //  }
  //  new_fixed_cloud_pub_ = nh_.advertise<sensor_msgs::PointCloud2>("new_fixed_cloud",
  //                                                               kPublisherQueueSize);
}

void LaserSlamWorker::scanCallback(const sensor_msgs::PointCloud2& cloud_msg_in) {
  std::lock_guard<std::recursive_mutex> lock_scan_callback(scan_callback_mutex_);
  if (!lock_scan_callback_) {
    if (tf_listener_.waitForTransform(params_.odom_frame, params_.sensor_frame,
                                      cloud_msg_in.header.stamp, ros::Duration(kTimeout_s))) {
      // Get the tf transform.
      tf::StampedTransform tf_transform;
      tf_listener_.lookupTransform(params_.odom_frame, params_.sensor_frame,
                                   cloud_msg_in.header.stamp, tf_transform);

      bool process_scan = false;
      SE3 current_pose;

      if (!last_pose_set_) {
        process_scan = true;
        last_pose_set_ = true;
        last_pose_ = tfTransformToPose(tf_transform).T_w;
      } else {
        current_pose = tfTransformToPose(tf_transform).T_w;
        float dist_m = distanceBetweenTwoSE3(current_pose, last_pose_);
        if (dist_m > params_.minimum_distance_to_add_pose) {
          process_scan = true;
          last_pose_ = current_pose;
        }
      }

      if (process_scan) {
        // Convert input cloud to laser scan.
        LaserScan new_scan;
        new_scan.scan = PointMatcher_ros::rosMsgToPointMatcherCloud<float>(cloud_msg_in);
        new_scan.time_ns = rosTimeToCurveTime(cloud_msg_in.header.stamp.toNSec());

        // Process the new scan and get new values and factors.
        gtsam::NonlinearFactorGraph new_factors;
        gtsam::Values new_values;
        bool is_prior;
        if (params_.use_odometry_information) {
          laser_track_->processPoseAndLaserScan(tfTransformToPose(tf_transform), new_scan,
                                                &new_factors, &new_values, &is_prior);
        } 
        else {
          Pose new_pose;

          Time new_pose_time_ns = tfTransformToPose(tf_transform).time_ns;

          if (laser_track_->getNumScans() > 2u) {
            Pose current_pose = laser_track_->getCurrentPose();

            if (current_pose.time_ns > new_pose_time_ns - current_pose.time_ns) {
              Time previous_pose_time = current_pose.time_ns -
                  (new_pose_time_ns - current_pose.time_ns);
              if (previous_pose_time >= laser_track_->getMinTime() &&
                  previous_pose_time <= laser_track_->getMaxTime()) {
                SE3 previous_pose = laser_track_->evaluate(previous_pose_time);
                new_pose.T_w = last_pose_sent_to_laser_track_.T_w *
                    previous_pose.inverse()  * current_pose.T_w ;
                new_pose.T_w = SE3(SO3::fromApproximateRotationMatrix(
                    new_pose.T_w.getRotation().getRotationMatrix()), new_pose.T_w.getPosition());
              }
            }
          }

          new_pose.time_ns = new_pose_time_ns;
          laser_track_->processPoseAndLaserScan(new_pose, new_scan,
                                                &new_factors, &new_values, &is_prior);

          last_pose_sent_to_laser_track_ = new_pose;
        }

        // Process the new values and factors.
        gtsam::Values result;
        if (is_prior) {
          result = incremental_estimator_->registerPrior(new_factors, new_values, worker_id_);
        } else {
          result = incremental_estimator_->estimate(new_factors, new_values, new_scan.time_ns);
        }

        // Update the trajectory.
        laser_track_->updateFromGTSAMValues(result);

        // Adjust the correction between the world and odom frames.
        Pose current_pose = laser_track_->getCurrentPose();
        SE3 T_odom_sensor = tfTransformToPose(tf_transform).T_w;
        SE3 T_w_sensor = current_pose.T_w;
        SE3 T_w_odom = T_w_sensor * T_odom_sensor.inverse();

        Eigen::Matrix<float, Eigen::Dynamic, Eigen::Dynamic> matrix;

        // TODO resize needed?
        matrix.resize(4, 4);
        matrix = T_w_odom.getTransformationMatrix().cast<float>();

        {
          std::lock_guard<std::recursive_mutex> lock_world_to_odom(world_to_odom_mutex_);
          world_to_odom_ = PointMatcher_ros::eigenMatrixToStampedTransform<float>(
              matrix, params_.world_frame, params_.odom_frame, cloud_msg_in.header.stamp);
        }

        publishTrajectories();

        // Get the last cloud in world frame.
        DataPoints new_fixed_cloud;
        laser_track_->getLocalCloudInWorldFrame(laser_track_->getMaxTime(), &new_fixed_cloud);

        // Transform the cloud in sensor frame
        //TODO(Renaud) move to a transformPointCloud() fct.
        //      laser_slam::PointMatcher::TransformationParameters transformation_matrix =
        //          T_w_sensor.inverse().getTransformationMatrix().cast<float>();
        //
        //      laser_slam::correctTransformationMatrix(&transformation_matrix);
        //
        //      laser_slam::PointMatcher::Transformation* rigid_transformation =
        //          laser_slam::PointMatcher::get().REG(Transformation).create("RigidTransformation");
        //      CHECK_NOTNULL(rigid_transformation);
        //
        //      laser_slam::PointMatcher::DataPoints fixed_cloud_in_sensor_frame =
        //          rigid_transformation->compute(new_fixed_cloud,transformation_matrix);
        //
        //
        //      new_fixed_cloud_pub_.publish(
        //          PointMatcher_ros::pointMatcherCloudToRosMsg<float>(fixed_cloud_in_sensor_frame,
        //                                                             params_.sensor_frame,
        //                                                             cloud_msg_in.header.stamp));

        PointCloud new_fixed_cloud_pcl = lpmToPcl(new_fixed_cloud);

        if (params_.remove_ground_from_local_map) {
          const double robot_height_m = current_pose.T_w.getPosition()(2);
          PointCloud new_fixed_cloud_no_ground;
          for (size_t i = 0u; i < new_fixed_cloud_pcl.size(); ++i) {
            if (new_fixed_cloud_pcl.points[i].z > robot_height_m -
                params_.ground_distance_to_robot_center_m) {
              new_fixed_cloud_no_ground.push_back(new_fixed_cloud_pcl.points[i]);
            }
          }
          new_fixed_cloud_no_ground.width = 1;
          new_fixed_cloud_no_ground.height = new_fixed_cloud_no_ground.points.size();
          new_fixed_cloud_pcl = new_fixed_cloud_no_ground;
        }

        // Add the local scans to the full point cloud.
        if (params_.create_filtered_map) {
          if (new_fixed_cloud_pcl.size() > 0u) {
            std::lock_guard<std::recursive_mutex> lock(local_map_mutex_);
            if (local_map_.size() > 0u) {
              local_map_ += new_fixed_cloud_pcl;
            } else {
              local_map_ = new_fixed_cloud_pcl;
            }
            local_map_queue_.push_back(new_fixed_cloud_pcl);
          }
        }
      }
    } else {
      ROS_WARN_STREAM("[SegMapper] Timeout while waiting between " + params_.odom_frame  +
                      " and " + params_.sensor_frame  + ".");
    }
  }
} 

void LaserSlamWorker::scanCallback_ARGOS_Format(
  const laser_slam::LabeledPointCloud::ConstPtr& labeled_cloud_msg_in, 
  const sensor_msgs::Imu::ConstPtr& imu_msg_in,
  const sensor_msgs::NavSatFix::ConstPtr& gps_msg_in) 
{ 
  std::lock_guard<std::recursive_mutex> lock_scan_callback(scan_callback_mutex_);
  if (!lock_scan_callback_) {    
    const sensor_msgs::PointCloud2& cloud_msg_in = labeled_cloud_msg_in->point_cloud; 
    const std::vector<int> semantic_id = labeled_cloud_msg_in->semantic_id; 

    geometry_msgs::Quaternion orientation = imu_msg_in->orientation;

    // Use a Mercator projection to get the translation vector
    float lon = gps_msg_in->longitude;
    float lat = gps_msg_in->latitude;
    float alt = gps_msg_in->altitude;

    if (scale == 0)
      scale = cos(lat * M_PI / 180.0f);

    float tx = scale * er * lon * M_PI / 180.0f;
    float ty = scale * er * log(tan((90.0f + lat) * M_PI / 360.0f));
    float tz = alt;

    if(origin == Eigen::Vector3f(0,0,0))
      origin = Eigen::Vector3f(tx,ty,tz);

    tf::StampedTransform tf_transform(
      tf::Transform(tf::Quaternion(orientation.x,orientation.y,orientation.z,orientation.w+1e-10),
      tf::Vector3(tx-origin[0],ty-origin[1],tz-origin[2])), 
      cloud_msg_in.header.stamp, params_.world_frame, params_.odom_frame
    );

    bool process_scan = false;
    SE3 current_pose;

    if (!last_pose_set_) {
      process_scan = true;
      last_pose_set_ = true;
      last_pose_ = tfTransformToPose(tf_transform).T_w;
    } 
    else {
      current_pose = tfTransformToPose(tf_transform).T_w;
      float dist_m = distanceBetweenTwoSE3(current_pose, last_pose_);
      if (dist_m > params_.minimum_distance_to_add_pose) {
        process_scan = true;
        last_pose_ = current_pose;
      }
    }

    if (process_scan) {
      // Convert input cloud to laser scan.
      LaserScan new_scan;
      new_scan.scan = PointMatcher_ros::rosMsgToPointMatcherCloud<float>(cloud_msg_in, semantic_id);
      new_scan.time_ns = rosTimeToCurveTime(cloud_msg_in.header.stamp.toNSec());

      // Process the new scan and get new values and factors.
      gtsam::NonlinearFactorGraph new_factors;
      gtsam::Values new_values;
      bool is_prior;
      if (params_.use_odometry_information) {
        laser_track_->processPoseAndLaserScan(tfTransformToPose(tf_transform), new_scan,
                                              &new_factors, &new_values, &is_prior);
      }
      else {
        Pose new_pose;

        Time new_pose_time_ns = tfTransformToPose(tf_transform).time_ns;

        if (laser_track_->getNumScans() > 2u) {
          Pose current_pose = laser_track_->getCurrentPose();

          if (current_pose.time_ns > new_pose_time_ns - current_pose.time_ns) {
            Time previous_pose_time = current_pose.time_ns -
                (new_pose_time_ns - current_pose.time_ns);
            if (previous_pose_time >= laser_track_->getMinTime() &&
                previous_pose_time <= laser_track_->getMaxTime()) {
              SE3 previous_pose = laser_track_->evaluate(previous_pose_time);
              new_pose.T_w = last_pose_sent_to_laser_track_.T_w *
                  previous_pose.inverse()  * current_pose.T_w ;
              new_pose.T_w = SE3(SO3::fromApproximateRotationMatrix(
                  new_pose.T_w.getRotation().getRotationMatrix()), new_pose.T_w.getPosition());
            }
          }
        }

        new_pose.time_ns = new_pose_time_ns;
        laser_track_->processPoseAndLaserScan(new_pose, new_scan,
                                              &new_factors, &new_values, &is_prior);

        last_pose_sent_to_laser_track_ = new_pose;
      } 

      // Process the new values and factors.
      gtsam::Values result;
      if (is_prior) {
        result = incremental_estimator_->registerPrior(new_factors, new_values, worker_id_);
      } else {
        result = incremental_estimator_->estimate(new_factors, new_values, new_scan.time_ns);
      }

      // Update the trajectory.
      laser_track_->updateFromGTSAMValues(result);

      // Adjust the correction between the world and odom frames.
      Pose current_pose = laser_track_->getCurrentPose();
      SE3 T_odom_sensor = tfTransformToPose(tf_transform).T_w;
      SE3 T_w_sensor = current_pose.T_w;
      SE3 T_w_odom = T_w_sensor * T_odom_sensor.inverse();

      Eigen::Matrix<float, Eigen::Dynamic, Eigen::Dynamic> matrix;

      // TODO resize needed?
      matrix.resize(4, 4);
      matrix = T_w_odom.getTransformationMatrix().cast<float>();

      {
        std::lock_guard<std::recursive_mutex> lock_world_to_odom(world_to_odom_mutex_);
        world_to_odom_ = PointMatcher_ros::eigenMatrixToStampedTransform<float>(
            matrix, params_.world_frame, params_.odom_frame, cloud_msg_in.header.stamp);
      }

      publishTrajectories();
      publishSemanticMap();

      // Get the last cloud in world frame.
      DataPoints new_fixed_cloud;
      laser_track_->getLocalCloudInWorldFrame(laser_track_->getMaxTime(), &new_fixed_cloud);
      
      PointCloud new_fixed_cloud_pcl = lpmToPcl(new_fixed_cloud);

      // for the semantic local map
      if (params_.publish_semantic_local_map) {
        PointICloud new_fixed_icloud_pcl = lpmToPcl_with_semantic(new_fixed_cloud);
        if (semantic_local_map_.size() > 0u) {
          semantic_local_map_ += new_fixed_icloud_pcl;
        } else {
          semantic_local_map_ = new_fixed_icloud_pcl;
        }
        semantic_local_map_queue_.push_back(new_fixed_icloud_pcl);
      }

      if (params_.remove_ground_from_local_map) {
        const double robot_height_m = current_pose.T_w.getPosition()(2);
        PointCloud new_fixed_cloud_no_ground;
        for (size_t i = 0u; i < new_fixed_cloud_pcl.size(); ++i) {
          if (new_fixed_cloud_pcl.points[i].z > robot_height_m -
              params_.ground_distance_to_robot_center_m) {
            new_fixed_cloud_no_ground.push_back(new_fixed_cloud_pcl.points[i]);
          }
        }
        new_fixed_cloud_no_ground.width = 1;
        new_fixed_cloud_no_ground.height = new_fixed_cloud_no_ground.points.size();
        new_fixed_cloud_pcl = new_fixed_cloud_no_ground;
      }

      // Add the local scans to the full point cloud.
      if (params_.create_filtered_map) {
        if (new_fixed_cloud_pcl.size() > 0u) {
          std::lock_guard<std::recursive_mutex> lock(local_map_mutex_);
          if (local_map_.size() > 0u) {
            local_map_ += new_fixed_cloud_pcl;
          } else {
            local_map_ = new_fixed_cloud_pcl;
          }
          local_map_queue_.push_back(new_fixed_cloud_pcl);
        }
      }
    }
  }
}


void LaserSlamWorker::mergeLidarPointCloud_SR(const pcl::PointCloud<pcl::PointXYZ> laserCloudIn1, const pcl::PointCloud<pcl::PointXYZ> laserCloudIn2) 
{
  pcl::PointCloud<pcl::PointXYZ> transformed_laserCloudIn1;
  pcl::PointCloud<pcl::PointXYZ> transformed_laserCloudIn2;

  Eigen::Affine3f aff1 = Eigen::Affine3f::Identity();
  // aff1.rotate(Eigen::AngleAxisf(M_PI, Eigen::Vector3f(3,0,-3)));
  // aff1.translation() = Eigen::Vector3f(0.0,-0.375,1.76);

  Eigen::Affine3f aff2 = Eigen::Affine3f::Identity();
  // aff2.rotate(Eigen::AngleAxisf(M_PI, Eigen::Vector3f(-1,0,-3)));
  // aff2.translation() = Eigen::Vector3f(-0.1,0.375,1.76);

  pcl::transformPointCloud(laserCloudIn1, transformed_laserCloudIn1, aff1);
  pcl::transformPointCloud(laserCloudIn2, transformed_laserCloudIn2, aff2);

  pcl::PointCloud<pcl::PointXYZ> mergedCloud;
  mergedCloud = transformed_laserCloudIn1 + transformed_laserCloudIn2;

  pcl::toROSMsg(mergedCloud, merged_cloud_msg_in);
}

void LaserSlamWorker::scanCallback_ARGOS_Format_double_lidars(
    const sensor_msgs::PointCloud2::ConstPtr& laserCloudMsg1, 
    const sensor_msgs::PointCloud2::ConstPtr& laserCloudMsg2,
    const sensor_msgs::NavSatFix::ConstPtr& gps_msg_in,
    const sensor_msgs::Imu::ConstPtr& imu_msg_in) 
{ 
  // scan callback
  std::lock_guard<std::recursive_mutex> lock_scan_callback(scan_callback_mutex_);

  if (!lock_scan_callback_) {

    // merge lidar scans
    pcl::PointCloud<pcl::PointXYZ> laserCloudIn1;
    pcl::fromROSMsg(*laserCloudMsg1, laserCloudIn1);

    pcl::PointCloud<pcl::PointXYZ> laserCloudIn2;
    pcl::fromROSMsg(*laserCloudMsg2, laserCloudIn2);

    mergeLidarPointCloud_SR(laserCloudIn1,laserCloudIn2);
    
    geometry_msgs::Quaternion orientation = imu_msg_in->orientation;

    // Use a Mercator projection to get the translation vector
    float lon = gps_msg_in->longitude;
    float lat = gps_msg_in->latitude;
    float alt = gps_msg_in->altitude;

    if (scale == 0)
      scale = cos(lat * M_PI / 180.0f);

    float tx = scale * er * lon * M_PI / 180.0f;
    float ty = scale * er * log(tan((90.0f + lat) * M_PI / 360.0f));
    float tz = alt;

    if(origin == Eigen::Vector3f(0,0,0))
      origin = Eigen::Vector3f(tx,ty,tz);

    tf::StampedTransform tf_transform(
      tf::Transform(tf::Quaternion(orientation.x,orientation.y,orientation.z,orientation.w),
      tf::Vector3(tx-origin[0],ty-origin[1],tz-origin[2])), 
      merged_cloud_msg_in.header.stamp, params_.world_frame, params_.odom_frame
    ) ;

    bool process_scan = false;
    SE3 current_pose;

    if (!last_pose_set_) {
      process_scan = true;
      last_pose_set_ = true;
      last_pose_ = tfTransformToPose(tf_transform).T_w;
    } else {
      current_pose = tfTransformToPose(tf_transform).T_w;
      float dist_m = distanceBetweenTwoSE3(current_pose, last_pose_);
      if (dist_m > params_.minimum_distance_to_add_pose) {
        process_scan = true;
        last_pose_ = current_pose;
      }
    }

    if (process_scan) {
      // Convert input cloud to laser scan.
      LaserScan new_scan;
      new_scan.scan = PointMatcher_ros::rosMsgToPointMatcherCloud<float>(merged_cloud_msg_in);
      new_scan.time_ns = rosTimeToCurveTime(merged_cloud_msg_in.header.stamp.toNSec());

      // Process the new scan and get new values and factors.
      gtsam::NonlinearFactorGraph new_factors;
      gtsam::Values new_values;
      bool is_prior;
      if (params_.use_odometry_information) {
        laser_track_->processPoseAndLaserScan(tfTransformToPose(tf_transform), new_scan,
                                              &new_factors, &new_values, &is_prior);
      } 
      else {
        Pose new_pose;

        Time new_pose_time_ns = tfTransformToPose(tf_transform).time_ns;

        if (laser_track_->getNumScans() > 2u) {
          Pose current_pose = laser_track_->getCurrentPose();

          if (current_pose.time_ns > new_pose_time_ns - current_pose.time_ns) {
            Time previous_pose_time = current_pose.time_ns - (new_pose_time_ns - current_pose.time_ns);
            if (previous_pose_time >= laser_track_->getMinTime() &&
                previous_pose_time <= laser_track_->getMaxTime()) {
              SE3 previous_pose = laser_track_->evaluate(previous_pose_time);
              new_pose.T_w = last_pose_sent_to_laser_track_.T_w *
                  previous_pose.inverse()  * current_pose.T_w ;
              new_pose.T_w = SE3(SO3::fromApproximateRotationMatrix(
                  new_pose.T_w.getRotation().getRotationMatrix()), new_pose.T_w.getPosition());
            }
          }
        }

        new_pose.time_ns = new_pose_time_ns;
        laser_track_->processPoseAndLaserScan(new_pose, new_scan,
                                              &new_factors, &new_values, &is_prior);

        last_pose_sent_to_laser_track_ = new_pose;
      }

      // Process the new values and factors.
      gtsam::Values result;
      if (is_prior) {
        result = incremental_estimator_->registerPrior(new_factors, new_values, worker_id_);
      } else {
        result = incremental_estimator_->estimate(new_factors, new_values, new_scan.time_ns);
      }

      // Update the trajectory.
      laser_track_->updateFromGTSAMValues(result);

      // Adjust the correction between the world and odom frames.
      Pose current_pose = laser_track_->getCurrentPose();
      SE3 T_odom_sensor = tfTransformToPose(tf_transform).T_w;
      SE3 T_w_sensor = current_pose.T_w;
      SE3 T_w_odom = T_w_sensor * T_odom_sensor.inverse();

      Eigen::Matrix<float, Eigen::Dynamic, Eigen::Dynamic> matrix;

      // TODO resize needed?
      matrix.resize(4, 4);
      matrix = T_w_odom.getTransformationMatrix().cast<float>();

      {
        std::lock_guard<std::recursive_mutex> lock_world_to_odom(world_to_odom_mutex_);
        world_to_odom_ = PointMatcher_ros::eigenMatrixToStampedTransform<float>(
            matrix, params_.world_frame, params_.odom_frame, merged_cloud_msg_in.header.stamp);
      }

      publishTrajectories();

      // Get the last cloud in world frame.
      DataPoints new_fixed_cloud;
      laser_track_->getLocalCloudInWorldFrame(laser_track_->getMaxTime(), &new_fixed_cloud);

      // // Transform the cloud in sensor frame
      // TODO(Renaud) move to a transformPointCloud() fct.
      //      laser_slam::PointMatcher::TransformationParameters transformation_matrix =
      //          T_w_sensor.inverse().getTransformationMatrix().cast<float>();
      
      //      laser_slam::correctTransformationMatrix(&transformation_matrix);
      
      //      laser_slam::PointMatcher::Transformation* rigid_transformation =
      //          laser_slam::PointMatcher::get().REG(Transformation).create("RigidTransformation");
      //      CHECK_NOTNULL(rigid_transformation);
      
      //      laser_slam::PointMatcher::DataPoints fixed_cloud_in_sensor_frame =
      //          rigid_transformation->compute(new_fixed_cloud,transformation_matrix);
      
      
      //      new_fixed_cloud_pub_.publish(
      //          PointMatcher_ros::pointMatcherCloudToRosMsg<float>(fixed_cloud_in_sensor_frame,
      //                                                             params_.sensor_frame,
      //                                                             cloud_msg_in.header.stamp));

      PointCloud new_fixed_cloud_pcl = lpmToPcl(new_fixed_cloud);

      if (params_.remove_ground_from_local_map) {
        const double robot_height_m = current_pose.T_w.getPosition()(2);
        PointCloud new_fixed_cloud_no_ground;
        for (size_t i = 0u; i < new_fixed_cloud_pcl.size(); ++i) {
          if (new_fixed_cloud_pcl.points[i].z > robot_height_m - params_.ground_distance_to_robot_center_m) {
            new_fixed_cloud_no_ground.push_back(new_fixed_cloud_pcl.points[i]);
          }
        }
        new_fixed_cloud_no_ground.width = 1;
        new_fixed_cloud_no_ground.height = new_fixed_cloud_no_ground.points.size();
        new_fixed_cloud_pcl = new_fixed_cloud_no_ground;
      }

      // Add the local scans to the full point cloud.
      if (params_.create_filtered_map) {
        if (new_fixed_cloud_pcl.size() > 0u) {
          std::lock_guard<std::recursive_mutex> lock(local_map_mutex_);
          if (local_map_.size() > 0u) {
            local_map_ += new_fixed_cloud_pcl;
          } else {
            local_map_ = new_fixed_cloud_pcl;
          }
          local_map_queue_.push_back(new_fixed_cloud_pcl);
        }
      }
    }
  }
}

void LaserSlamWorker::mergeLidarPointCloud_KAIST(const pcl::PointCloud<pcl::PointXYZ> laserCloudIn1, const pcl::PointCloud<pcl::PointXYZ> laserCloudIn2) 
{
  pcl::PointCloud<pcl::PointXYZ> transformed_laserCloudIn1;
  pcl::PointCloud<pcl::PointXYZ> transformed_laserCloudIn2;

  Eigen::Matrix4f transform_right = Eigen::Matrix4f::Identity();
  transform_right(0,0) = -0.499043;
  transform_right(0,1) = 0.716822;
  transform_right(0,2) = -0.486952;
  transform_right(1,0) = -0.508527;
  transform_right(1,1) = -0.697242; 
  transform_right(1,2) = -0.505227;
  transform_right(2,0) = -0.701681;
  transform_right(2,1) = -0.00450156;
  transform_right(2,2) = 0.712477;;

  transform_right(0,3) = -0.298725;
  transform_right(1,3) = -0.422423;
  transform_right(2,3) = 1.95223;

  Eigen::Matrix4f transform_left = Eigen::Matrix4f::Identity();
  transform_left(0,0) = -0.522638;
  transform_left(0,1) = -0.689933;
  transform_left(0,2) = -0.500841;
  transform_left(1,0) = 0.480742;
  transform_left(1,1) = -0.723649;
  transform_left(1,2) = 0.495197;
  transform_left(2,0) = -0.704085;
  transform_left(2,1) = 0.0180335;
  transform_left(2,2) = 0.709886;

  transform_left(0,3) = -0.318732;
  transform_left(1,3) = 0.389231;
  transform_left(2,3) = 1.94661;

  pcl::transformPointCloud(laserCloudIn1, transformed_laserCloudIn1, transform_right);
  pcl::transformPointCloud(laserCloudIn2, transformed_laserCloudIn2, transform_left);
  
  pcl::PointCloud<pcl::PointXYZ> mergedCloud;
  mergedCloud = transformed_laserCloudIn1 + transformed_laserCloudIn2;

  pcl::toROSMsg(mergedCloud, merged_cloud_msg_in);
}


void LaserSlamWorker::scanCallback_IRAP_Format(
  const sensor_msgs::PointCloud2::ConstPtr& laserCloudMsg1, 
  const sensor_msgs::PointCloud2::ConstPtr& laserCloudMsg2,
  const sensor_msgs::NavSatFix::ConstPtr& gps_msg_in,
  const laser_slam::imu::ConstPtr& imu_msg_in
  )
{ 
  // scan callback
  std::lock_guard<std::recursive_mutex> lock_scan_callback(scan_callback_mutex_);

  if (!lock_scan_callback_) {

    // merge lidar scans
    pcl::PointCloud<pcl::PointXYZ> laserCloudIn1;
    pcl::fromROSMsg(*laserCloudMsg1, laserCloudIn1);

    pcl::PointCloud<pcl::PointXYZ> laserCloudIn2;
    pcl::fromROSMsg(*laserCloudMsg2, laserCloudIn2);

    mergeLidarPointCloud_KAIST(laserCloudIn1,laserCloudIn2);
    
    geometry_msgs::Quaternion orientation = imu_msg_in->quaternion_data;

    // Use a Mercator projection to get the translation vector
    float lon = gps_msg_in->longitude;
    float lat = gps_msg_in->latitude;
    float alt = gps_msg_in->altitude;

    if (scale == 0)
      scale = cos(lat * M_PI / 180.0f);

    float tx = scale * er * lon * M_PI / 180.0f;
    float ty = scale * er * log(tan((90.0f + lat) * M_PI / 360.0f));
    float tz = alt;

    if(origin == Eigen::Vector3f(0,0,0))
      origin = Eigen::Vector3f(tx,ty,tz);

    tf::StampedTransform tf_transform(
      tf::Transform(tf::Quaternion(orientation.x,orientation.y,orientation.z,orientation.w),
      tf::Vector3(tx-origin[0],ty-origin[1],tz-origin[2])), 
      merged_cloud_msg_in.header.stamp, params_.world_frame, params_.odom_frame
    ) ;

    bool process_scan = false;
    SE3 current_pose;

    if (!last_pose_set_) {
      process_scan = true;
      last_pose_set_ = true;
      last_pose_ = tfTransformToPose(tf_transform).T_w;
    } else {
      current_pose = tfTransformToPose(tf_transform).T_w;
      float dist_m = distanceBetweenTwoSE3(current_pose, last_pose_);
      if (dist_m > params_.minimum_distance_to_add_pose) {
        process_scan = true;
        last_pose_ = current_pose;
      }
    }

    if (process_scan) {
      // Convert input cloud to laser scan.
      LaserScan new_scan;
      new_scan.scan = PointMatcher_ros::rosMsgToPointMatcherCloud<float>(merged_cloud_msg_in);
      new_scan.time_ns = rosTimeToCurveTime(merged_cloud_msg_in.header.stamp.toNSec());

      // Process the new scan and get new values and factors.
      gtsam::NonlinearFactorGraph new_factors;
      gtsam::Values new_values;
      bool is_prior;
      if (params_.use_odometry_information) {
        laser_track_->processPoseAndLaserScan(tfTransformToPose(tf_transform), new_scan,
                                              &new_factors, &new_values, &is_prior);
      } 
      // else {
      //   Pose new_pose;

      //   Time new_pose_time_ns = tfTransformToPose(tf_transform).time_ns;

      //   if (laser_track_->getNumScans() > 2u) {
      //     Pose current_pose = laser_track_->getCurrentPose();

      //     if (current_pose.time_ns > new_pose_time_ns - current_pose.time_ns) {
      //       Time previous_pose_time = current_pose.time_ns - (new_pose_time_ns - current_pose.time_ns);
      //       if (previous_pose_time >= laser_track_->getMinTime() &&
      //           previous_pose_time <= laser_track_->getMaxTime()) {
      //         SE3 previous_pose = laser_track_->evaluate(previous_pose_time);
      //         new_pose.T_w = last_pose_sent_to_laser_track_.T_w *
      //             previous_pose.inverse()  * current_pose.T_w ;
      //         new_pose.T_w = SE3(SO3::fromApproximateRotationMatrix(
      //             new_pose.T_w.getRotation().getRotationMatrix()), new_pose.T_w.getPosition());
      //       }
      //     }
      //   }

      //   new_pose.time_ns = new_pose_time_ns;
      //   laser_track_->processPoseAndLaserScan(new_pose, new_scan,
      //                                         &new_factors, &new_values, &is_prior);

      //   last_pose_sent_to_laser_track_ = new_pose;
      // }

      // Process the new values and factors.
      gtsam::Values result;
      if (is_prior) {
        result = incremental_estimator_->registerPrior(new_factors, new_values, worker_id_);
      } else {
        result = incremental_estimator_->estimate(new_factors, new_values, new_scan.time_ns);
      }

      // Update the trajectory.
      laser_track_->updateFromGTSAMValues(result);

      // Adjust the correction between the world and odom frames.
      Pose current_pose = laser_track_->getCurrentPose();
      SE3 T_odom_sensor = tfTransformToPose(tf_transform).T_w;
      SE3 T_w_sensor = current_pose.T_w;
      SE3 T_w_odom = T_w_sensor * T_odom_sensor.inverse();

      Eigen::Matrix<float, Eigen::Dynamic, Eigen::Dynamic> matrix;

      // TODO resize needed?
      matrix.resize(4, 4);
      matrix = T_w_odom.getTransformationMatrix().cast<float>();

      {
        std::lock_guard<std::recursive_mutex> lock_world_to_odom(world_to_odom_mutex_);
        world_to_odom_ = PointMatcher_ros::eigenMatrixToStampedTransform<float>(
            matrix, params_.world_frame, params_.odom_frame, merged_cloud_msg_in.header.stamp);
      }

      publishTrajectories();

      // Get the last cloud in world frame.
      DataPoints new_fixed_cloud;
      laser_track_->getLocalCloudInWorldFrame(laser_track_->getMaxTime(), &new_fixed_cloud);

      // // Transform the cloud in sensor frame
      // TODO(Renaud) move to a transformPointCloud() fct.
      //      laser_slam::PointMatcher::TransformationParameters transformation_matrix =
      //          T_w_sensor.inverse().getTransformationMatrix().cast<float>();
      
      //      laser_slam::correctTransformationMatrix(&transformation_matrix);
      
      //      laser_slam::PointMatcher::Transformation* rigid_transformation =
      //          laser_slam::PointMatcher::get().REG(Transformation).create("RigidTransformation");
      //      CHECK_NOTNULL(rigid_transformation);
      
      //      laser_slam::PointMatcher::DataPoints fixed_cloud_in_sensor_frame =
      //          rigid_transformation->compute(new_fixed_cloud,transformation_matrix);
      
      
      //      new_fixed_cloud_pub_.publish(
      //          PointMatcher_ros::pointMatcherCloudToRosMsg<float>(fixed_cloud_in_sensor_frame,
      //                                                             params_.sensor_frame,
      //                                                             cloud_msg_in.header.stamp));

      PointCloud new_fixed_cloud_pcl = lpmToPcl(new_fixed_cloud);

      if (params_.remove_ground_from_local_map) {
        const double robot_height_m = current_pose.T_w.getPosition()(2);
        PointCloud new_fixed_cloud_no_ground;
        for (size_t i = 0u; i < new_fixed_cloud_pcl.size(); ++i) {
          if (new_fixed_cloud_pcl.points[i].z > robot_height_m - params_.ground_distance_to_robot_center_m) {
            new_fixed_cloud_no_ground.push_back(new_fixed_cloud_pcl.points[i]);
          }
        }
        new_fixed_cloud_no_ground.width = 1;
        new_fixed_cloud_no_ground.height = new_fixed_cloud_no_ground.points.size();
        new_fixed_cloud_pcl = new_fixed_cloud_no_ground;
      }

      // Add the local scans to the full point cloud.
      if (params_.create_filtered_map) {
        if (new_fixed_cloud_pcl.size() > 0u) {
          std::lock_guard<std::recursive_mutex> lock(local_map_mutex_);
          if (local_map_.size() > 0u) {
            local_map_ += new_fixed_cloud_pcl;
          } else {
            local_map_ = new_fixed_cloud_pcl;
          }
          local_map_queue_.push_back(new_fixed_cloud_pcl);
        }
      }
    }
  }
}

void LaserSlamWorker::scanCallback_VoxelNetFormat(const sensor_msgs::PointCloud2& cloud_msg_in)
{
  std::lock_guard<std::recursive_mutex> lock_scan_callback(scan_callback_mutex_);
  if (!lock_scan_callback_) {
    if (true) {
      // Get the tf transform.
      tf::StampedTransform tf_transform(tf::Transform(tf::Quaternion(0,0,0,1),tf::Vector3(0,0,0)),
                                        cloud_msg_in.header.stamp, "odom", "world");

      bool process_scan = true;
      SE3 current_pose;

      if (!last_pose_set_) {
        process_scan = true;
        last_pose_set_ = true;
        last_pose_ = tfTransformToPose(tf_transform).T_w;
      } else {
        current_pose = tfTransformToPose(tf_transform).T_w;
        float dist_m = distanceBetweenTwoSE3(current_pose, last_pose_);
        if (dist_m > params_.minimum_distance_to_add_pose) {
          process_scan = true;
          last_pose_ = current_pose;
        }
      }

      if (process_scan) {
        // Convert input cloud to laser scan.
        LaserScan new_scan;
        new_scan.scan = PointMatcher_ros::rosMsgToPointMatcherCloud<float>(cloud_msg_in);
        new_scan.time_ns = rosTimeToCurveTime(cloud_msg_in.header.stamp.toNSec());

        // Process the new scan and get new values and factors.
        gtsam::NonlinearFactorGraph new_factors;
        gtsam::Values new_values;
        bool is_prior;
        if (params_.use_odometry_information) {
          laser_track_->processPoseAndLaserScan(tfTransformToPose(tf_transform), new_scan,
                                                &new_factors, &new_values, &is_prior);
        } 
        else {
          Pose new_pose;

          Time new_pose_time_ns = tfTransformToPose(tf_transform).time_ns;

          if (laser_track_->getNumScans() > 2u) {
            Pose current_pose = laser_track_->getCurrentPose();

            if (current_pose.time_ns > new_pose_time_ns - current_pose.time_ns) {
              Time previous_pose_time = current_pose.time_ns -
                  (new_pose_time_ns - current_pose.time_ns);
              if (previous_pose_time >= laser_track_->getMinTime() &&
                  previous_pose_time <= laser_track_->getMaxTime()) {
                SE3 previous_pose = laser_track_->evaluate(previous_pose_time);
                new_pose.T_w = last_pose_sent_to_laser_track_.T_w *
                    previous_pose.inverse()  * current_pose.T_w ;
                new_pose.T_w = SE3(SO3::fromApproximateRotationMatrix(
                    new_pose.T_w.getRotation().getRotationMatrix()), new_pose.T_w.getPosition());
              }
            }
          }

          new_pose.time_ns = new_pose_time_ns;
          laser_track_->processPoseAndLaserScan(new_pose, new_scan,
                                                &new_factors, &new_values, &is_prior);

          last_pose_sent_to_laser_track_ = new_pose;
        }

        // Process the new values and factors.
        gtsam::Values result;
        if (is_prior) {
          result = incremental_estimator_->registerPrior(new_factors, new_values, worker_id_);
        } else {
          result = incremental_estimator_->estimate(new_factors, new_values, new_scan.time_ns);
        }

        // Update the trajectory.
        laser_track_->updateFromGTSAMValues(result);

        // Adjust the correction between the world and odom frames.
        Pose current_pose = laser_track_->getCurrentPose();
        SE3 T_odom_sensor = tfTransformToPose(tf_transform).T_w;
        SE3 T_w_sensor = current_pose.T_w;
        SE3 T_w_odom = T_w_sensor * T_odom_sensor.inverse();

        Eigen::Matrix<float, Eigen::Dynamic, Eigen::Dynamic> matrix;

        // TODO resize needed?
        matrix.resize(4, 4);
        matrix = T_w_odom.getTransformationMatrix().cast<float>();

        {
          std::lock_guard<std::recursive_mutex> lock_world_to_odom(world_to_odom_mutex_);
          world_to_odom_ = PointMatcher_ros::eigenMatrixToStampedTransform<float>(
              matrix, params_.world_frame, params_.odom_frame, cloud_msg_in.header.stamp);
        }

        publishTrajectories();

        // Get the last cloud in world frame.
        DataPoints new_fixed_cloud;
        laser_track_->getLocalCloudInWorldFrame(laser_track_->getMaxTime(), &new_fixed_cloud);

        // Transform the cloud in sensor frame
        //TODO(Renaud) move to a transformPointCloud() fct.
        //      laser_slam::PointMatcher::TransformationParameters transformation_matrix =
        //          T_w_sensor.inverse().getTransformationMatrix().cast<float>();
        //
        //      laser_slam::correctTransformationMatrix(&transformation_matrix);
        //
        //      laser_slam::PointMatcher::Transformation* rigid_transformation =
        //          laser_slam::PointMatcher::get().REG(Transformation).create("RigidTransformation");
        //      CHECK_NOTNULL(rigid_transformation);
        //
        //      laser_slam::PointMatcher::DataPoints fixed_cloud_in_sensor_frame =
        //          rigid_transformation->compute(new_fixed_cloud,transformation_matrix);
        //
        //
        //      new_fixed_cloud_pub_.publish(
        //          PointMatcher_ros::pointMatcherCloudToRosMsg<float>(fixed_cloud_in_sensor_frame,
        //                                                             params_.sensor_frame,
        //                                                             cloud_msg_in.header.stamp));

        PointCloud new_fixed_cloud_pcl = lpmToPcl(new_fixed_cloud);

        if (params_.remove_ground_from_local_map) {
          const double robot_height_m = current_pose.T_w.getPosition()(2);
          PointCloud new_fixed_cloud_no_ground;
          for (size_t i = 0u; i < new_fixed_cloud_pcl.size(); ++i) {
            if (new_fixed_cloud_pcl.points[i].z > robot_height_m -
                params_.ground_distance_to_robot_center_m) {
              new_fixed_cloud_no_ground.push_back(new_fixed_cloud_pcl.points[i]);
            }
          }
          new_fixed_cloud_no_ground.width = 1;
          new_fixed_cloud_no_ground.height = new_fixed_cloud_no_ground.points.size();
          new_fixed_cloud_pcl = new_fixed_cloud_no_ground;
        }

        // Add the local scans to the full point cloud.
        if (params_.create_filtered_map) {
          if (new_fixed_cloud_pcl.size() > 0u) {
            std::lock_guard<std::recursive_mutex> lock(local_map_mutex_);
            if (local_map_.size() > 0u) {
              local_map_ += new_fixed_cloud_pcl;
            } else {
              local_map_ = new_fixed_cloud_pcl;
            }
            local_map_queue_.push_back(new_fixed_cloud_pcl);
          }
        }
      }
    } else {
      ROS_WARN_STREAM("[SegMapper] Timeout while waiting between " + params_.odom_frame  +
                      " and " + params_.sensor_frame  + ".");
    }
  }
}

void LaserSlamWorker::setLockScanCallback(bool new_state) {
  std::lock_guard<std::recursive_mutex> lock(scan_callback_mutex_);
  lock_scan_callback_ = new_state;
}

bool LaserSlamWorker::getLaserTracksServiceCall(
    laser_slam_ros::GetLaserTrackSrv::Request& request,
    laser_slam_ros::GetLaserTrackSrv::Response& response) {
  std::vector<std::shared_ptr<LaserTrack> > laser_tracks =
      incremental_estimator_->getAllLaserTracks();
  Trajectory trajectory;
  ros::Time scan_stamp;
  tf::StampedTransform tf_transform;
  geometry_msgs::TransformStamped ros_transform;
  std::vector<std::tuple<laser_slam::Time, sensor_msgs::PointCloud2, geometry_msgs::TransformStamped> > data;
  for (const auto& track: laser_tracks) {
    track->getTrajectory(&trajectory);
    for (const auto& scan: track->getLaserScans()) {
      // Get data.
      scan_stamp.fromNSec(curveTimeToRosTime(scan.time_ns));

      sensor_msgs::PointCloud2 pc = PointMatcher_ros::pointMatcherCloudToRosMsg<float>(
	scan.scan, params_.sensor_frame, scan_stamp);

      tf_transform = PointMatcher_ros::eigenMatrixToStampedTransform<float>(
          trajectory.at(scan.time_ns).getTransformationMatrix().cast<float>(),
          params_.world_frame,
          params_.sensor_frame,
          scan_stamp);
      tf::transformStampedTFToMsg(tf_transform, ros_transform);
      
      data.push_back(std::make_tuple(scan.time_ns, pc, ros_transform));
    }
  }
  
  std::sort(data.begin(),data.end(),
       [](const std::tuple<laser_slam::Time, sensor_msgs::PointCloud2, geometry_msgs::TransformStamped>& a,
       const std::tuple<laser_slam::Time, sensor_msgs::PointCloud2, geometry_msgs::TransformStamped>& b) -> bool
       {
         return std::get<0>(a) <= std::get<0>(b);
       });
  
  bool zero_added = false;
  // Fill response.
  for (const auto& elem : data) {
    laser_slam::Time time;
    sensor_msgs::PointCloud2 pc;
    geometry_msgs::TransformStamped tf;
    std::tie(time, pc, tf) = elem;
    LOG(INFO) << "Time " << time;
    if (time == 0u) {
      if (!zero_added) {
	zero_added = true;
      } else {
	continue;
      }
    } 
    response.laser_scans.push_back(pc);
    response.transforms.push_back(tf);
  }

  return true;
}

void LaserSlamWorker::publishTrajectory(const Trajectory& trajectory,
                                        const ros::Publisher& publisher) const {
  nav_msgs::Path traj_msg;
  traj_msg.header.frame_id = params_.world_frame;
  Time traj_time = curveTimeToRosTime(trajectory.rbegin()->first);
  traj_msg.header.stamp.fromNSec(traj_time);

  for (const auto& timePose : trajectory) {
    geometry_msgs::PoseStamped pose_msg;
    pose_msg.header = traj_msg.header;
    pose_msg.header.stamp.fromNSec(curveTimeToRosTime(timePose.first));

    //TODO functionize
    pose_msg.pose.position.x = timePose.second.getPosition().x();
    pose_msg.pose.position.y = timePose.second.getPosition().y();
    pose_msg.pose.position.z = timePose.second.getPosition().z();
    pose_msg.pose.orientation.w = timePose.second.getRotation().w();
    pose_msg.pose.orientation.x = timePose.second.getRotation().x();
    pose_msg.pose.orientation.y = timePose.second.getRotation().y();
    pose_msg.pose.orientation.z = timePose.second.getRotation().z();
    traj_msg.poses.push_back(pose_msg);
  }
  publisher.publish(traj_msg);
}

void LaserSlamWorker::publishMap() {
  // TODO make thread safe.
  if (local_map_.size() > 0) {
    PointCloud filtered_map;
    getFilteredMap(&filtered_map);

    //maximumNumberPointsFilter(&filtered_map);
    //    if (params_.publish_full_map) {
    //      sensor_msgs::PointCloud2 msg;
    //      convert_to_point_cloud_2_msg(filtered_map, params_.world_frame, &msg);
    //      point_cloud_pub_.publish(msg);
    //    }
    if (params_.publish_local_map) {
      sensor_msgs::PointCloud2 msg;
      convert_to_point_cloud_2_msg(filtered_map, params_.world_frame, &msg);
      local_map_pub_.publish(msg);
    }
    //    if (params_.publish_distant_map) {
    //      sensor_msgs::PointCloud2 msg;
    //      convert_to_point_cloud_2_msg(distant_map_, params_.world_frame, &msg);
    //      distant_map_pub_.publish(msg);
    //    }
  }
}

void LaserSlamWorker::publishSemanticMap(bool loop_closure_detected) {
  // TODO make thread safe.
  if (semantic_local_map_.size() > 0) {
    if (params_.publish_semantic_local_map) {
      PointICloud filtered_map;
      getFilteredSemanticMap(&filtered_map, loop_closure_detected);

      sensor_msgs::PointCloud2 local_map_msg;
      convert_to_point_cloud_2_msg(filtered_map, params_.world_frame, &local_map_msg);
      semantic_local_map_pub_.publish(local_map_msg);

      if (params_.publish_semantic_full_map) {
        sensor_msgs::PointCloud2 full_map_msg;
        convert_to_point_cloud_2_msg(semantic_full_map_, params_.world_frame, &full_map_msg);
        semantic_full_map_pub_.publish(full_map_msg);
      }   
    }     
  }
}

void LaserSlamWorker::publishTrajectories() {
  Trajectory trajectory;
  laser_track_->getTrajectory(&trajectory);
  if (trajectory.size() > 0u) publishTrajectory(trajectory, trajectory_pub_);
}

// TODO can we move?
Pose LaserSlamWorker::tfTransformToPose(const tf::StampedTransform& tf_transform) {
  // Register new pose.
  Pose pose;
  SE3::Position pos(tf_transform.getOrigin().getX(), tf_transform.getOrigin().getY(),
                    tf_transform.getOrigin().getZ());
  SE3::Rotation::Implementation rot(tf_transform.getRotation().getW(),
                                    tf_transform.getRotation().getX(),
                                    tf_transform.getRotation().getY(),
                                    tf_transform.getRotation().getZ());
  pose.T_w = SE3(pos, rot);
  pose.time_ns = rosTimeToCurveTime(tf_transform.stamp_.toNSec());

  return pose;
}

Time LaserSlamWorker::rosTimeToCurveTime(const Time& timestamp_ns) {
  if (!base_time_set_) {
    base_time_ns_ = timestamp_ns;
    base_time_set_ = true;
  }
  return timestamp_ns - base_time_ns_;
}

Time LaserSlamWorker::curveTimeToRosTime(const Time& timestamp_ns) const {
  CHECK(base_time_set_);
  return timestamp_ns + base_time_ns_;
}

std::vector<laser_slam_ros::PointCloud> LaserSlamWorker::getQueuedPoints() {
  std::lock_guard<std::recursive_mutex> lock(local_map_mutex_);
  std::vector<laser_slam_ros::PointCloud> new_points;
  new_points.swap(local_map_queue_);

  return new_points;
}

std::vector<laser_slam_ros::PointICloud> LaserSlamWorker::getSemanticQueuedPoints() {
  std::lock_guard<std::recursive_mutex> lock(local_map_mutex_);
  std::vector<laser_slam_ros::PointICloud> new_semantic_points;
  new_semantic_points.swap(semantic_local_map_queue_);

  return new_semantic_points;
}

// TODO one shot of cleaning.
void LaserSlamWorker::getFilteredMap(PointCloud* filtered_map) {
  laser_slam::Pose current_pose = laser_track_->getCurrentPose();

  PclPoint current_position;
  current_position.x = current_pose.T_w.getPosition()[0];
  current_position.y = current_pose.T_w.getPosition()[1];
  current_position.z = current_pose.T_w.getPosition()[2];

  // Apply the cylindrical filter on the local map and get a copy.
  PointCloud local_map;
  {
    std::lock_guard<std::recursive_mutex> lock(local_map_mutex_);
    local_map = local_map_;
    applyCylindricalFilter(current_position, params_.distance_to_consider_fixed, 40, false, &local_map_);
  }

  *filtered_map = local_map;

  // // Apply a voxel filter.
  // laser_slam::Clock clock;

  // PointCloudPtr local_map_ptr(new PointCloud());
  // pcl::copyPointCloud<PclPoint, PclPoint>(local_map, *local_map_ptr);

  // PointCloud local_map_filtered;

  // voxel_filter_.setInputCloud(local_map_ptr);
  // voxel_filter_.filter(local_map_filtered);

  // clock.takeTime();

  // if (params_.separate_distant_map) {
  //   // If separating the map is enabled, the distance between each point in the local_map_ will
  //   // be compared to the current robot position. Points which are far from the robot will
  //   // be transfered to the distant_map_. This is helpful for publishing (points in distant_map_
  //   // need to be filtered only once) and for any other processing which needs to be done only
  //   // when a map is distant from robot and can be assumed as static (until loop closure).

  //   // TODO(renaud) Is there a way to separate the cloud without having to transform in sensor
  //   // frame by setting the position to compute distance from?
  //   // Transform local_map_ in sensor frame.
  //   clock.start();

  //   // Save before removing points.
  //   PointCloud new_distant_map = local_map_filtered;

  //   applyCylindricalFilter(current_position, params_.distance_to_consider_fixed,
  //                          40, false, &local_map_filtered);

  //   applyCylindricalFilter(current_position, params_.distance_to_consider_fixed,
  //                          40, true, &new_distant_map);
  //   {
  //     std::lock_guard<std::recursive_mutex> lock(local_map_filtered_mutex_);
  //     local_map_filtered_ = local_map_filtered;
  //   }

  //   // Add the new_distant_map to the distant_map_.
  //   // TODO add lock if used
  //   if (distant_map_.size() > 0u) {
  //     distant_map_ += new_distant_map;
  //   } else {
  //     distant_map_ = new_distant_map;
  //   }

  //   *filtered_map = local_map_filtered;
  //   *filtered_map += distant_map_;

  //   clock.takeTime();
  //   // LOG(INFO) << "new_local_map.size() " << local_map.size();
  //   // LOG(INFO) << "new_distant_map.size() " << new_distant_map.size();
  //   // LOG(INFO) << "distant_map_.size() " << distant_map_.size();
  //   // LOG(INFO) << "Separating done! Took " << clock.getRealTime() << " ms.";
  // } else {
  //   *filtered_map = local_map;
  // }
}

void LaserSlamWorker::getFilteredSemanticMap(PointICloud* filtered_map, bool loop_closure_detected) {
  laser_slam::Pose current_pose = laser_track_->getCurrentPose();

  PointI current_position;
  current_position.x = current_pose.T_w.getPosition()[0];
  current_position.y = current_pose.T_w.getPosition()[1];
  current_position.z = current_pose.T_w.getPosition()[2];

  // Apply the cylindrical filter on the local map and get a copy.
  PointICloud semantic_local_map;
  semantic_local_map = semantic_local_map_;
  applyCylindricalFilter(current_position, params_.distance_to_consider_fixed, 40, false, &semantic_local_map_);

  if (params_.publish_semantic_full_map)
  {
    if (!loop_closure_detected)
      semantic_full_map_ += semantic_local_map;
    else 
    {
      semantic_full_map_.clear();
      std::vector<LaserScan> laser_scans_ = laser_track_->getLaserScans();
      for (int i = 0 ; i < laser_scans_.size() ; i+=20)
      {
        const DataPoints scan = laser_scans_[i].scan;
        sensor_msgs::PointCloud2 pc = PointMatcher_ros::pointMatcherCloudToRosMsg<float>(scan, "map", ros::Time());
        pc.fields[3].name = "intensity";
        PointICloud temp_scan, transformed_temp_scan;
        convert_to_pcl_point_cloud(pc, &temp_scan);

        SE3 transform = laser_track_->evaluate(laser_scans_[i].time_ns);
        pcl::transformPointCloud(temp_scan, transformed_temp_scan, transform.getTransformationMatrix());

        semantic_full_map_ += transformed_temp_scan;
      }
    }

    PointICloudPtr semantic_full_map_ptr(new PointICloud());
    pcl::copyPointCloud<PointI, PointI>(semantic_full_map_, *semantic_full_map_ptr);
    voxel_filter_with_semantic_.setInputCloud(semantic_full_map_ptr);

    float voxel_size = 1.0;
    voxel_filter_with_semantic_.setLeafSize(voxel_size, voxel_size,voxel_size);

    PointICloud semantic_full_map_filtered;
    voxel_filter_with_semantic_.filter(semantic_full_map_filtered);

    pcl::copyPointCloud<PointI, PointI>(semantic_full_map_filtered, semantic_full_map_);
  }

  *filtered_map = semantic_local_map;
}

void LaserSlamWorker::getLocalMapFiltered(PointCloud* local_map_filtered) {
  CHECK_NOTNULL(local_map_filtered);
  std::lock_guard<std::recursive_mutex> lock(local_map_filtered_mutex_);
  *local_map_filtered = local_map_filtered_;
}

void LaserSlamWorker::clearLocalMap() {
  {
    std::lock_guard<std::recursive_mutex> lock(local_map_mutex_);
    local_map_.clear();
  }
 
  {
    std::lock_guard<std::recursive_mutex> lock(local_map_filtered_mutex_);
    local_map_filtered_.clear();
  }
}

tf::StampedTransform LaserSlamWorker::getWorldToOdom() {
  std::lock_guard<std::recursive_mutex> lock_world_to_odom(world_to_odom_mutex_);
  tf::StampedTransform world_to_odom = world_to_odom_;
  return world_to_odom;
}

void LaserSlamWorker::getTrajectory(Trajectory* out_trajectory) const {
  laser_track_->getTrajectory(out_trajectory);
}

void LaserSlamWorker::getOdometryTrajectory(Trajectory* out_trajectory) const {
  laser_track_->getOdometryTrajectory(out_trajectory);
}

void LaserSlamWorker::updateLocalMap(const SE3& last_pose_before_update,
                                     const laser_slam::Time last_pose_before_update_timestamp_ns) {

  Trajectory new_trajectory;
  laser_track_->getTrajectory(&new_trajectory);

  SE3 new_last_pose = new_trajectory.at(last_pose_before_update_timestamp_ns);

  const Eigen::Matrix4f transform_matrix = (new_last_pose * last_pose_before_update.inverse()).
      getTransformationMatrix().cast<float>();
  {
    std::lock_guard<std::recursive_mutex> lock(local_map_mutex_);
    pcl::transformPointCloud(local_map_, local_map_, transform_matrix);
  }
  {
    std::lock_guard<std::recursive_mutex> lock(local_map_filtered_mutex_);
    pcl::transformPointCloud(local_map_filtered_, local_map_filtered_, transform_matrix);
  }
}

laser_slam::SE3 LaserSlamWorker::getTransformBetweenPoses(
    const laser_slam::SE3& start_pose, const laser_slam::Time end_pose_timestamp_ns) const {
  Trajectory new_trajectory;
  laser_track_->getTrajectory(&new_trajectory);

  SE3 last_pose = new_trajectory.at(end_pose_timestamp_ns);
  return last_pose * start_pose.inverse();
}

void LaserSlamWorker::exportTrajectories() const {
  Trajectory traj;
  laser_track_->getTrajectory(&traj);
  Eigen::MatrixXd matrix;
  matrix.resize(traj.size(), 4);
  unsigned int i = 0u;
  for (const auto& pose : traj) {
    matrix(i,0) = pose.first;
    matrix(i,1) = pose.second.getPosition().x();
    matrix(i,2) = pose.second.getPosition().y();
    matrix(i,3) = pose.second.getPosition().z();
    ++i;
  }
  writeEigenMatrixXdCSV(matrix, "/tmp/trajectory_" + std::to_string(worker_id_) + ".csv");
}

void LaserSlamWorker::exportTrajectoryHead(laser_slam::Time head_duration_ns,
                                           const std::string& filename) const {
  Eigen::MatrixXd matrix;
  Trajectory traj;
  laser_track_->getTrajectory(&traj);
  CHECK_GE(traj.size(), 1u);
  matrix.resize(traj.size(), 4);

  const Time traj_end_ns = traj.rbegin()->first;
  Time head_start_ns;
  if (traj_end_ns > head_duration_ns) {
    head_start_ns = traj_end_ns - head_duration_ns;
  } else {
    head_start_ns = 0u;
  }

  unsigned int i = 0u;
  for (const auto& pose : traj) {
    if (pose.first > head_start_ns) {
      matrix(i,0) = pose.first;
      matrix(i,1) = pose.second.getPosition().x();
      matrix(i,2) = pose.second.getPosition().y();
      matrix(i,3) = pose.second.getPosition().z();
      ++i;
     }
  }
  matrix.conservativeResize(i, 4);
  writeEigenMatrixXdCSV(matrix, filename);
  LOG(INFO) << "Exported " << i << " trajectory poses to " << filename << ".";
}

void LaserSlamWorker::exportTrajectory_KITTI(const std::string& filename) const {
  
  laser_slam::Time head_duration_ns = laser_track_->getMaxTime();

  Eigen::MatrixXd matrix;
  Trajectory traj;
  laser_track_->getTrajectory(&traj);
  CHECK_GE(traj.size(), 1u);
  matrix.resize(traj.size(), 9);

  const Time traj_end_ns = traj.rbegin()->first;
  Time head_start_ns;
  if (traj_end_ns > head_duration_ns) {
    head_start_ns = traj_end_ns - head_duration_ns;
  } else {
    head_start_ns = 0u;
  }

  unsigned int i = 0u;
  for (const auto& pose : traj) {
    if (pose.first > head_start_ns) {

      matrix(i,0) = pose.first*1e-9;
      matrix(i,1) = pose.second.getPosition().x();
      matrix(i,2) = pose.second.getPosition().y();
      matrix(i,3) = pose.second.getPosition().z();
      matrix(i,4) = pose.second.getRotation().x();
      matrix(i,5) = pose.second.getRotation().y();
      matrix(i,6) = pose.second.getRotation().z();
      matrix(i,7) = pose.second.getRotation().w();
      ++i;
     }
  }
  matrix.conservativeResize(i, 8);
  writeEigenMatrixXdCSV(matrix, filename);
  LOG(INFO) << "Exported " << i << " trajectory poses to " << filename << ".";
}

bool LaserSlamWorker::exportTrajectoryServiceCall(std_srvs::Empty::Request& req,
                                                  std_srvs::Empty::Response& res) {
  exportTrajectory_KITTI("/tmp/trajectory.csv");
  return true;
}

} // namespace laser_slam_ros
