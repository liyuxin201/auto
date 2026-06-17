#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>

#include <Eigen/Eigen>
#include <algorithm>
#include <cmath>
#include <iostream>
#include <nav_msgs/Path.h>
#include <sensor_msgs/Imu.h>
#include <std_msgs/Empty.h>
#include <vector>
#include <visualization_msgs/Marker.h>

#include <bspline_opt/bspline_optimizer.h>
#include <geometry_msgs/PoseStamped.h>
#include <plan_env/grid_map.h>
#include <plan_manage/planner_manager.h>
#include <traj_utils/Bspline.h>
#include <traj_utils/DataDisp.h>
#include <traj_utils/MultiBsplines.h>
#include <traj_utils/planning_visualization.h>

#define private public
#include <plan_manage/ego_replan_fsm.h>
#undef private

namespace {

class ExternalMapPlannerNode {
public:
  void init(ros::NodeHandle& nh) {
    fsm_.init(nh);

    // Hold the original FSM until an external inflated occupancy map arrives.
    fsm_.exec_timer_.stop();
    fsm_.safety_timer_.stop();

    external_map_sub_ = nh.subscribe(
        "external_map/occupancy_inflate", 1,
        &ExternalMapPlannerNode::externalMapCallback, this);

    ROS_WARN("external_map_planner_node is waiting for ~external_map/occupancy_inflate");
  }

private:
  void externalMapCallback(const sensor_msgs::PointCloud2ConstPtr& msg) {
    auto map = fsm_.planner_manager_->grid_map_;
    if (!map) {
      ROS_ERROR_THROTTLE(1.0, "Planner grid map is not initialized.");
      return;
    }

    if (have_last_bbox_) {
      map->resetBuffer(last_bbox_min_, last_bbox_max_);
      have_last_bbox_ = false;
    }

    pcl::PointCloud<pcl::PointXYZ> cloud;
    pcl::fromROSMsg(*msg, cloud);

    bool have_valid_point = false;
    Eigen::Vector3d bbox_min = Eigen::Vector3d::Zero();
    Eigen::Vector3d bbox_max = Eigen::Vector3d::Zero();

    for (const auto& pt : cloud.points) {
      if (!std::isfinite(pt.x) || !std::isfinite(pt.y) || !std::isfinite(pt.z)) {
        continue;
      }

      Eigen::Vector3d pos(pt.x, pt.y, pt.z);
      map->setOccupied(pos);

      if (!have_valid_point) {
        bbox_min = pos;
        bbox_max = pos;
        have_valid_point = true;
      } else {
        bbox_min = bbox_min.cwiseMin(pos);
        bbox_max = bbox_max.cwiseMax(pos);
      }
    }

    if (have_valid_point) {
      last_bbox_min_ = bbox_min;
      last_bbox_max_ = bbox_max;
      have_last_bbox_ = true;
    }

    if (!planner_started_) {
      planner_started_ = true;
      fsm_.exec_timer_.start();
      fsm_.safety_timer_.start();
      ROS_INFO("External occupancy map received, planner FSM started.");
    }
  }

  ego_planner::EGOReplanFSM fsm_;
  ros::Subscriber external_map_sub_;
  bool planner_started_{false};
  bool have_last_bbox_{false};
  Eigen::Vector3d last_bbox_min_{Eigen::Vector3d::Zero()};
  Eigen::Vector3d last_bbox_max_{Eigen::Vector3d::Zero()};
};

}  // namespace

int main(int argc, char** argv) {
  ros::init(argc, argv, "external_map_planner_node");
  ros::NodeHandle nh("~");

  ExternalMapPlannerNode node;
  node.init(nh);

  ros::spin();
  return 0;
}
