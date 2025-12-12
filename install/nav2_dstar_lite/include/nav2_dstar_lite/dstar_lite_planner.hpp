#ifndef NAV2_DSTAR_LITE_PLANNER_HPP_
#define NAV2_DSTAR_LITE_PLANNER_HPP_

#include <memory>
#include <string>

#include "rclcpp/rclcpp.hpp"
#include "nav2_core/global_planner.hpp"
#include "nav2_costmap_2d/costmap_2d_ros.hpp"
#include "nav2_util/robot_utils.hpp"
#include "nav2_util/lifecycle_node.hpp"
#include "nav_msgs/msg/path.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "tf2_ros/buffer.h"

namespace nav2_dstar_lite
{

  class DStarLitePlanner : public nav2_core::GlobalPlanner
  {
  public:
    DStarLitePlanner() = default;
    ~DStarLitePlanner() override = default;

    void configure(
        const rclcpp_lifecycle::LifecycleNode::WeakPtr &parent,
        std::string name,
        const std::shared_ptr<tf2_ros::Buffer> &tf,
        const std::shared_ptr<nav2_costmap_2d::Costmap2DROS> &costmap_ros);

    void cleanup() override;
    void activate() override;
    void deactivate() override;

    nav_msgs::msg::Path createPlan(
        const geometry_msgs::msg::PoseStamped &start,
        const geometry_msgs::msg::PoseStamped &goal) override;

  private:
    rclcpp_lifecycle::LifecycleNode::WeakPtr parent_node_;
    std::shared_ptr<nav2_costmap_2d::Costmap2DROS> costmap_ros_;
    nav2_util::LifecycleNode::SharedPtr node_;
    std::shared_ptr<tf2_ros::Buffer> tf_;
    std::string global_frame_, name_;
  };

} // namespace nav2_dstar_lite

#endif
