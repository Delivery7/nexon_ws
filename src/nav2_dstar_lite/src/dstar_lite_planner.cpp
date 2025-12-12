#include "nav2_dstar_lite/dstar_lite_planner.hpp"

namespace nav2_dstar_lite
{

    void DStarLitePlanner::configure(
        const rclcpp_lifecycle::LifecycleNode::WeakPtr &parent,
        std::string name,
        const std::shared_ptr<tf2_ros::Buffer> &tf,
        const std::shared_ptr<nav2_costmap_2d::Costmap2DROS> &costmap_ros)
    {
        name_ = name;
        parent_node_ = parent;
        tf_ = tf;
        costmap_ros_ = costmap_ros;

        auto node = parent_node_.lock();
        RCLCPP_INFO(node->get_logger(), "D* Lite Planner configured");
    }

    void DStarLitePlanner::cleanup()
    {
        costmap_ros_.reset();
    }

    void DStarLitePlanner::activate()
    {
        RCLCPP_INFO(
            node_->get_logger(), "Activating plugin %s of type Dstar Planner",
            name_.c_str());
    }
    void DStarLitePlanner::deactivate()
    {
        RCLCPP_INFO(
            node_->get_logger(), "Deactivating plugin %s of type Dstar Planner",
            name_.c_str());
    }

    nav_msgs::msg::Path DStarLitePlanner::createPlan(
        const geometry_msgs::msg::PoseStamped &start,
        const geometry_msgs::msg::PoseStamped &goal)
    {
        nav_msgs::msg::Path path;
        path.header.frame_id = costmap_ros_->getGlobalFrameID();
        path.header.stamp = rclcpp::Clock().now();

        // TODO: Implement real D* Lite logic
        // Dummy path for now

        path.poses.push_back(start);
        path.poses.push_back(goal);

        return path;
    }

} // namespace nav2_dstar_lite
