#ifndef MANIPULATION__MTCNODE_HPP
#define MANIPULATION__MTCNODE_HPP


#include "rclcpp_action/rclcpp_action.hpp"
#include <rclcpp/rclcpp.hpp>
#include <moveit/planning_scene/planning_scene.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>
#include <moveit/task_constructor/task.h>
#include <moveit/task_constructor/solvers.h>
#include <moveit/task_constructor/stages.h>
#if __has_include(<tf2_geometry_msgs/tf2_geometry_msgs.hpp>)
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#else
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#endif
#if __has_include(<tf2_eigen/tf2_eigen.hpp>)
#include <tf2_eigen/tf2_eigen.hpp>
#else
#include <tf2_eigen/tf2_eigen.h>
#endif

namespace manipulation
{



class MTCNode
{
public:    
    MTCNode(const rclcpp::NodeOptions& options,
            std::string group,
            std::string goal);

    rclcpp::node_interfaces::NodeBaseInterface::SharedPtr get_node_base_interface();

    void do_task();
    void setup_planning_scene();

private:
    moveit::task_constructor::Task create_task();

    moveit::task_constructor::Task task_;

    rclcpp::Node::SharedPtr node_;

    std::string group_, goal_;
};

} // end namespace manipulation

#endif // MANIPULATION__MTCNODE_HPP
