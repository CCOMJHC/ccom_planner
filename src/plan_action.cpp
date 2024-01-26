#include <ccom_planner/plan_action.h>

#include <geometry_msgs/PoseStamped.h>
#include <project11_nav_msgs/RobotState.h>
#include <tf2_ros/buffer.h>
#include <project11_navigation/robot_capabilities.h>
#include <project11_navigation/context.h>
#include <ccom_planner/dubins_astar.h>

BT_REGISTER_NODES(factory)
{
  factory.registerNodeType<ccom_planner::PlanAction>("CCOMPlanner");
}

namespace ccom_planner
{

PlanAction::PlanAction(const std::string& name, const BT::NodeConfig& config):
  BT::StatefulActionNode(name, config)
{

}

BT::PortsList PlanAction::providedPorts()
{
  return {
    BT::InputPort<geometry_msgs::PoseStamped>("start_pose"),
    BT::InputPort<geometry_msgs::PoseStamped>("goal_pose"),
    BT::OutputPort<std::shared_ptr<std::vector<geometry_msgs::PoseStamped> > >("navigation_path"),
    BT::InputPort<std::shared_ptr<project11_navigation::RobotCapabilities> >("robot_capabilities"),
    BT::InputPort<std::shared_ptr<tf2_ros::Buffer> >("tf_buffer"),
    BT::InputPort<std::shared_ptr<project11_navigation::Context> >("context")
  };
}

BT::NodeStatus PlanAction::onStart()
{
  auto start_pose = getInput<geometry_msgs::PoseStamped>("start_pose");
  if(!start_pose)
  {
    throw BT::RuntimeError("PlanAction named ", name(), " missing required input [start_pose]: ", start_pose.error() );    
  }

  auto goal_pose = getInput<geometry_msgs::PoseStamped>("goal_pose");
  if(!goal_pose)
  {
    throw BT::RuntimeError("PlanAction named ", name(), " missing required input [goal_pose]: ", goal_pose.error() );    
  }

  auto caps = getInput<std::shared_ptr<project11_navigation::RobotCapabilities> >("robot_capabilities");
  if(!caps)
  {
    throw BT::RuntimeError("PlanAction named ", name(), " missing required input [robot_capabilities]: ", caps.error() );    
  }

  auto tf_buffer = getInput<std::shared_ptr<tf2_ros::Buffer> >("tf_buffer");
  if(!tf_buffer)
  {
    throw BT::RuntimeError("PlanAction named ", name(), " missing required input [tf_buffer]: ", tf_buffer.error() );
  }

  auto context = getInput<std::shared_ptr<project11_navigation::Context> >("context");
  if(!context)
  {
    throw BT::RuntimeError("PlanAction named ", name(), " missing required input [context]: ", context.error() );
  }

  auto radius = caps.value()->getTurnRadiusAtSpeed(caps.value()->default_velocity.linear.x);

  auto start = start_pose.value(); 
  auto goal = goal_pose.value();

  std::string map_frame = context.value()->environment().mapFrame();

  if(start.header.frame_id != map_frame)
  {
    try
    {
      context.value()->tfBuffer()->transform(start, start, map_frame);
      start.header.frame_id = map_frame;
    }
    catch(const std::exception& e)
    {
      ROS_WARN_STREAM("Error transforming start_pose to map frame. " << e.what());
      return BT::NodeStatus::FAILURE;
    }
  }
  if(goal.header.frame_id != map_frame)
  {
    try
    {
      context.value()->tfBuffer()->transform(goal, goal, map_frame);
      goal.header.frame_id = map_frame;
    }
    catch(const std::exception& e)
    {
      ROS_WARN_STREAM("Error transforming goal_pose to map frame. " << e.what());
      return BT::NodeStatus::FAILURE;
    }
  }

  project11_nav_msgs::RobotState start_state, goal_state;
  start_state.pose = start.pose;
  goal_state.pose = goal.pose;


  planner_ = std::make_shared<DubinsAStar>(start_state, goal_state, context.value());

  return BT::NodeStatus::RUNNING;
}

BT::NodeStatus PlanAction::onRunning()
{
  auto start_pose = getInput<geometry_msgs::PoseStamped>("start_pose");
  if(!start_pose)
  {
    throw BT::RuntimeError("PlanAction named ", name(), " missing required input [start_pose]: ", start_pose.error() );    
  }


  auto context = getInput<std::shared_ptr<project11_navigation::Context> >("context");
  if(!context)
  {
    throw BT::RuntimeError("PlanAction named ", name(), " missing required input [context]: ", context.error() );
  }

  std::string map_frame = context.value()->environment().mapFrame();

  std_msgs::Header start_header;
  start_header.frame_id = map_frame;
  start_header.stamp = start_pose.value().header.stamp;

  std::vector<geometry_msgs::PoseStamped> potential_plan;
  if(planner_->getPlan(potential_plan, start_header))
  {
    setOutput("navigation_path", std::make_shared<std::vector<geometry_msgs::PoseStamped> >(potential_plan));
    return BT::NodeStatus::SUCCESS;  
  }
  else
      setOutput("potential_path", std::make_shared<std::vector<geometry_msgs::PoseStamped> >(potential_plan));

  return BT::NodeStatus::RUNNING;
}

void PlanAction::onHalted()
{
  planner_.reset();
}

} // namespace ccom_planner

