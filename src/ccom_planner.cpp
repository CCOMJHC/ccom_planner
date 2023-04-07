#include "ccom_planner.h"

#include <nav_msgs/Path.h>
#include <pluginlib/class_list_macros.h>

#include <tf2/utils.h>

PLUGINLIB_EXPORT_CLASS(ccom_planner::Planner, project11_navigation::TaskToTaskWorkflow);

namespace ccom_planner
{

void Planner::configure(std::string name, project11_navigation::Context::Ptr context)
{
  context_ = context;
  ROS_INFO_STREAM("Initializing CCOM Planner plugin with name " << name);
  ros::NodeHandle nh("~/" + name);
  nh.param("output_task_type", output_task_type_, output_task_type_);
  nh.param("output_task_name", output_task_name_, output_task_name_);

  plan_publisher_ = nh.advertise<nav_msgs::Path>("plan", 1, true);
}

void Planner::setGoal(const std::shared_ptr<project11_navigation::Task>& input)
{
  input_task_ = input;
  task_update_time_ = ros::Time();
  if(input_task_ && input_task_->message().poses.size() < 2)
    input_task_->setDone();
  output_task_.reset();
  plan_.clear();
  planner_.reset();
  publishPlan();
}

bool Planner::running()
{
  bool done = true;
  if(input_task_ && !input_task_->done())
  {
    if(input_task_->message().poses.size() < 2)
    {
      input_task_->setDone();
      output_task_.reset();
      return false;
    }
    
    done = false;

    if(planner_ && plan_.empty())
    {
      std::vector<geometry_msgs::PoseStamped> potential_plan;
      if(planner_->getPlan(potential_plan, start_header_))
      {
        plan_ = potential_plan;
        planner_.reset();
      }
      else
        publishPlan(potential_plan);

      if(!plan_.empty())
      {
        ROS_DEBUG_STREAM("Start: " << plan_.front().header.stamp << " end: " << plan_.back().header.stamp);
        for(auto t: input_task_->children().tasks())
          if(t->message().type == output_task_type_ && t->message().id == input_task_->getChildID(output_task_name_))
          {
            output_task_ = t;
            break;
          }
        if(!output_task_)
        {
          output_task_ = input_task_->createChildTaskBefore(std::shared_ptr<project11_navigation::Task>(),output_task_type_);
          input_task_->setChildID(output_task_, output_task_name_);
        }
        auto out_msg = output_task_->message();
        out_msg.curved_trajectories.clear();
        out_msg.poses = plan_;
        output_task_->update(out_msg);

        publishPlan();
      }
    }

    if(plan_.empty() && !planner_)
    {
      auto caps = context_->getRobotCapabilities();

      speed_ = caps.default_velocity.linear.x;
      double radius = caps.getTurnRadiusAtSpeed(speed_);
      if( radius <= 0.0)
        ROS_WARN_STREAM_THROTTLE(1.0, "Radius is not > zero: " << radius);
      auto ros_costmap = context_->costmap();
      if(!ros_costmap || !ros_costmap->getLayeredCostmap())
      {
        ROS_ERROR_STREAM("Costmap not found for Dubins planner");
        return false;
      }
      auto* costmap = ros_costmap->getLayeredCostmap()->getCostmap();

      auto start = input_task_->message().poses[0]; 
      auto goal = input_task_->message().poses[1];

      std::string map_frame = ros_costmap->getGlobalFrameID();
      map_frame = context_->environment().getStaticGrids().begin()->second.getFrameId();

      if(start.header.frame_id != map_frame)
      {
        try
        {
          context_->tfBuffer().transform(start, start, map_frame);
          start.header.frame_id = map_frame;
        }
        catch(const std::exception& e)
        {
          ROS_WARN_STREAM(e.what());
          return !done;
        }
      }
      if(goal.header.frame_id != map_frame)
      {
        try
        {
          context_->tfBuffer().transform(goal, goal, map_frame);
          goal.header.frame_id = map_frame;
        }
        catch(const std::exception& e)
        {
          ROS_WARN_STREAM(e.what());
          return !done;
        }
      }

      State start_state, goal_state;
      start_state.x = start.pose.position.x;
      start_state.y = start.pose.position.y;
      start_state.yaw = tf2::getYaw(start.pose.orientation);

      goal_state.x = goal.pose.position.x;
      goal_state.y = goal.pose.position.y;
      goal_state.yaw = tf2::getYaw(goal.pose.orientation);

      ROS_INFO_STREAM("Launching planner from " << start_state << " to " << goal_state);

      start_header_ = start.header;

      planner_ = std::make_shared<DubinsAStar>(start_state, goal_state, context_);
      task_update_time_ = input_task_->lastUpdateTime();
    }
  }
  return !done;
}

bool Planner::getResult(std::shared_ptr<project11_navigation::Task>& output)
{
  if(output_task_)
  {
    output = output_task_;
    return true;
  }
  return false;
}

void Planner::publishPlan(const std::vector<geometry_msgs::PoseStamped> &plan)
{
  nav_msgs::Path path;
  if(!plan.empty())
  {
    path.header = plan.front().header;
    path.poses = plan;
  }
  plan_publisher_.publish(path);
}

void Planner::publishPlan()
{
  publishPlan(plan_);
}

} // namespace dubins_planner
