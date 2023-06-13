#include "dubins_astar.h"
#include <tf2/utils.h>

namespace ccom_planner
{

DubinsAStar::DubinsAStar(project11_nav_msgs::RobotState start, project11_nav_msgs::RobotState goal, project11_navigation::Context::Ptr context, double yaw_step): yaw_step_(yaw_step)
{
  environment_snapshot_ = context->environment().snapshot();

  step_size_ = environment_snapshot_.static_grids_by_resolution.begin()->first;
 
  turn_radius_ = context->getRobotCapabilities().getTurnRadiusAtSpeed(context->getRobotCapabilities().default_velocity.linear.x);

  double max_yaw_step = step_size_/turn_radius_;

  search_steps_.push_back(SearchStep(0.0, step_size_));
  for(double yaw = yaw_step_; yaw < M_PI; yaw += yaw_step_)
  {
    if (yaw >= max_yaw_step)
    {
      search_steps_.push_back(SearchStep(max_yaw_step, step_size_));
      search_steps_.push_back(SearchStep(-max_yaw_step, step_size_));
      break;
    }
    search_steps_.push_back(SearchStep(yaw, step_size_));
    search_steps_.push_back(SearchStep(-yaw, step_size_));
  }

  speed_ = context->getRobotCapabilities().default_velocity.linear.x;

  start_ = start;

  // If goal orientation is not valid, assume it means it is not important
  // so add all discretized orientations as goals.
  tf2::Quaternion q;
  tf2::fromMsg(goal.pose.orientation, q);
  if(q.length2() < 0.5)
  {
    auto gs = goal;
    for(double y = 0.0; y < 2*M_PI; y += yaw_step)
    {
      tf2::Quaternion q;
      q.setRPY(0, 0, y);
      gs.pose.orientation = tf2::toMsg(q);
      goals_.push_back(gs);
      goal_indexes_.push_back(indexOf(gs));
    }
  }
  else
  {
    goals_.push_back(goal);
    goal_indexes_.push_back(indexOf(goal));
  }

  auto start_index = indexOf(start);

  if (!isGoal(start_index))
  {
    // Don't check the start position against the costmap in case we are 
    // at a dock or other valid situation where we'd fail a costmap check
    open_set_.push(std::make_shared<Node>(start, heuristic(start, goal)));
    visited_nodes_[start_index] = true;
  }
  
  // start the planning thread
  plan_ready_ = std::async(&DubinsAStar::plan, this);
}

DubinsAStar::~DubinsAStar()
{
  {
    std::lock_guard<std::mutex> lock(abort_flag_mutex_);
    abort_flag_ = true;
  }
  if(plan_ready_.valid())
    plan_ready_.wait();
}

bool DubinsAStar::isGoal(const NodeIndex& i) const
{
  for(auto gi: goal_indexes_)
    if(gi == i)
      return true;
  return false;
}

bool DubinsAStar::getPlan(std::vector<geometry_msgs::PoseStamped> &plan, const std_msgs::Header& start_header)
{
  if(!plan_)
  {
    // is the planning thread still running or done with an answer?
    if(plan_ready_.valid())
    {
      if(plan_ready_.wait_for(std::chrono::milliseconds(10)) == std::future_status::ready) 
      {
        // the planning thread has returned
        plan_ = plan_ready_.get();
      }
      else
      {
        // Still planning, so 
        // return the top of the open set as a plan in progress
        std::lock_guard<std::mutex> lock(open_set_mutex_);
        if(!open_set_.empty())
          unwrap(open_set_.top(), plan, start_header);
        return false;
      }
    }
    else
      return true; // done planning, but with no plan
  }
  if(plan_)
  {
    unwrap(plan_, plan, start_header);
    return true; // done planning with a plan
  }
  return false;
}

Node::Ptr DubinsAStar::plan()
{
  bool done;
  {
    std::lock_guard<std::mutex> lock(open_set_mutex_);
    done = open_set_.empty();
  }

  // In case we start where the map says we can go.
  bool free_from_starting_obstacle = false;

  while(!done)
  {
    {
      // Check if we need to quit before we're done
      std::lock_guard<std::mutex> abort_lock(abort_flag_mutex_);
      if(abort_flag_)
        break;
    }

    Node::Ptr current;
    {
      std::lock_guard<std::mutex> lock(open_set_mutex_);
      current = open_set_.top();
      open_set_.pop();
    }

    //ROS_INFO_STREAM(*current);

    NodeIndex current_index = indexOf(current->state);

    auto neighbors = generateNeighbors(current);

    // in case we are still stuck in a starting obstacle,
    // are any of the neigbours no longer in an obstacle? 
    bool free_from_obstacles = false;

    for(auto n: neighbors)
    {
      auto ni = indexOf(n->state);
      if(isGoal(ni))
      {
        n->h = 0.0;
        return n;
      }
      // A negative heuristic means an invalid state, so make sure
      // it's non negative. Also check that we haven't been here
      // already
      if(n->h >= 0 && (ni.samePosition(current_index) || !visited_nodes_[ni]))
      {
        // A negative cost means a leathal or off the map state
        double cost = getCost(n->state);
        if(cost > 0.0)
          free_from_obstacles = true;

        // allow leeway if starting close to an obstacle, such as a dock
        if(!free_from_starting_obstacle) 
          if(cost < 0.0)
            cost = 0.001;
          else if( cost < 0.01)
            cost = 0.01;
        if(cost >= 0.0)
        {
          auto speed = cost;
          if(speed > 0)
          {
            n->state.twist.linear.x = speed;
            // update the total time so far using the speed to calculate
            // how long from previous state to this one
            n->g += (n->cummulative_distance-current->cummulative_distance)/speed;
            std::lock_guard<std::mutex> lock(open_set_mutex_);
            open_set_.push(n);
          }
        }
        // mark the discretized state as visited no matter if the state is valid or not
        visited_nodes_[ni] = true;
      }
    }
    if (free_from_obstacles)
      free_from_starting_obstacle = true;
    std::lock_guard<std::mutex> lock(open_set_mutex_);
    done = open_set_.empty();
  }
  // if we made it here, no plan was found
  return Node::Ptr();
}

NodeIndex DubinsAStar::indexOf(const project11_nav_msgs::RobotState& state) const
{
  NodeIndex ret;
  ret.xi = state.pose.position.x/step_size_;
  ret.yi = state.pose.position.y/step_size_;
  auto yaw = tf2::getYaw(state.pose.orientation);
  ret.yawi = yaw/yaw_step_;
  return ret;
}

bool DubinsAStar::dubins(const project11_nav_msgs::RobotState & from, const project11_nav_msgs::RobotState & to, DubinsPath & path) const
{
  double start[3];
  start[0] = from.pose.position.x;
  start[1] = from.pose.position.y;
  start[2] = tf2::getYaw(from.pose.orientation);
  
  double target[3];
  target[0] = to.pose.position.x;
  target[1] = to.pose.position.y;
  target[2] = tf2::getYaw(to.pose.orientation);

  int dubins_ret = dubins_shortest_path(&path, start, target, turn_radius_);
  
  return dubins_ret == 0;
}

double DubinsAStar::heuristic(const project11_nav_msgs::RobotState & from, const project11_nav_msgs::RobotState & to) const
{
  DubinsPath path;

  if(dubins(from, to, path))  
    return dubins_path_length(&path)/speed_;

  ROS_WARN_STREAM("Dubins path not found");
  return -1;
}

std::vector<Node::Ptr> DubinsAStar::generateNeighbors(Node::Ptr from) const
{
  std::vector<project11_nav_msgs::RobotState> states;

  // First, let's sample from a direct Dubins path if we can.
  DubinsPath path;

  for(auto goal: goals_)
  {
    bool success = dubins(from->state, goal, path);

    if(success)
    {
      double q[3];
      double distance = step_size_;
      if(dubins_path_sample(&path, distance, q) == 0)
      {
        project11_nav_msgs::RobotState state;
        state.pose.position.x = q[0];
        state.pose.position.y = q[1];
        tf2::Quaternion quat;
        quat.setRPY(0,0,q[2]);
        state.pose.orientation = tf2::toMsg(quat);
        states.push_back(state);
      }
    }
  }

  auto yaw = tf2::getYaw(from->state.pose.orientation);

  auto cosyaw = cos(yaw);
  auto sinyaw = sin(yaw);

  // Now consider our standard directions
  for(auto step: search_steps_)
  {
    project11_nav_msgs::RobotState state;
    tf2::Quaternion quat;
    quat.setRPY(0, 0, yaw+step.delta_yaw);
    state.pose.orientation = tf2::toMsg(quat);

    state.pose.position.x = from->state.pose.position.x + step.dx*cosyaw + step.dy*sinyaw;
    state.pose.position.y = from->state.pose.position.y + step.dx*sinyaw + step.dy*cosyaw;
    states.push_back(state);
  }

  std::vector<Node::Ptr> ret;

  for(auto state: states)
  {
    double h = -1;
    for(auto goal: goals_)
    {
      double gh = heuristic(state, goal);
      if(gh >= 0.0)
        if(h < 0.0 || gh < h)
          h = gh;
    }
    if(h >= 0)
      ret.push_back(std::make_shared<Node>(state, h, from->cummulative_distance+step_size_, from->g, from));
  }
  return ret;
}

double DubinsAStar::getCost(const project11_nav_msgs::RobotState& state)
{
  grid_map::Position position(state.pose.position.x, state.pose.position.y);
  for(auto gv: environment_snapshot_.static_grids_by_resolution)
    for(auto grid_label: gv.second)
    {
      grid_map::Index index;
      grid_map::GridMap& grid = environment_snapshot_.static_grids[grid_label];
      if(grid.getIndex(position, index))
      {
        float ret = grid.at("speed", index);
        for(grid_map::CircleIterator i(grid, position, turn_radius_); !i.isPastEnd(); ++i)
          ret = std::min(ret, grid.at("speed", *i));
        return ret;
      }
    }
  return -1.0;
}

void unwrap(Node::Ptr plan, std::vector<geometry_msgs::PoseStamped> &poses, const std_msgs::Header& start_header)

{
  // Reverse the order of the states
  std::deque<project11_nav_msgs::RobotState> states;
  while(plan)
  {
    states.push_front(plan->state);
    plan = plan->previous_node;
  }

  double last_speed = 0.0;
  ros::Duration cumulative_time(0.0);
  for(auto s: states)
  {
    geometry_msgs::PoseStamped p;
    p.header = start_header;
    p.pose = s.pose;

    // set the timestamp based on planned speeds
    if(!poses.empty())
    {
      auto avg_speed = std::max(0.1,(last_speed + s.twist.linear.x)/2.0);
      auto dx = s.pose.position.x - poses.back().pose.position.x;
      auto dy = s.pose.position.y - poses.back().pose.position.y;
      cumulative_time += ros::Duration(sqrt(dx*dx+dy*dy)/avg_speed);
      p.header.stamp = start_header.stamp + cumulative_time;
      last_speed = s.twist.linear.x;
    }
    poses.push_back(p);
  }
}

}; // namespace ccom_planner

// std::ostream& operator<< (std::ostream& os, const ccom_planner::State& state)
// {
//   os << "x,y: " << state.x << "," << state.y << "\tyaw: " << project11::AngleDegreesPositive(state.yaw).value();
//   return os;
// }

// std::ostream& operator<< (std::ostream& os, const ccom_planner::Node& node)
// {
//   os << node.state << "\tf: " <<  node.f()  << "\tg: " << node.g << "\th: " << node.h << " distance: " << node.cummulative_distance;
//   return os;
// }
