#include "dubins_astar.h"

namespace ccom_planner
{

DubinsAStar::DubinsAStar(State start, State goal, project11_navigation::Context::Ptr context, double yaw_step): yaw_step_(yaw_step)
{
  costmap_ = *context->costmap()->getCostmap();

  step_size_ = costmap_.getResolution();

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

  goal_ = goal;
  goal_index_ = indexOf(goal);

  auto start_index = indexOf(start);

  if (start_index != goal_index_)
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
  while(!open_set_.empty())
  {
    {
      // Check if we need to quit before we're done
      std::lock_guard<std::mutex> abort_lock(abort_flag_mutex_);
      if(abort_flag_)
        break;
    }

    std::lock_guard<std::mutex> lock(open_set_mutex_);
    auto current = open_set_.top();
    open_set_.pop();

    NodeIndex current_index = indexOf(current->state);

    auto neighbors = generateNeighbors(current);

    for(auto n: neighbors)
    {
      auto ni = indexOf(n->state);
      if(ni == goal_index_)
      {
        n->h = 0.0;
        return n;
      }
      // A negative heuristic means an invalid state, so make sure
      // it's non negative. Also check that we haven't been here
      // already
      if(n->h >= 0 && (ni == current_index || !visited_nodes_[ni]))
      {
        // A negative cost means a leathal or off the map state
        double cost = getCost(n->state);
        if(cost >= 0.0)
        {
          // scale our speed using cost
          auto speed = speed_*(1.0-cost*.9);
          if(speed > 0)
          {
            n->state.speed = speed;
            // update the total time so far using the speed to calculate
            // how long from previous state to this one
            n->g += (n->cummulative_distance-current->cummulative_distance)/speed;
            open_set_.push(n);
          }
        }
        // mark the discretized state as visited no matter if the state is valid or not
        visited_nodes_[ni] = true;
      }
    }
  }
  // if we made it here, no plan was found
  return Node::Ptr();
}

NodeIndex DubinsAStar::indexOf(const State& state) const
{
  NodeIndex ret;
  ret.xi = state.x/step_size_;
  ret.yi = state.y/step_size_;
  ret.yawi = state.yaw.value()/yaw_step_;
  return ret;
}

bool DubinsAStar::dubins(const State & from, const State & to, DubinsPath & path) const
{
  double start[3];
  start[0] = from.x;
  start[1] = from.y;
  start[2] = from.yaw.value();
  
  double target[3];
  target[0] = to.x;
  target[1] = to.y;
  target[2] = to.yaw.value();

  int dubins_ret = dubins_shortest_path(&path, start, target, turn_radius_);
  
  return dubins_ret == 0;
}

double DubinsAStar::heuristic(const State & from, const State & to) const
{
  DubinsPath path;

  if(dubins(from, to, path))  
    return dubins_path_length(&path)/speed_;

  ROS_WARN_STREAM("Dubins path not found");
  return -1;
}

std::vector<Node::Ptr> DubinsAStar::generateNeighbors(Node::Ptr from) const
{
  std::vector<State> states;

  // First, let's sample from a direct Dubins path if we can.
  DubinsPath path;

  bool success = dubins(from->state, goal_, path);

  if(success)
  {
    double q[3];
    double distance = step_size_;
    if(dubins_path_sample(&path, distance, q) == 0)
    {
      State state;
      state.x = q[0];
      state.y = q[1];
      state.yaw = q[2];
      states.push_back(state);
    }
  }

  auto cosyaw = cos(from->state.yaw);
  auto sinyaw = sin(from->state.yaw);

  // Now consider our standard directions
  for(auto step: search_steps_)
  {
    State state;
    state.yaw = from->state.yaw + step.delta_yaw;

    state.x = from->state.x + step.dx*cosyaw + step.dy*sinyaw;
    state.y = from->state.y + step.dx*sinyaw + step.dy*cosyaw;
    states.push_back(state);
  }

  std::vector<Node::Ptr> ret;

  for(auto state: states)
  {
    double h = heuristic(state, goal_);
    if(h < 0)
      ROS_WARN_STREAM("Unable to calculate heuristic for neighbour from " << state << " to goal " << goal_);
    else
      ret.push_back(std::make_shared<Node>(state, h, from->cummulative_distance+step_size_, from->g, from));
  }
  return ret;
}

double DubinsAStar::getCost(const State& state)
{
  unsigned int x,y;
  if(costmap_.worldToMap(state.x, state.y, x, y))
  {
    auto cost = costmap_.getCost(x,y);
    if (cost < costmap_2d::INSCRIBED_INFLATED_OBSTACLE)
      return cost/double(costmap_2d::INSCRIBED_INFLATED_OBSTACLE-1);
  }
  return -1.0;
}

void unwrap(Node::Ptr plan, std::vector<geometry_msgs::PoseStamped> &poses, const std_msgs::Header& start_header)

{
  // Reverse the order of the states
  std::deque<State> states;
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
    p.pose.position.x = s.x;
    p.pose.position.y = s.y;
    tf2::Quaternion q;
    q.setRPY(0.0, 0.0, s.yaw.value());
    tf2::convert(q, p.pose.orientation);

    // set the timestamp based on planned speeds
    if(!poses.empty())
    {
      auto avg_speed = std::max(0.1,(last_speed + s.speed)/2.0);
      auto dx = s.x - poses.back().pose.position.x;
      auto dy = s.y - poses.back().pose.position.y;
      cumulative_time += ros::Duration(sqrt(dx*dx+dy*dy)/avg_speed);
      p.header.stamp = start_header.stamp + cumulative_time;
      last_speed = s.speed;
    }
    poses.push_back(p);
  }
}

}; // namespace ccom_planner

std::ostream& operator<< (std::ostream& os, const ccom_planner::State& state)
{
  os << "x,y: " << state.x << "," << state.y << "\tyaw: " << project11::AngleDegreesPositive(state.yaw).value();
  return os;
}

std::ostream& operator<< (std::ostream& os, const ccom_planner::Node& node)
{
  os << node.state << "\tf: " <<  node.f()  << "\tg: " << node.g << "\th: " << node.h << " distance: " << node.cummulative_distance;
  return os;
}
