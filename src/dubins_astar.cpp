#include "dubins_astar.h"

namespace ccom_planner
{

DubinsAStar::DubinsAStar(State start, State goal, project11_navigation::Context::Ptr context, double yaw_step): yaw_step_(yaw_step)
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
  if(isnan(goal.yaw))
  {
    State gs = goal;
    for(double y = 0.0; y < 2*M_PI; y += yaw_step)
    {
      gs.yaw = y;
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

    //ROS_INFO_STREAM(*current);

    NodeIndex current_index = indexOf(current->state);

    auto neighbors = generateNeighbors(current);

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
        if(n->cummulative_distance < 4*turn_radius_)
          if(cost < 0.0)
            cost = 0.001;
          else if( cost < 0.01)
            cost = 0.01;
        if(cost >= 0.0)
        {
          // scale our speed using cost
          //auto speed = speed_*(1.0-cost*.9);
          auto speed = cost;
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

  for(auto goal: goals_)
  {
    bool success = dubins(from->state, goal, path);

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

double DubinsAStar::getCost(const State& state)
{
  grid_map::Position position(state.x, state.y);
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
