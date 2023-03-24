#include "dubins_astar.h"

namespace ccom_planner
{

DubinsAStar::DubinsAStar(State start, State goal, project11_navigation::Context::Ptr context, double yaw_step): yaw_step_(yaw_step)
{
  costmap_ = context->costmap();

  for(auto p: costmap_->getRobotFootprint())
  {
    auto distance2 = p.x*p.x + p.y*p.y;
    robot_radius_ = std::max(robot_radius_, distance2);
  }
  // delay the sqrt for efficency
  if(robot_radius_ > 0)
    robot_radius_ = sqrt(robot_radius_);

  step_size_ = costmap_->getCostmap()->getResolution();

  turn_radius_ = context->getRobotCapabilities().getTurnRadiusAtSpeed(context->getRobotCapabilities().default_velocity.linear.x);
  max_yaw_step_ = step_size_/turn_radius_;

  yaw_search_steps_.push_back(0.0);
  for(double yaw = yaw_step_; yaw < M_PI; yaw += yaw_step_)
  {
    if (yaw >= max_yaw_step_)
    {
      yaw_search_steps_.push_back(max_yaw_step_);
      yaw_search_steps_.push_back(-max_yaw_step_);
      break;
    }
    yaw_search_steps_.push_back(yaw);
    yaw_search_steps_.push_back(-yaw);
  }


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

Node::Ptr DubinsAStar::getPlan()
{
  if(!plan_)
  {
    if(plan_ready_.valid())
    {
      if(plan_ready_.wait_for(std::chrono::milliseconds(10)) == std::future_status::ready) 
      {
        plan_ = plan_ready_.get();
      }
      else
      {
        std::lock_guard<std::mutex> lock(open_set_mutex_);
        if(!open_set_.empty())
          return open_set_.top();
      }
    }
  }
  return plan_;
}

Node::Ptr DubinsAStar::plan()
{
  while(!open_set_.empty())
  {
    {
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
      if(n->h >= 0 && (ni == current_index || !visited_nodes_[ni]))
      {
        double cost = getCost(n->state);
        if(cost >= 0.0)
        {
          n->g += cost;
          open_set_.push(n);
        }
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
  if(isnan(state.yaw) || state.yaw < 0.0)
    ret.yawi = -1;
  else
    ret.yawi = state.yaw/yaw_step_;
  return ret;
}

bool DubinsAStar::dubins(const State & from, const State & to, DubinsPath & path) const
{
  double start[3];
  start[0] = from.x;
  start[1] = from.y;
  start[2] = from.yaw;
  
  double target[3];
  target[0] = to.x;
  target[1] = to.y;
  if(isnan(to.yaw))
    target[2] = from.yaw;
  else
    target[2] = to.yaw;

  int dubins_ret = dubins_shortest_path(&path, start, target, turn_radius_);
  
  return dubins_ret == 0;
}

double DubinsAStar::heuristic(const State & from, const State & to) const
{
  DubinsPath path;

  if(dubins(from, to, path))  
    return dubins_path_length(&path);

  ROS_WARN_STREAM("Dubins path not found");
  return -1;
}

std::vector<Node::Ptr> DubinsAStar::generateNeighbors(Node::Ptr from) const
{
  std::vector<Node::Ptr> ret;

  // First, let's sample from a direct Dubins path if we can.
  DubinsPath path;

  if(dubins(from->state, goal_, path))
  {
    double q[3];
    if(dubins_path_sample(&path, step_size_, q) == 0)
    {
      State state;
      state.x = q[0];
      state.y = q[1];
      state.yaw = q[2];
      if(state.yaw < 0.0)
        state.yaw += 2.0*M_PI;
      ret.push_back(std::make_shared<Node>(state, heuristic(state, goal_), from->g+step_size_, from));
    }
  }  

  // Now consider out standard directions
  for(auto delta_yaw: yaw_search_steps_)
  {
    auto yaw = from->state.yaw + delta_yaw;
    auto cosyaw = cos(yaw);
    auto sinyaw = sin(yaw);
    State state;
    state.x = from->state.x + step_size_*cosyaw;
    state.y = from->state.y + step_size_*sinyaw;
    if (yaw > 2.0*M_PI)
      yaw -= (2.0*M_PI);
    if (yaw < 0.0)
      yaw += (2.0*M_PI);
    state.yaw = yaw;
    ret.push_back(std::make_shared<Node>(state, heuristic(state, goal_), from->g+step_size_, from));
  }
  return ret;
}

double DubinsAStar::getCost(const State& state)
{
  unsigned char cost = 0; 
  double radius_squared = robot_radius_*robot_radius_;
  for(double dx = -robot_radius_; dx <= robot_radius_; dx += step_size_)
    for(double dy = -robot_radius_; dy <= robot_radius_; dy += step_size_)
      if(dx*dx+dy*dy <= radius_squared)
      {
        unsigned int x,y;
        if(costmap_->getCostmap()->worldToMap(state.x+dx, state.y+dy, x, y))
          cost = std::max(cost, costmap_->getCostmap()->getCost(x,y));
        else
          cost = 255;
        if(cost > 250)
          break;
      }
  if (cost>250)
    return -1.0;
  return cost/128.0;
}

std::vector<State> unwrap(Node::Ptr plan)
{
  std::vector<State> ret;
  std::deque<State> states;
  while(plan)
  {
    states.push_front(plan->state);
    plan = plan->previous_node;
  }
  for(auto s: states)
    ret.push_back(s);
  return ret;
}

}; // namespace ccom_planner

std::ostream& operator<< (std::ostream& os, const ccom_planner::State& state)
{
  os << "x,y: " << state.x << "," << state.y << "\tyaw: " << state.yaw;
  return os;
}

std::ostream& operator<< (std::ostream& os, const ccom_planner::Node& node)
{
  os << node.state << "\tf: " <<  node.f()  << "\tg: " << node.g << "\th: " << node.h;
  return os;
}
