#ifndef CCOM_PLANNER_PLAN_ACTION_H
#define CCOM_PLANNER_PLAN_ACTION_H

#include <behaviortree_cpp/bt_factory.h>

namespace ccom_planner
{

class DubinsAStar;

class PlanAction: public BT::StatefulActionNode
{

public:
  PlanAction(const std::string& name, const BT::NodeConfig& config);

  static BT::PortsList providedPorts();

  BT::NodeStatus onStart() override;
  BT::NodeStatus onRunning() override;

  void onHalted() override;


private:
  std::shared_ptr<DubinsAStar> planner_;

};

} // namespace ccom_planner

#endif
