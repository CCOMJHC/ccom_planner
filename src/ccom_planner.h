#ifndef CCOM_PLANNER_PLANNER_H
#define CCOM_PLANNER_PLANNER_H

#include <project11_navigation/interfaces/task_to_task_workflow.h>
#include <std_msgs/Header.h>
#include "dubins_astar.h"

namespace ccom_planner
{

class Planner: public project11_navigation::TaskToTaskWorkflow
{
public:
  void configure(std::string name, project11_navigation::Context::Ptr context) override;
  void setGoal(const std::shared_ptr<project11_navigation::Task>& input) override;
  bool running() override;
  bool getResult(std::shared_ptr<project11_navigation::Task>& output) override;
private:
  void publishPlan(const std::vector<geometry_msgs::PoseStamped> &plan);
  void publishPlan();

  project11_navigation::Context::Ptr context_;
  std::shared_ptr<project11_navigation::Task> input_task_ = nullptr;
  std::shared_ptr<project11_navigation::Task> output_task_ = nullptr;

  ros::Time task_update_time_;

  std::vector<geometry_msgs::PoseStamped> plan_;

  std_msgs::Header start_header_;
  double speed_;

  ros::Publisher plan_publisher_;

  std::string output_task_type_ = "follow_trajectory";
  std::string output_task_name_ = "navigation_trajectory";

  std::shared_ptr<DubinsAStar> planner_;

};

} // namespace ccom_planner

#endif
