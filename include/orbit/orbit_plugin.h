#ifndef ORBIT_ORBIT_PLUGIN_H
#define ORBIT_ORBIT_PLUGIN_H

#include <project11_navigation/interfaces/task_to_task_workflow.h>
#include <geometry_msgs/PointStamped.h>

namespace orbit
{

class Orbit: public project11_navigation::TaskToTaskWorkflow
{
public:
  void configure(std::string name, project11_navigation::Context::Ptr context) override;
  void setGoal(const std::shared_ptr<project11_navigation::Task>& input) override;
  bool running() override;
  bool getResult(std::shared_ptr<project11_navigation::Task>& output) override;
private:
  project11_navigation::Context::Ptr context_;
  std::shared_ptr<project11_navigation::Task> input_task_;
  std::shared_ptr<project11_navigation::Task> output_task_;

  geometry_msgs::PointStamped target_;
  double default_radius_ = 250.0;
  double step_size_ = 5.0;
  double default_speed_ = 2.5;

  std::string output_task_type_ = "follow_trajectory";
  std::string output_task_name_ = "navigation_trajectory";
};


} // namespace orbit

#endif
