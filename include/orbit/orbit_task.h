#ifndef ORBIT_ORBIT_TASK_H
#define ORBIT_ORBIT_TASK_H

#include <project11_navigation/task.h>
#include <project11_navigation/context.h>

namespace p11n = project11_navigation;

namespace orbit
{

class OrbitTask: public p11n::Task
{

public:
  void updateTransit(const geometry_msgs::PoseStamped& from_pose, geometry_msgs::PoseStamped& out_pose, p11n::Context::Ptr context) override;
  p11n::Task::Ptr getCurrentNavigationTask() override;


};

} // namespace orbit

#endif