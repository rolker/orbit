#ifndef ORBIT_ORBIT_TASK_H
#define ORBIT_ORBIT_TASK_H

#include <project11_navigation/interfaces/task_wrapper.h>

namespace p11n = project11_navigation;

namespace orbit
{

class OrbitTask: public p11n::TaskWrapper
{

public:
  void updateTransit(const geometry_msgs::PoseStamped& from_pose, geometry_msgs::PoseStamped& out_pose) override;
  std::shared_ptr<p11n::Task> getCurrentNavigationTask() override;
  void configure(std::string name, std::shared_ptr<p11n::Context> context) override;
  void getPreviewDisplay(visualization_msgs::MarkerArray& marker_array) override;

};

} // namespace orbit

#endif