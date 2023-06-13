#include <orbit/orbit_task.h>
#include <project11_navigation/context.h>
#include <pluginlib/class_list_macros.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>

PLUGINLIB_EXPORT_CLASS(orbit::OrbitTask, project11_navigation::Task)

namespace orbit
{

void OrbitTask::updateTransit(const geometry_msgs::PoseStamped& from_pose, geometry_msgs::PoseStamped& out_pose, std::shared_ptr<p11n::Context> context)
{
  ROS_INFO_STREAM("OrbitTask::updateTransit");
  if(!message().poses.empty())
  {
    auto target = message().poses.front();
    auto odom = context->getOdometry();
    auto task_data = data();
    auto radius = 10.0;
    if(task_data["radius"])
      radius = task_data["radius"].as<double>();
    
    geometry_msgs::Point target_map = target.pose.position;
    if(odom.header.frame_id != target.header.frame_id)
      try
      {
        auto t = context->tfBuffer().lookupTransform(odom.header.frame_id, target.header.frame_id, ros::Time(0));
        tf2::doTransform(target.pose.position, target_map, t);
      }
      catch (tf2::TransformException &ex)
      {
        ROS_WARN_STREAM_THROTTLE(1.0,"Orbit: " << ex.what());
        out_pose = from_pose;
        return;
      }

    double dx = odom.pose.pose.position.x-target_map.x;
    double dy = odom.pose.pose.position.y-target_map.y;
    auto distance = sqrt(dx*dx+dy*dy);
    if(distance > 0)
    {
      auto scale = radius/distance;
      out_pose.header.frame_id = odom.header.frame_id;
      out_pose.pose.position.x = target_map.x +dx*scale;
      out_pose.pose.position.y = target_map.y +dy*scale;

      auto arrival_heading = atan2(dy,dx)+M_PI;
      tf2::Quaternion q;
      q.setRPY(0,0,arrival_heading);
      tf2::convert(q, out_pose.pose.orientation);
      return;
    }
  }
  out_pose = from_pose;
}

boost::shared_ptr<p11n::Task> OrbitTask::getCurrentNavigationTask()
{
  for(auto t: children().tasksByPriority(true))
  {
    if(t->message().type == "transit")
      return t;
  }
  return self();
}

} // namespace orbit
