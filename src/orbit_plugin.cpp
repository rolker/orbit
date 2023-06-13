#include <orbit/orbit_plugin.h>
#include <pluginlib/class_list_macros.h>
#include <tf2/utils.h>

PLUGINLIB_EXPORT_CLASS(orbit::Orbit, project11_navigation::TaskToTaskWorkflow);

namespace orbit
{

void Orbit::configure(std::string name, project11_navigation::Context::Ptr context)
{
  context_ = context;
  ROS_INFO_STREAM("Initializing Orbit plugin with name " << name);
  ros::NodeHandle nh("~/" + name);
  nh.param("step_size", step_size_, step_size_);
  nh.param("default_radius", default_radius_, default_radius_);
  nh.param("default_speed", default_speed_, default_speed_);
  nh.param("output_task_type", output_task_type_, output_task_type_);
  nh.param("output_task_name", output_task_name_, output_task_name_);
}

void Orbit::setGoal(const boost::shared_ptr<project11_navigation::Task>& input)
{
  ROS_INFO_STREAM("orbit goal");
  input_task_ = input;
  output_task_.reset();
  if(input_task_)
  {
    auto odom = context_->getOdometry();
    if(input_task_->message().poses.empty())
    {
      target_.point = odom.pose.pose.position;
      target_.header.frame_id = odom.header.frame_id;
    }
    else
    {
      target_.point = input_task_->message().poses.front().pose.position;
      target_.header.frame_id = input_task_->message().poses.front().header.frame_id;
    }

    for(auto t: input_task_->children().tasks())
      if(t->message().type == output_task_type_ && t->message().id == input_task_->getChildID(output_task_name_))
      {
        output_task_ = t;
        break;
      }
    if(!output_task_)
    {
      output_task_ = input_task_->createChildTaskBefore(project11_navigation::Task::Ptr(),output_task_type_);
      input_task_->setChildID(output_task_, output_task_name_);
    }
    auto out_msg = output_task_->message();
    out_msg.curved_trajectories.clear();
    out_msg.poses.clear();
    output_task_->update(out_msg);
  }
}

bool Orbit::running()
{
  if(input_task_)
    return true;
  return false;
}

bool Orbit::getResult(project11_navigation::Task::Ptr& output)
{
  if(output_task_)
  {
    auto odom = context_->getOdometry();
    auto data = input_task_->data();

    auto radius = default_radius_;
    if(data["radius"])
      radius = data["radius"].as<double>();

    auto safety_distance = radius;
    if(data["safety_distance"])
      safety_distance = data["safety_distance"].as<double>();

    auto speed = default_speed_;
    if(data["speed"])
      speed = data["speed"].as<double>();

    geometry_msgs::Point target_map = target_.point;
    if(odom.header.frame_id != target_.header.frame_id)
      try
      {
        auto t = context_->tfBuffer().lookupTransform(odom.header.frame_id, target_.header.frame_id, ros::Time(0));
        tf2::doTransform(target_.point, target_map, t);
        ROS_INFO_STREAM_THROTTLE(1.0,"odom frame: " << odom.header.frame_id << " orbit target frame: " << target_.header.frame_id << " target point: " << target_.point << " maped: " << target_map << " t: " << t);
      }
      catch (tf2::TransformException &ex)
      {
        ROS_WARN_STREAM_THROTTLE(1.0,"Orbit: " << ex.what());
        return false;
      }

    if (safety_distance < radius && abs(target_map.z))
      radius = sqrt(pow(safety_distance, 2.0)-pow(target_map.z, 2.0));

    double dx = odom.pose.pose.position.x-target_map.x;
    double dy = odom.pose.pose.position.y-target_map.y;
    
    auto start_angle = atan2(dy,dx);
    auto dangle =  2*M_PI*step_size_/radius;

    auto out_msg = output_task_->message();
    out_msg.poses.clear();
    geometry_msgs::PoseStamped out_pose;
    out_pose.header = odom.header;
    out_pose.pose = odom.pose.pose;
    out_msg.poses.push_back(out_pose);

    ros::Time next_time;
    if(speed)
    {
      auto current_distance = sqrt(dx*dx+dy*dy);
      next_time = odom.header.stamp + ros::Duration(std::abs(radius-current_distance)/speed);
    }

    double angle = start_angle+dangle;
    while(angle < start_angle+M_PI*2.0)
    {
      out_pose.pose.position.x = radius*cos(angle);
      out_pose.pose.position.y = radius*sin(angle);
      out_pose.header.stamp = next_time;
      out_msg.poses.push_back(out_pose);
      angle += dangle;
      if(speed)
        next_time += ros::Duration(step_size_/speed);
    }
    
    output_task_->update(out_msg);

    output = output_task_;
    return true;
    
  }
  return false;
}

} // namespace orbit
