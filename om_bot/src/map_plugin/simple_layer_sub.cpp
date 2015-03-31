#include "simple_layer_sub.h"
#include <pluginlib/class_list_macros.h>
#include <geometry_msgs/Pose.h>
#include <my_repository/map_cfgConfig.h>
#include <std_srvs/Empty.h>

PLUGINLIB_EXPORT_CLASS(simple_layer_namespace::SimpleLayerSub, costmap_2d::Layer)

using costmap_2d::LETHAL_OBSTACLE;
using costmap_2d::FREE_SPACE;

namespace simple_layer_namespace
{

SimpleLayerSub::SimpleLayerSub() {}

void SimpleLayerSub::onInitialize()
{
  ros::NodeHandle nh("~/" + name_);
  current_ = true;
  //client_ = nh.serviceClient<std_srvs::Empty>("/move_base/clear_costmaps");
  dsrv_ = new dynamic_reconfigure::Server<my_repository::map_cfgConfig>(nh);
  dynamic_reconfigure::Server<my_repository::map_cfgConfig>::CallbackType cb = boost::bind(
      &SimpleLayerSub::reconfigureCB, this, _1, _2);
  dsrv_->setCallback(cb);

  
  obs_x = 0;
  obs_y = 0;
  last_obj_x = 0;
  last_obj_y = 0;
  last_cost = LETHAL_OBSTACLE;
}



void SimpleLayerSub::reconfigureCB(my_repository::map_cfgConfig&config, uint32_t level)
{
  enabled_ = config.enabled;
  last_obj_x = obs_x;
  last_obj_y = obs_y;
  obs_x = config.object_x;
  obs_y = config.object_y;
}

void SimpleLayerSub::updateBounds(double origin_x, double origin_y, double origin_yaw, double* min_x,
                                           double* min_y, double* max_x, double* max_y)
{
  if (!enabled_)
    return;

  
  //mark_x_ = origin_x + cos(origin_yaw);
  //mark_y_ = origin_y + sin(origin_yaw);

  mark_x_ = obs_x;
  mark_y_ = obs_y;
  //ROS_INFO("robot_x: %f", origin_x);
  //ROS_INFO("robot_y: %f", origin_y);
  //ROS_INFO("robot_yaw: %f", origin_yaw);

  *min_x = std::min(*min_x, mark_x_);
  *min_y = std::min(*min_y, mark_y_);
  *max_x = std::max(*max_x, mark_x_);
  *max_y = std::max(*max_y, mark_y_);
}

void SimpleLayerSub::updateCosts(costmap_2d::Costmap2D& master_grid, int min_i, int min_j, int max_i,
                                          int max_j)
{
    unsigned int mx;
    unsigned int my;
  if (!enabled_)
  {
      std_srvs::Empty srv;
      //client_.call(srv);
      if(master_grid.worldToMap(last_obj_x, last_obj_y, mx, my)){
      master_grid.setCost(mx, my, last_cost);
      }
       return;
  }


  if(last_obj_x!=obs_x||last_obj_y!=obs_y){
      if(master_grid.worldToMap(last_obj_x, last_obj_y, mx, my)){
      master_grid.setCost(mx, my, last_cost);
      }

      if(master_grid.worldToMap(mark_x_, mark_y_, mx, my)){
          last_cost = master_grid.getCost(mx,my);
        master_grid.setCost(mx, my, LETHAL_OBSTACLE);
      }
  }
}

} // end namespace
