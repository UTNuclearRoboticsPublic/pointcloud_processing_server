#include <ros/ros.h>
#include <tf/transform_broadcaster.h>
#include <tf/transform_listener.h>

int main(int argc, char** argv){
  ros::init(argc, argv, "map_maker_pcs");
  ros::NodeHandle nh;

  static tf::TransformBroadcaster br_map;
  static tf::TransformBroadcaster br_cam;
  static tf::TransformBroadcaster br_cam2;

  std::vector<float> map_origin, map_rotation, cam_origin, cam_rotation;
  if( !nh.getParam("map_maker_pcs/map_origin", map_origin) )
  {
    map_origin.push_back(0.0);
    map_origin.push_back(0.0);
    map_origin.push_back(0.0);
  }
  if( !nh.getParam("map_maker_pcs/map_rotation", map_rotation) )
  {
    map_rotation.push_back(0.0);
    map_rotation.push_back(0.0);
    map_rotation.push_back(0.0);
  }
  if( !nh.getParam("map_maker_pcs/cam_origin", cam_origin) )
  {
    cam_origin.push_back(0.0);
    cam_origin.push_back(0.0);
    cam_origin.push_back(0.0);
  }
  if( !nh.getParam("map_maker_pcs/cam_rotation", cam_rotation) )
  {
    cam_rotation.push_back(1.4);
    cam_rotation.push_back(0.0);
    cam_rotation.push_back(3.14);
  }

  tf::Transform transform_map;
  transform_map.setOrigin( tf::Vector3(map_origin[0],map_origin[1],map_origin[2]) );
  tf::Quaternion q_map;
  q_map.setRPY(map_rotation[0],map_rotation[1],map_rotation[2]);
  transform_map.setRotation(q_map);

  tf::Transform transform_cam;
  transform_cam.setOrigin( tf::Vector3(cam_origin[0],cam_origin[1],cam_origin[2]) );
  tf::Quaternion q_cam;
  q_cam.setRPY(cam_rotation[0],cam_rotation[1],cam_rotation[2]);
  transform_cam.setRotation(q_cam);

      tf::TransformListener listener;


  while(ros::ok())
  {
    br_map.sendTransform(tf::StampedTransform(transform_map, ros::Time::now(), "world", "map"));
    br_cam.sendTransform(tf::StampedTransform(transform_cam, ros::Time::now(), "map", "camera_depth_optical_frame"));
    br_cam2.sendTransform(tf::StampedTransform(transform_map, ros::Time::now(), "camera_depth_optical_frame", "camera_rgb_optical_frame"));
    ros::Duration(1/30).sleep();
  };                                    

};
