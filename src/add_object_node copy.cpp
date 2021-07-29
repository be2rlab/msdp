#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>

#include <moveit_msgs/DisplayRobotState.h>
#include <moveit_msgs/DisplayTrajectory.h>

#include <moveit_msgs/AttachedCollisionObject.h>
#include <moveit_msgs/CollisionObject.h>

#include <moveit_visual_tools/moveit_visual_tools.h>

#include <stdio.h>
#include <cstdlib>
#include <geometry_msgs/PointStamped.h>
#include <geometry_msgs/Pose.h>
#include <tf/transform_listener.h>
#include <tf/transform_broadcaster.h>

#include "std_msgs/Float32MultiArray.h"

#include "msdp/GoalPoses.h"
static const std::string PLANNING_GROUP = "manipulator";
class Add_object_class
{

  double b[11];
  //namespace rvt = rviz_visual_tools;
  ros::NodeHandle node_handle;
  ros::Subscriber sub;
  geometry_msgs::PointStamped base_point1;
  geometry_msgs::PointStamped obb_point;
  geometry_msgs::QuaternionStamped base_orient;
  geometry_msgs::QuaternionStamped obb_orient;
  moveit_msgs::CollisionObject collision_object;
  // msdp::GoalPoses srv;
  geometry_msgs::Pose p1, p2;

public:
  Add_object_class()
  {

    sub = node_handle.subscribe("/obb_array", 10, &Add_object_class::IvansNodeCallback, this);
    //ros::ServiceClient client = node_handle.serviceClient<msdp::GoalPoses>("GoalPoses");
  }

  void add_object(double a[], std::vector<moveit_msgs::CollisionObject> &collision_objects, std::string name_obj, moveit_msgs::CollisionObject &collision_object)
  {

    collision_object.id = name_obj;

    //Препятствие

    // Define a box to add to the world.
    shape_msgs::SolidPrimitive primitive;
    primitive.type = primitive.BOX;
    primitive.dimensions.resize(3);
    primitive.dimensions[primitive.BOX_X] = a[0];
    primitive.dimensions[primitive.BOX_Y] = a[1];
    primitive.dimensions[primitive.BOX_Z] = a[2];

    // Define a pose for the box (specified relative to frame_id)
    geometry_msgs::Pose box_pose;
    box_pose.orientation.x = a[6];
    box_pose.orientation.y = a[7];
    box_pose.orientation.z = a[8];
    box_pose.orientation.w = a[9];
    box_pose.position.x = a[3];
    box_pose.position.y = a[4];
    box_pose.position.z = a[5];obb_frame
    {
      ros::Time now = ros::Time::now();
      listener.waitForTransform("/base", "/camera_color_optical_frame",
                                now, ros::Duration(3.0));
      listener.transformPoint("/base_link", wrist3_point, base_point);

      ROS_INFO("camera_color_optical_frame: (%.5f, %.5f. %.5f) -----> base_link: (%.5f, %.5f, %.5f) at time %.5f",
               wrist3_point.point.x, wrist3_point.point.y, wrist3_point.point.z,
               base_point.point.x, base_point.point.y, base_point.point.z, base_point.header.stamp.toSec());
    }
    catch (tf::TransformException &ex)
    {
      ROS_ERROR("Received an exception trying to transform a point from \"camera_color_optical_frame\" to \"base_link\": %s", ex.what());
    }
  }

  void transformQuaternion(const tf::TransformListener &listener, geometry_msgs::QuaternionStamped &base, geometry_msgs::QuaternionStamped &obj)
  {
    //we'll create a point in the base_laser frame that we'd like to transform to the base_link frame

    try
    {
      ros::Time now = ros::Time::now();
      listener.waitForTransform("/base_link", "/obb_frame",
                                now, ros::Duration(3.0));
      listener.transformQuaternion("/base_link", obj, base);

      ROS_INFO("obj:  (%.2f, %.2f. %.2f) -----> base: (%.2f, %.2f, %.2f) at time %.2f",
               obj.quaternion.x, obj.quaternion.y, obj.quaternion.z,
               base.quaternion.x, base.quaternion.y, base.quaternion.z, base.header.stamp.toSec());
    }
    catch (tf::TransformException &ex)
    {
      ROS_ERROR("Received an exception trying to transform a quaternion from \"obj\" to \"base\": %s", ex.what());
      ROS_INFO("obj:  (%.5f, %.5f. %.5f) -----> base: (%.5f, %.5f, %.5f) at time %.5f",
               obj.quaternion.x, obj.quaternion.y, obj.quaternion.z,
               base.quaternion.x, base.quaternion.y, base.quaternion.z, base.header.stamp.toSec());
    }
  }

  void IvansNodeCallback(const std_msgs::Float32MultiArray::ConstPtr &msg)
  {
    //   static const std::string PLANNING_GROUP = "manipulator";

    for (int i = 0; i <= 11; i++)
    {
      b[i] = msg->data[i];
      std::cout << b[i] << std::endl;
    }

    ROS_INFO("Data from topic: (%.5f, %.5f, %.5f) -----> : (%.5f, %.5f, %.5f) -----> : (%.5f, %.5f, %.5f)-----> : (%.5f, %.5f, %.5f)",
             b[0], b[1], b[2],
             b[3], b[4], b[5],
             b[6], b[7], b[8],
             b[9], b[10], b[11]);

    std::string name_obj;
    name_obj = "box1";
    //Переменная для центра масс и размеров объекта, который будем добавлять. Первые три числа - размеры объекта, Вторые три - центр масс,
    //  последние четыре - ориентация объекта в виде кватерниона.
    double a[9];

    tf::TransformListener listener1(ros::Duration(1));

    // Размеры
    a[0] = b[9];
    a[1] = b[10];
    a[2] = b[11];
    // Центр

    obb_point.header.frame_id = "/camera_color_optical_frame";
    base_point1.header.frame_id = "/base";
    //we'll just use the most recent transform available for our simple example
    obb_point.header.stamp = ros::Time();

    //just an arbitrary point in space
    obb_point.point.x = b[6];
    obb_point.point.y = b[7];
    obb_point.point.z = b[8];

    transformPoint(listener1, base_point1, obb_point);
    a[3] = base_point1.point.x;
    a[4] = base_point1.point.y;
    a[5] = base_point1.point.z;

    // Ориентация

    Eigen::Vector3f major_vector(b[0], b[1], b[2]);
    Eigen::Vector3f middle_vector(b[3], b[4], b[5]);

    Eigen::Vector3f minor_vector = major_vector.cross(middle_vector);

    Eigen::Matrix3f rot_matrix;
    rot_matrix.col(0) = major_vector;
    rot_matrix.col(1) = middle_vector;
    rot_matrix.col(2) = minor_vector;

    Eigen::Quaternionf quat(rot_matrix);

    obb_orient.header.frame_id = "/camera_color_optical_frame";
    base_orient.header.frame_id = "/base";

    obb_orient.quaternion.x = quat.x();
    obb_orient.quaternion.y = quat.y();
    obb_orient.quaternion.z = quat.z();
    obb_orient.quaternion.w = quat.w();

    // transformQuaternion(listener1, base_orient, obb_orient);

    // a[6] = base_orient.quaternion.x;
    // a[7] = base_orient.quaternion.y;
    // a[8] = base_orient.quaternion.z;
    // a[9] = base_orient.quaternion.w;

    // collision_object.header.frame_id = move_group_interface.getPlanningFrame();

    // moveit_visual_tools::MoveItVisualTools visual_tools("base"); //shoulder_link
    // visual_tools.deleteAllMarkers();
    // visual_tools.loadRemoteControl();
    // std::vector<moveit_msgs::CollisionObject> collision_objects;
    // add_object(a, collision_objects, name_obj, collision_object);
    // planning_scene_interface.addCollisionObjects(collision_objects);

    // p1.position.x = a[3];
    // p1.position.y = a[4];
    // p1.position.z = a[5];
    // p1.orientation.x = a[6];
    // p1.orientation.y = a[7];
    // p1.orientation.z = a[8];
    // p1.orientation.w = a[9];

    // p2.position.x = -0.3;
    // p2.position.y = -0.3;
    // p2.position.z = 0;
    // p2.orientation.x = 0;
    // p2.orientation.y = 0;
    // p2.orientation.z = 0;
    // p2.orientation.w = 1;
    // ros::ServiceClient client = node_handle.serviceClient<msdp::GoalPoses>("/GoalPoses");
    // srv.request.p1 = p1;
    // srv.request.p2 = p2;
    // if (client.call(srv))
    // {
    //   ROS_INFO("Send two grasping points to client");
    // }
    // else
    // {
    //   ROS_ERROR("Failed to call service GoalPoses");
    // }
  }

  void spin()
  {

    ros::Rate R(300);

    while (node_handle.ok())
    {

      // ros::spin();
      //       static const std::string PLANNING_GROUP = "manipulator";
      //    //ros::Subscriber sub = node_handle.subscribe("/obb_array", 1000, &Add_object_class::IvansNodeCallback, this);

      //     // for ( int i = 0; i <= 11; i++)
      //     // {
      //     //   b[i] = msg->data[i];
      //     // }
      //     // ROS_INFO("Data from topic: (%.5f, %.5f, %.5f) -----> : (%.5f, %.5f, %.5f) -----> : (%.5f, %.5f, %.5f)-----> : (%.5f, %.5f, %.5f)",
      //     // b[0], b[1], b[2],
      //     // b[3], b[4], b[5],
      //     // b[6], b[7], b[8],
      //     // b[9], b[10], b[11]
      //     // );

      //       //Подключаем то, чем управляем к интерфейсу
      //   moveit::planning_interface::MoveGroupInterface move_group_interface(PLANNING_GROUP);

      //   // Подгружаем сцену из рвиза
      //    moveit::planning_interface::PlanningSceneInterface planning_scene_interface;

      //   // Raw pointers are frequently used to refer to the planning group for improved performance.
      //   const moveit::core::JointModelGroup* joint_model_group =
      //       move_group_interface.getCurrentState()->getJointModelGroup(PLANNING_GROUP);
      //   std::string name_obj;
      //   name_obj = "box1";
      //   //Переменная для центра масс и размеров объекта, который будем добавлять. Первые три числа - размеры объекта, Вторые три - центр масс,
      //   //  последние четыре - ориентация объекта в виде кватерниона.
      //   double a[9];

      //   tf::TransformListener listener1(ros::Duration(5));

      //   // Размеры
      //   a[0] = b[9];
      //   a[1] = b[10];
      //   a[2] = b[11];
      //   // Центр

      //   obb_point.header.frame_id = "/camera_color_optical_frame";
      //   base_point1.header.frame_id = "/base";
      //   //we'll just use the most recent transform available for our simple example
      //   obb_point.header.stamp = ros::Time();

      //   //just an arbitrary point in space
      //   obb_point.point.x = b[6];
      //   obb_point.point.y = b[7];
      //   obb_point.point.z = b[8];

      // ROS_INFO("00");
      //     transformPoint(listener1, base_point1, obb_point);
      //     ROS_INFO("01");
      //   a[3] = base_point1.point.x;
      //   a[4] = base_point1.point.y;
      //   a[5] = base_point1.point.z;

      //   // Ориентация

      // 		Eigen::Vector3f major_vector(b[0],b[1],b[2]);
      // 		Eigen::Vector3f middle_vector(b[3],b[4],b[5]);

      // 		Eigen::Vector3f minor_vector = major_vector.cross(middle_vector);

      // 		Eigen::Matrix3f rot_matrix;
      // 		rot_matrix.col(0) = major_vector;
      // 		rot_matrix.col(1) = middle_vector;
      // 		rot_matrix.col(2) = minor_vector;

      //     Eigen::Quaternionf quat(rot_matrix);

      //   obb_orient.header.frame_id = "/camera_color_optical_frame";
      //   base_orient.header.frame_id = "/base";

      //   obb_orient.quaternion.x = quat.x();
      //   obb_orient.quaternion.y = quat.y();
      //   obb_orient.quaternion.z = quat.z();
      //   obb_orient.quaternion.w = quat.w();
      // ROS_INFO("02");
      //   transformQuaternion(listener1, base_orient, obb_orient);
      // ROS_INFO("03");
      //   a[6] = base_orient.quaternion.x;
      //   a[7] = base_orient.quaternion.y;
      //   a[8] = base_orient.quaternion.z;
      //   a[9] = base_orient.quaternion.w;

      //   collision_object.header.frame_id = move_group_interface.getPlanningFrame();
      //   ROS_INFO("04");
      //   moveit_visual_tools::MoveItVisualTools visual_tools("base"); //shoulder_link
      //   visual_tools.deleteAllMarkers();
      //   visual_tools.loadRemoteControl();
      //   std::vector<moveit_msgs::CollisionObject> collision_objects;
      //   add_object (a, collision_objects, name_obj, collision_object);
      //   planning_scene_interface.addCollisionObjects(collision_objects);
      // ROS_INFO("05");
      //   p1.position.x = a[3];
      //   p1.position.y = a[4];
      //   p1.position.z = a[5];
      //   p1.orientation.x = a[6];
      //   p1.orientation.y = a[7];
      //   p1.orientation.z = a[8];
      //   p1.orientation.w = a[9];

      //   p2.position.x = -0.3;
      //   p2.position.y = -0.3;
      //   p2.position.z = 0;
      //   p2.orientation.x = 0;
      //   p2.orientation.y = 0;
      //   p2.orientation.z = 0;
      //   p2.orientation.w = 1;
      //   ros::ServiceClient client = node_handle.serviceClient<msdp::GoalPoses>("/GoalPoses");
      //   srv.request.p1 = p1;
      //   srv.request.p2 = p2;
      //   if (client.call(srv))
      //   {
      //     ROS_INFO("Send two grasping points to client");
      //   }
      //   else
      //   {
      //     ROS_ERROR("Failed to call service GoalPoses");
      //   }

      R.sleep();
    }
  }
};
int main(int argc, char **argv)
{
  ros::init(argc, argv, "add_object_node");
  ros::AsyncSpinner spinner(1);
  spinner.start();
  ROS_INFO("0");
  Add_object_class obj;
  ROS_INFO("1");
  obj.spin();
  // // obj.IvansNodeCallback;
  ROS_INFO("2");
  ros::shutdown();

  return 0;
}
