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

#include "the_mainest/GoalPoses.h"
#include "the_mainest/GetPNPPoses.h"


static const std::string PLANNING_GROUP = "manipulator";

class Add_object_class
{
private:


  double b[11];
  ros::NodeHandle node_handle;
  ros::Subscriber sub;
  Eigen::Quaternionf qq;
  geometry_msgs::PointStamped base_point1;
  geometry_msgs::PointStamped obb_point;
  geometry_msgs::QuaternionStamped base_orient;
  geometry_msgs::QuaternionStamped obb_orient;
  moveit_msgs::CollisionObject collision_object;
  std::vector<moveit_msgs::CollisionObject> collision_objects;

  msdp::GoalPoses srv;
  geometry_msgs::Pose p1, p2;

  moveit::planning_interface::PlanningSceneInterface planning_scene_interface;
  moveit::planning_interface::MoveGroupInterface *move_group_interface_ptr;
  const moveit::core::JointModelGroup *joint_model_group;

public:
  Add_object_class() : node_handle("~")
  {

    move_group_interface_ptr = new moveit::planning_interface::MoveGroupInterface(PLANNING_GROUP);
    joint_model_group = move_group_interface_ptr->getCurrentState()->getJointModelGroup(PLANNING_GROUP);

    // sub = node_handle.subscribe("/obb_array", 10, &Add_object_class::IvansNodeCallback, this);
    ros::ServiceClient client = node_handle.serviceClient<msdp::GoalPoses>("GoalPoses");

  }

  ~Add_object_class()
  {
    delete move_group_interface_ptr;
  }

  int id_max_vector(double v1, double v2, double v3)
  {
    int id_max = 0;
    double mas[2];
    mas[0] = v1;
    mas[1] = v2;
    mas[2] = v3;
    for (int i = 0; i < 2; i++)
    {
      if (mas[i] > mas[id_max])
      {
        id_max = i;
        i = 0;
      }
    }
    return id_max;
  }

  double angleBetweenVectors(Eigen::Vector3f a, Eigen::Vector3f b) 
  {
    double angle = 0.0;
    angle = std::atan2(a.cross(b).norm(), a.dot(b));
    return angle;

  }

  void add_object(double a[], std::string name_obj)
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
    box_pose.position.x = a[3];
    box_pose.position.y = a[4];
    box_pose.position.z = a[5];
    box_pose.orientation.x = a[6];
    box_pose.orientation.y = a[7];
    box_pose.orientation.z = a[8];
    box_pose.orientation.w = a[9];

    collision_object.primitives.push_back(primitive);
    collision_object.primitive_poses.push_back(box_pose);
    collision_object.operation = collision_object.ADD;

    collision_objects.push_back(collision_object);

    // Now, let's add the collision object into the world
    // (using a vector that could contain additional objects)
  }

  void get_transform(const tf::TransformListener &listener)
  {
    //we'll create a point in the base_laser frame that we'd like to transform to the base_link frame
    tf::StampedTransform transform;
    try{
      listener.lookupTransform("/tool0", "/base",   
                               ros::Time(0), transform);
    }
    catch (tf::TransformException ex){
      ROS_ERROR("%s",ex.what());
      ros::Duration(1.0).sleep();
    }
    tf::Quaternion q = transform.getRotation();

    qq.x() = q.x();
    qq.y() = q.y();
    qq.z() = q.z();
  }

  void transformPoint(const tf::TransformListener &listener, geometry_msgs::PointStamped &base_point, geometry_msgs::PointStamped &wrist3_point)
  {
    //we'll create a point in the base_laser frame that we'd like to transform to the base_link frame

    try
    {
      ros::Time now = ros::Time(0);
      // ros::Time now = ros::Time::now();
      wrist3_point.header.stamp = now;

      listener.waitForTransform( "/base", "/camera_color_optical_frame",
                                now, ros::Duration(3.0));
      listener.transformPoint("/base_link", wrist3_point, base_point);

      ROS_INFO("camera_color_optical_frame: (%.5f, %.5f. %.5f) -----> base: (%.5f, %.5f, %.5f) at time %.5f",
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
      listener.waitForTransform("/camera_color_optical_frame", "/base_link", 
                                now, ros::Duration(3.0));
      listener.transformQuaternion("/base_link", obj, base);

      ROS_INFO("obj:  (%.5f, %.5f. %.5f) -----> base_link: (%.5f, %.5f, %.5f) at time %.5f",
               obj.quaternion.x, obj.quaternion.y, obj.quaternion.z,
               base.quaternion.x, base.quaternion.y, base.quaternion.z, base.header.stamp.toSec());
    }
    catch (tf::TransformException &ex)
    {
      ROS_ERROR("Received an exception trying to transform a quaternion from \"obj\" to \"base\": %s", ex.what());
      ROS_INFO("obj:  (%.5f, %.5f. %.5f) -----> base_link: (%.5f, %.5f, %.5f) at time %.5f",
               obj.quaternion.x, obj.quaternion.y, obj.quaternion.z,
               base.quaternion.x, base.quaternion.y, base.quaternion.z, base.header.stamp.toSec());
    }
  }

  void IvansNodeCallback(const std_msgs::Float32MultiArray::ConstPtr &msg)
  {

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
    obb_point.header.stamp = ros::Time::now();


    // //just an arbitrary point in space
    obb_point.point.x = b[6];
    obb_point.point.y = b[7];
    obb_point.point.z = b[8];

    ROS_INFO("now im before transofrm");
    transformPoint(listener1, base_point1, obb_point);
    a[3] = base_point1.point.x;
    a[4] = base_point1.point.y;
    a[5] = base_point1.point.z;
    ROS_INFO("now im after transofrm");
    // // Ориентация

    // geometry_msgs::Pose gripper_pose;
    // gripper_pose.position.x = base_point1.point.x;
    // gripper_pose.position.y = base_point1.point.y;
    // gripper_pose.position.z = base_point1.point.z;



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

    transformQuaternion(listener1, base_orient, obb_orient);

    a[6] = base_orient.quaternion.x;
    a[7] = base_orient.quaternion.y;
    a[8] = base_orient.quaternion.z;
    a[9] = base_orient.quaternion.w;
    Eigen::Quaternionf quat1;
    quat1.x() = base_orient.quaternion.x;
    quat1.y() = base_orient.quaternion.y;
    quat1.z() = base_orient.quaternion.z;
    quat1.w() = base_orient.quaternion.w;
    Eigen::Matrix3f R = quat1.normalized().toRotationMatrix();

    Eigen::Vector3f v1(R.col(0));
    Eigen::Vector3f v2(R.col(1));
    Eigen::Vector3f v3(R.col(2));

    double v1_length, v2_length, v3_length;
    v1_length = sqrt(v1(0)*v1(0) + v1(1)*v1(1));
    v2_length = sqrt(v2(0)*v2(0) + v2(1)*v2(1));
    v3_length = sqrt(v3(0)*v3(0) + v3(1)*v3(1));

    int id_max = id_max_vector(v1_length, v2_length, v3_length);

    Eigen::Vector3f max_vector(R.col(id_max));

    // Eigen::Vector3f vectr(0, 1, 0);
    // Eigen::Vector3f vectr_tr(vectr.transpose());
    // Eigen::Vector3f first_vector;
    // first_vector = (max_vector*vectr);

    get_transform(listener1);
    Eigen::Matrix3f rot_to_tool = qq.normalized().toRotationMatrix();
    Eigen::Vector3f tool_rot(0, 1, 0);
    double angle;
    angle = angleBetweenVectors(max_vector, rot_to_tool*tool_rot);
    std::cout<<std::endl<<angle<<std::endl;


    collision_object.header.frame_id = move_group_interface_ptr->getPlanningFrame();

    moveit_visual_tools::MoveItVisualTools visual_tools("base"); //shoulder_link
    visual_tools.deleteAllMarkers();
    visual_tools.loadRemoteControl();

    add_object(a, name_obj);
    planning_scene_interface.addCollisionObjects(collision_objects);
    Eigen::Quaternionf quat2;
    quat2.x() = 0;
    quat2.y() = 0;
    quat2.z() = 1;
    quat2.w() = angle/M_PI;
    Eigen::Matrix3f R_last = quat2.normalized().toRotationMatrix();

    Eigen::Matrix3f rot_naoborot;
    rot_naoborot << -1, 0, 0, 0, 1, 0, 0, 0, -1;
    // rot_naoborot.col(0) = (-1, 0, 0);
    // rot_naoborot.col(1) = (0, 1, 0);
    // rot_naoborot.col(2) = (0, 0, -1);

    Eigen::Matrix3f rot_naoborot2=rot_naoborot*R_last;
    Eigen::Quaternionf quat3(rot_naoborot2);

    p1.position.x = a[3];
    p1.position.y = a[4];
    p1.position.z = a[5];
    // p1.orientation.x = a[6];
    // p1.orientation.y = a[7];
    // p1.orientation.z = a[8];
    // p1.orientation.w = a[9];
    p1.orientation.x = quat3.x();
    p1.orientation.y = quat3.y();
    p1.orientation.z = quat3.z();
    p1.orientation.w = quat3.w();

    p2.position.x = -0.3;
    p2.position.y = -0.3;
    p2.position.z = 0;
    p2.orientation.x = 0;
    p2.orientation.y = 0;
    p2.orientation.z = 0;
    p2.orientation.w = 1;

    ros::ServiceClient client = node_handle.serviceClient<msdp::GoalPoses>("/GoalPoses");
    srv.request.p1 = p1;
    srv.request.p2 = p2;
    if (client.call(srv))
    {
      ROS_INFO("Send two grasping points to client");
    }
    else
    {
      ROS_ERROR("Failed to call service GoalPoses");
    }
  }

  bool get_pnp_poses_handler(the_mainest::GetPNPPoses::ConstPtr& req, the_mainest::GetPNPPoses::ConstPtr& res) {

    for (int i = 0; i <= 11; i++) {
      b[i] = req->data[i];
      std::cout << b[i] << std::endl;
    }

    ROS_INFO("Data from topic: (%.5f, %.5f, %.5f) -----> : (%.5f, %.5f, %.5f) -----> : (%.5f, %.5f, %.5f)-----> : (%.5f, %.5f, %.5f)",
             b[0], b[1], b[2],
             b[3], b[4], b[5],
             b[6], b[7], b[8],
             b[9], b[10], b[11]);

    std::string name_obj;
    name_obj = "obj";

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
    // we'll just use the most recent transform available for our simple example
    obb_point.header.stamp = ros::Time::now();

    // just an arbitrary point in space
    obb_point.point.x = b[6];
    obb_point.point.y = b[7];
    obb_point.point.z = b[8];

    transformPoint(listener1, base_point1, obb_point);
    a[3] = base_point1.point.x;
    a[4] = base_point1.point.y;
    a[5] = base_point1.point.z;

    // Ориентация
    // geometry_msgs::Pose gripper_pose;
    // gripper_pose.position.x = base_point1.point.x;
    // gripper_pose.position.y = base_point1.point.y;
    // gripper_pose.position.z = base_point1.point.z;
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

    transformQuaternion(listener1, base_orient, obb_orient);

    a[6] = base_orient.quaternion.x;
    a[7] = base_orient.quaternion.y;
    a[8] = base_orient.quaternion.z;
    a[9] = base_orient.quaternion.w;

    Eigen::Quaternionf quat1;
    quat1.x() = base_orient.quaternion.x;
    quat1.y() = base_orient.quaternion.y;
    quat1.z() = base_orient.quaternion.z;
    quat1.w() = base_orient.quaternion.w;
    Eigen::Matrix3f R = quat1.normalized().toRotationMatrix();

    Eigen::Vector3f v1(R.col(0));
    Eigen::Vector3f v2(R.col(1));
    Eigen::Vector3f v3(R.col(2));

    double v1_length, v2_length, v3_length;
    v1_length = sqrt(v1(0)*v1(0) + v1(1)*v1(1));
    v2_length = sqrt(v2(0)*v2(0) + v2(1)*v2(1));
    v3_length = sqrt(v3(0)*v3(0) + v3(1)*v3(1));

    int id_max = id_max_vector(v1_length, v2_length, v3_length);

    Eigen::Vector3f max_vector(R.col(id_max));

    // Eigen::Vector3f vectr(0, 1, 0);
    // Eigen::Vector3f vectr_tr(vectr.transpose());
    // Eigen::Vector3f first_vector;
    // first_vector = (max_vector*vectr);

    get_transform(listener1);
    Eigen::Matrix3f rot_to_tool = qq.normalized().toRotationMatrix();
    Eigen::Vector3f tool_rot(0, 1, 0);
    double angle;
    angle = angleBetweenVectors(max_vector, rot_to_tool*tool_rot);
    std::cout<<std::endl<<angle<<std::endl;

    collision_object.header.frame_id = move_group_interface_ptr->getPlanningFrame();

    moveit_visual_tools::MoveItVisualTools visual_tools("base"); //shoulder_link
    visual_tools.deleteAllMarkers();
    visual_tools.loadRemoteControl();

    add_object(a, name_obj);
    planning_scene_interface.addCollisionObjects(collision_objects);

    Eigen::Quaternionf quat2;
    quat2.x() = 0;
    quat2.y() = 0;
    quat2.z() = 1;
    quat2.w() = angle/M_PI;
    Eigen::Matrix3f R_last = quat2.normalized().toRotationMatrix();

    Eigen::Matrix3f rot_naoborot;
    rot_naoborot << -1, 0, 0, 0, 1, 0, 0, 0, -1;
    // rot_naoborot.col(0) = (-1, 0, 0);
    // rot_naoborot.col(1) = (0, 1, 0);
    // rot_naoborot.col(2) = (0, 0, -1);

    Eigen::Matrix3f rot_naoborot2 = rot_naoborot * R_last;
    Eigen::Quaternionf quat3(rot_naoborot2);

    p1.position.x = a[3];
    p1.position.y = a[4];
    p1.position.z = a[5];
    // p1.orientation.x = a[6];
    // p1.orientation.y = a[7];
    // p1.orientation.z = a[8];
    // p1.orientation.w = a[9];
    p1.orientation.x = quat3.x();
    p1.orientation.y = quat3.y();
    p1.orientation.z = quat3.z();
    p1.orientation.w = quat3.w();

    p2.position.x = -0.3;
    p2.position.y = -0.3;
    p2.position.z = 0;
    p2.orientation.x = 0;
    p2.orientation.y = 0;
    p2.orientation.z = 0;
    p2.orientation.w = 1;

    res->p1 = p1;
    res->p2 = p2;
    return true;
  }

};

int main(int argc, char **argv)
{
  ros::init(argc, argv, "add_object_node");

  ros::AsyncSpinner spinner(2);
  spinner.start();

  Add_object_class obj;
  ros::waitForShutdown()

  ros::shutdown();
  return 0;
}
