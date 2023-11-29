
#include "ros/ros.h"
#include "gazebo_msgs/ModelStates.h"
#include "gazebo_msgs/ModelState.h"
#include "std_msgs/String.h"
#include "ModelSubPub.h"
#include "Agent.h"
#include "Neighbor.h"
#include <cmath>
#include <geometry_msgs/PoseStamped.h>
#include "nav_msgs/Path.h"
namespace RVO
{
  ModelSubPub::ModelSubPub(const std::string &modelName, double time, gazebo_msgs::ModelState target_model_state,
                           geometry_msgs::Pose goal_pose, double maxSpeed_, double neighborDistance_, double timeHorizon_, double radius_)
      : modelName_(modelName),
        time(time),
        maxSpeed_(maxSpeed_),
        neighborDistance_(neighborDistance_),
        timeHorizon_(timeHorizon_),
        radius_(radius_),
        goal_pose(goal_pose),
        target_model_state(target_model_state)
  {
    // 初始化ROS节点
    ros::NodeHandle nh;
    target_model_ = modelName;
    model_states_sub_ = nh.subscribe("/gazebo/model_states", 10, &ModelSubPub::modelStatesCallback, this);
    model_states_pub_ = nh.advertise<gazebo_msgs::ModelState>("/gazebo/set_model_state", 10);
    pose_stamped_pub_ = nh.advertise<geometry_msgs::PoseStamped>("/pose_stamped_topic", 10); // 新增的发布器
    path_pub_ = nh.advertise<nav_msgs::Path>("/path_topic", 10);                             // 新增的发布器
  }
  // 回调函数，处理模型状态信息
  void ModelSubPub::modelStatesCallback(const gazebo_msgs::ModelStates::ConstPtr &msg)
  {
    other_models_states.clear();
    // gazebo_msgs::ModelState target_model_state;
    // 遍历所有模型
    for (size_t i = 0; i < msg->name.size(); ++i)
    {
      if (msg->name[i] == target_model_)
      {
        // 存储特定目标模型的信
        target_model_state.model_name = msg->name[i];
        target_model_state.pose = msg->pose[i];
        target_model_state.twist = msg->twist[i];
      }
      else if (msg->name[i] != "ground_plane")
      {
        // 存储其他模型的信息
        gazebo_msgs::ModelState other_model_state;
        other_model_state.model_name = msg->name[i];
        other_model_state.pose = msg->pose[i];
        other_model_state.twist = msg->twist[i];
        other_models_states.push_back(other_model_state);
      }
    }
    std::string agentname = target_model_;
    agentpose = target_model_state.pose;
    agenttwist = target_model_state.twist;
    // 格式转化
    Vector2 agentPosition(agentpose.position.x, agentpose.position.y);
    double deltaTheta = agenttwist.angular.z * time;
    // 根据新的朝向角度和线速度计算速度向量
    double velocityX = agenttwist.linear.x * cos(deltaTheta);
    double velocityY = agenttwist.linear.x * sin(deltaTheta);
    Vector2 agentVelocity(velocityX, velocityY);
    Vector2 goalPosition(goal_pose.position.x, goal_pose.position.y);
    // 计算邻居信息
    RVO::Neighbor neighborobject(*this);
    //  获取计算后的邻居信息
    std::vector<RVO::Agent *> agentNeighbors_ = neighborobject.getAgentNeighbors();
    std::vector<RVO::Obstacle *> obstacleNeighbors_ = neighborobject.getObstacleNeighbors();
    // 对目标信息进行ORCA算法计算
    RVO::Agent agent(agentPosition, agentVelocity, goalPosition, time, maxSpeed_,
                     neighborDistance_, timeHorizon_, other_models_states, radius_);
    Vector2 newVelocity = agent.computeNewVelocity(agentPosition, agentVelocity,
                                                   goalPosition, agentNeighbors_, obstacleNeighbors_, time);
    // 只输出计算后的速度，相关位置移动信息在下面计算
    if (std::isnan(newVelocity.x()) || std::isnan(newVelocity.y()))
    {
      new_pose.position.x = agentPosition.x();
      new_pose.position.y = agentPosition.y();
      std::cout << "New velocity contains NaN. Keeping original position." << std::endl;
    }
    else
    {
      new_pose.position.x = agentPosition.x() + newVelocity.x() * time;
      new_pose.position.y = agentPosition.y() + newVelocity.y() * time;
    }
    gazebo_msgs::ModelState model_state;
    model_state.model_name = agentname;
    model_state.pose = new_pose;
    model_states_pub_.publish(model_state);

    new_poses.push_back(new_pose);
    std::size_t size=new_poses.size();
    std::cout<<"a111size:"<<size<<std::endl;

    // 发布pose信息
    geometry_msgs::PoseStamped pose_stamped_msg;
    pose_stamped_msg.header.stamp = ros::Time::now(); // 使用当前时间作为时间戳
    pose_stamped_msg.header.frame_id = "map";
    pose_stamped_msg.pose.position.x = new_pose.position.x;
    pose_stamped_msg.pose.position.y = new_pose.position.y;
    pose_stamped_msg.pose.orientation = new_pose.orientation;
    // 发布 geometry_msgs::PoseStamped 类型的消息
    pose_stamped_pub_.publish(pose_stamped_msg);

    // 发布path信息
    nav_msgs::Path path_msg;
    path_msg.header.stamp = ros::Time::now();
    path_msg.header.frame_id = "map"; // 设置路径消息的坐标系
    // 需要设置较多的信息
    for (int i = 0; i < new_poses.size(); ++i)
    {
      // 添加路径点到路径消息中
      geometry_msgs::PoseStamped pose;
      pose.header.stamp = ros::Time::now();
      pose.header.frame_id = "map"; // 设置路径点的坐标系
      pose.pose = new_poses[i];
      // pose.pose.position.x = 0.1 * i;
      // pose.pose.position.y = 0.1 * i;
      // pose.pose.position.z = 0.5;
      // pose.pose.orientation.x = 0;
      // pose.pose.orientation.y = 0;
      // pose.pose.orientation.z = 0;
      // pose.pose.orientation.w = 1;
      // 信息传输成功，但是值不是有效值；那么应该怎么设置
      std::cout << "11111Moved to new position: x=" << new_poses[i].position.x << ", y=" << new_poses[i].position.y << std::endl;
      // 一直都是同一个值

      path_msg.poses.push_back(pose); // 将路径点添加到路径消息中
    }
    path_pub_.publish(path_msg); // 发布路径消息
  }
  std::vector<gazebo_msgs::ModelState> ModelSubPub::getothermodels() const
  {
    return other_models_states;
  };
}
