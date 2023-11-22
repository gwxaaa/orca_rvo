
#include "ros/ros.h"
#include "gazebo_msgs/ModelStates.h"
#include "gazebo_msgs/ModelState.h"
#include "std_msgs/String.h"
#include "ModelSubPub.h"
#include "Agent.h"
#include "Neighbor.h"
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
        goal_pose(goal_pose)

  {
    // 初始化ROS节点
    ros::NodeHandle nh;
    //  receivedInfo_=false;
    target_model_ = modelName;
    model_states_sub_ = nh.subscribe("/gazebo/model_states", 10, &ModelSubPub::modelStatesCallback, this);
    // 发布信息
    model_states_pub_ = nh.advertise<gazebo_msgs::ModelState>("/gazebo/set_model_state", 10);
  }
  // 回调函数，处理模型状态信息
  void ModelSubPub::modelStatesCallback(const gazebo_msgs::ModelStates::ConstPtr &msg)
  {
    other_models_states.clear();
    // 存储特定模型的信息
    gazebo_msgs::ModelState target_model_state;
    // 遍历所有模型
    for (size_t i = 0; i < msg->name.size(); ++i)
    {
      // 找到特定目标模型
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
    // std::string agentname = target_model_;
    // agentpose = target_model_state.pose;
    // agenttwist = target_model_state.twist;
    // targetpose.position = goal_pose.position;
    RVO::Neighbor neighborobject(*this);
    // neighborobject.computeNeighborInformation();
    // // 获取计算后的邻居信息
    std::vector<RVO::Agent *> agentNeighbors_ = neighborobject.getAgentNeighbors();
    std::vector<RVO::Obstacle *> obstacleNeighbors_ = neighborobject.getObstacleNeighbors();

    RVO::Agent agent(agentpose, agenttwist, time, goal_pose, maxSpeed_, neighborDistance_, timeHorizon_, other_models_states, radius_);
    std::string agentname = target_model_;
    agentpose = target_model_state.pose;
    agenttwist = target_model_state.twist;
    targetpose.position = goal_pose.position;
    gazebo_msgs::ModelState new_velocity = agent.computeNewVelocity(agentpose, agenttwist, targetpose, targettwist,
                                                                    agentNeighbors_, obstacleNeighbors_, time);
    // agent是现在运行的模型，target是目标点
    gazebo_msgs::ModelState model_state;
    model_state.model_name = agentname;
    model_state.pose = new_velocity.pose;
    model_states_pub_.publish(model_state);
  }

  std::vector<gazebo_msgs::ModelState> ModelSubPub::getothermodels() const
  {
    return other_models_states;
  };
}
