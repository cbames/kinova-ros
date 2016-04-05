#include <ros/ros.h>
#include <iostream>
#include <fstream>
#include <kdl/tree.hpp>
#include <kdl/frames.hpp>
#include <kdl/segment.hpp>
#include <kdl/chain.hpp>
#include <kdl_parser/kdl_parser.hpp>
#include <urdf/model.h>
#include <kdl/chain.hpp>
#include <std_msgs/Float64MultiArray.h>
#include <std_msgs/Float64.h>
#include <sensor_msgs/JointState.h>
#include <kdl/chainidsolver_recursive_newton_euler.hpp>

KDL::Chain chain; 
ros::Publisher joint_efforts_gc;
 
void gcCallback(const sensor_msgs::JointState::ConstPtr& msg)
{
  KDL::Vector gravity(0,0,-9.81); 
  KDL::ChainIdSolver_RNE id_solver(chain, gravity); 
  
  KDL::JntArray q(chain.getNrOfJoints());
  KDL::JntArray qd(chain.getNrOfJoints()); 
  KDL::JntArray qdd(chain.getNrOfJoints()); 

  KDL::Wrenches external_wrenches; 

  KDL::JntArray tau_gc(chain.getNrOfJoints()); 

  std::cout << "chain size:" << chain.getNrOfJoints() << std::endl; 

  for(int i=0; i<chain.getNrOfJoints(); i++)
  {
   q(i) = msg->position[i]; 
   qd(i) = 0.0; //msg->velocity[i]; 
   qdd(i) = 0.0; 
   external_wrenches.push_back(KDL::Wrench()); 
  }


 int ret = id_solver.CartToJnt(q,qd,qdd,external_wrenches, tau_gc);
 std::cout << "ret:" << ret << std::endl; 
 if (ret < 0) ROS_ERROR("KDL: inverse dynamics ERROR");
 std_msgs::Float64MultiArray gc_torques; 
 gc_torques.data.clear(); 

 double sign[] = {1.0,-1.0,1.0,1.0,1.0,1.0};

 for(int i=0; i<chain.getNrOfJoints(); i++)
 {
  std::cout << msg->name[i]<<":"<< tau_gc(i) << std::endl; 
  std_msgs::Float64 tau_gc_datum; 
//tau_gc_datum.data = ;
  gc_torques.data.push_back((double)(msg->velocity[i]+sign[i]*tau_gc(i)));
 }
 
 joint_efforts_gc.publish(gc_torques); 
}



int main ( int argc, char** argv)
{
  ros::init(argc, argv, "gravity_compensation");
  ros::NodeHandle nh;

  std::string robot_name, chain_start, chain_end; 
  nh.param("robot", robot_name, std::string("jaco2"));
  nh.param("chain_start", chain_start, std::string("jaco2_link_base"));
  nh.param("chain_end", chain_end, std::string("jaco2_link_hand"));

  KDL::Tree tree;
  urdf::Model robot_model;
  if (robot_model.initParam("/robot_description"))
    {
      if (!kdl_parser::treeFromUrdfModel(robot_model, tree))
        ROS_FATAL("Failed to extract kdl tree from xml robot description");
      KDL::SegmentMap segs = tree.getSegments();
      KDL::SegmentMap::iterator sit = segs.begin();
      for(; sit!=segs.end(); ++sit)
        {
          ROS_INFO("Read segment %s", sit->first.c_str());
        }

      if(!tree.getChain(chain_start, chain_end, chain))
        ROS_FATAL("Couldn't find chain %s to %s",chain_start.c_str(),chain_end.c_str());
    }
  else
    ROS_FATAL("/robot_description does not exist!");

  if (chain.getNrOfJoints()==0)
    return 1;


  joint_efforts_gc = nh.advertise<std_msgs::Float64MultiArray>("joint_efforts_gc",1);
  ros::Subscriber joint_efforts = nh.subscribe("joint_states",1, gcCallback); 

  ros::spin(); 

  return 0; 
}

