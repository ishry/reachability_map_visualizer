#include <reachability_map_visualizer/reachability_map_visualizer.h>
#include <choreonoid_viewer/choreonoid_viewer.h>
#include <cnoid/Body>
#include <cnoid/BodyLoader>
#include <iostream>
#include <ros/package.h>

namespace reachability_map_visualizer_sample
{
  void manta_tail()
  {
    std::shared_ptr<choreonoid_viewer::Viewer> viewer = std::make_shared<choreonoid_viewer::Viewer>();
    std::shared_ptr<reachability_map_visualizer::ReachabilityMapParam> param = std::make_shared<reachability_map_visualizer::ReachabilityMapParam>();

    cnoid::BodyLoader bodyLoader;
    param->robot = bodyLoader.load(ros::package::getPath("manta_models") + "/MANTA/MANTA_HR_TAILmain.wrl");
    if (!(param->robot))
      std::cerr << "!robot" << std::endl;

    param->robot->rootLink()->p() = cnoid::Vector3(0, 0, 0.0);
    param->robot->rootLink()->v().setZero();
    param->robot->rootLink()->R() = cnoid::Matrix3::Identity();
    param->robot->rootLink()->w().setZero();
    std::vector<double> reset_manip_pose{
      0, 0,                               // wheel
      0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, // joint
      0, 0,                               // hand       
    };

    for (int j = 0; j < param->robot->numJoints(); ++j)
    {
      param->robot->joint(j)->q() = reset_manip_pose[j];
    }
    param->robot->calcForwardKinematics();
    param->robot->calcCenterOfMass();
    // joint limit
    std::vector<std::shared_ptr<ik_constraint2::IKConstraint>> constraints;
    for (int i = 0; i < param->robot->numJoints(); i++)
    {
      std::shared_ptr<ik_constraint2::JointLimitConstraint> constraint = std::make_shared<ik_constraint2::JointLimitConstraint>();
      constraint->joint() = param->robot->joint(i);
      constraints.push_back(constraint);
    }
    param->constraints.push_back(constraints);
    param->variables.push_back(param->robot->joint("TAIL_JOINT0"));
    param->variables.push_back(param->robot->joint("TAIL_JOINT1"));
    param->variables.push_back(param->robot->joint("TAIL_JOINT2"));
    param->variables.push_back(param->robot->joint("TAIL_JOINT3"));
    param->variables.push_back(param->robot->joint("TAIL_JOINT4"));
    param->variables.push_back(param->robot->joint("TAIL_JOINT5"));
    param->variables.push_back(param->robot->joint("TAIL_JOINT6"));
    param->variables.push_back(param->robot->joint("TAIL_JOINT7"));
    param->variables.push_back(param->robot->joint("TAIL_JOINT8"));
    param->variables.push_back(param->robot->joint("TAIL_JOINT9"));
    param->variables.push_back(param->robot->joint("TAIL_JOINT10"));
    param->variables.push_back(param->robot->joint("TAIL_JOINT11"));
    reachability_map_visualizer::EndEffector ee;
    ee.parent = param->robot->link("TAIL_JOINT11");
    ee.localPose.translation() = cnoid::Vector3(0.0, -0.25, 0.0);
    ee.localPose.linear() = cnoid::rotFromRpy(3.14, 0.0, -1.57);
    param->endEffectors.push_back(ee);
    param->posResolution = 0.2;
    param->pikParam.maxIteration = 30;
    param->testPerGrid = 10;
    param->origin = cnoid::Vector3(0.0, 0.0, 1); // IKが解けない時に無駄な計算をしてしまうので、ぎりぎりのサイズにしたほうが速い
    param->size = cnoid::Vector3(3.5, 3.5, 2.5);
    param->weight[5] = 0.0;
    std::shared_ptr<reachability_map_visualizer::ReachabilityMap> map = std::make_shared<reachability_map_visualizer::ReachabilityMap>();
    reachability_map_visualizer::createMap(param, map,5);

    reachability_map_visualizer::writeMap(ros::package::getPath("reachability_map_visualizer_sample") + "/config/manta_tail.yaml", map);
    viewer->objects(param->robot);
    reachability_map_visualizer::visualizeMap(map, viewer);
    viewer->drawObjects();
    while(true);
  }
}
