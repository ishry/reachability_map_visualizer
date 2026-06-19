#include <reachability_map_visualizer/reachability_map_visualizer.h>
#include <choreonoid_viewer/choreonoid_viewer.h>
#include <cnoid/Body>
#include <cnoid/BodyLoader>
#include <iostream>
#include <ros/package.h>

namespace reachability_map_visualizer_sample
{
  std::shared_ptr<reachability_map_visualizer::ReachabilityMapParam> create_manta_tail_param()
  {
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
    // IKで使う変数
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
    
    // トルクリミット
    param->enable_torque_check = false;
    param->torque_limits.push_back(reachability_map_visualizer::TorqueLimit(param->robot->joint("TAIL_JOINT0"), 192.0));
    param->torque_limits.push_back(reachability_map_visualizer::TorqueLimit(param->robot->joint("TAIL_JOINT1"), 192.0));
    param->torque_limits.push_back(reachability_map_visualizer::TorqueLimit(param->robot->joint("TAIL_JOINT2"), 96.0));
    param->torque_limits.push_back(reachability_map_visualizer::TorqueLimit(param->robot->joint("TAIL_JOINT3"), 96.0));
    param->torque_limits.push_back(reachability_map_visualizer::TorqueLimit(param->robot->joint("TAIL_JOINT4"), 96.0));
    param->torque_limits.push_back(reachability_map_visualizer::TorqueLimit(param->robot->joint("TAIL_JOINT5"), 96.0));
    param->torque_limits.push_back(reachability_map_visualizer::TorqueLimit(param->robot->joint("TAIL_JOINT6"), 96.0));
    param->torque_limits.push_back(reachability_map_visualizer::TorqueLimit(param->robot->joint("TAIL_JOINT7"), 96.0));
    param->torque_limits.push_back(reachability_map_visualizer::TorqueLimit(param->robot->joint("TAIL_JOINT8"), 96.0));
    param->torque_limits.push_back(reachability_map_visualizer::TorqueLimit(param->robot->joint("TAIL_JOINT9"), 96.0));
    param->torque_limits.push_back(reachability_map_visualizer::TorqueLimit(param->robot->joint("TAIL_JOINT10"), 96.0));
    param->torque_limits.push_back(reachability_map_visualizer::TorqueLimit(param->robot->joint("TAIL_JOINT11"), 96.0));

    // 重心が支持多角形内にあるかをチェック
    bool enable_support_polygon_check = true;
    param->enable_support_polygon_check = enable_support_polygon_check;
    param->support_points.push_back(reachability_map_visualizer::SupportPoint(param->robot->link("R_WHEEL_JOINT"), cnoid::Vector3::Zero()));
    param->support_points.push_back(reachability_map_visualizer::SupportPoint(param->robot->link("L_WHEEL_JOINT"), cnoid::Vector3::Zero()));
    param->support_points.push_back(reachability_map_visualizer::SupportPoint(param->robot->link("OMNI_WHEEL_JOINT"), cnoid::Vector3::Zero()));
    
    reachability_map_visualizer::EndEffector ee;
    ee.parent = param->robot->link("TAIL_JOINT11");
    ee.localPose.translation() = cnoid::Vector3(0.0, -0.25, 0.0);
    ee.localPose.linear() = cnoid::rotFromRpy(3.14, 0.0, -1.57);
    param->endEffectors.push_back(ee);
    param->posResolution = 0.1; // グリッドの大きさ
    param->pikParam.maxIteration = 30;
    param->testPerGrid = 10; //各グリッドの計算回数
    param->origin = cnoid::Vector3(-0.5, 0.0, 1.0); // IKが解けない時に無駄な計算をしてしまうので、ぎりぎりのサイズにしたほうが速い
    param->size = cnoid::Vector3(3.2, 3.2, 2);
    param->weight[5] = 0.0;
    return param;
  }

  void manta_tail()
  {
    std::shared_ptr<choreonoid_viewer::Viewer> viewer = std::make_shared<choreonoid_viewer::Viewer>();
    std::shared_ptr<reachability_map_visualizer::ReachabilityMapParam> param = create_manta_tail_param();
    std::shared_ptr<reachability_map_visualizer::ReachabilityMap> map = std::make_shared<reachability_map_visualizer::ReachabilityMap>();
    reachability_map_visualizer::createMap(param, map,8); //並列数

    reachability_map_visualizer::writeMap(ros::package::getPath("reachability_map_visualizer_sample") + "/config/manta_tail.yaml", map);
    viewer->objects(param->robot);
    reachability_map_visualizer::visualizeMap(map, viewer);
    viewer->drawObjects();
    while(true);
  }
}
