#include <reachability_map_visualizer/reachability_map_visualizer.h>
#include <choreonoid_viewer/choreonoid_viewer.h>
#include <cnoid/Body>
#include <cnoid/BodyLoader>
#include <iostream>
#include <ros/package.h>

namespace reachability_map_visualizer_sample{
  void visualize(){
    std::shared_ptr<choreonoid_viewer::Viewer> viewer = std::make_shared<choreonoid_viewer::Viewer>();
    cnoid::BodyLoader bodyLoader;
    cnoid::BodyPtr robot = bodyLoader.load(ros::package::getPath("manta_models") + "/MANTA/MANTA_HR_TAILmain.wrl");
    if(!robot) std::cerr << "!robot" << std::endl;

    robot->rootLink()->p() = cnoid::Vector3(0,0,0.0);
    robot->rootLink()->v().setZero();
    robot->rootLink()->R() = cnoid::Matrix3::Identity();
    robot->rootLink()->w().setZero();
    std::vector<double> reset_manip_pose{
      0, 0, // wheel
      0, 1.57, 0, 1.57, 1.535, 0, 0, 0, 0, 0, 1.535, 0, // joint
      0, 0, // hand
        };

    for(int j=0; j < robot->numJoints(); ++j){
      robot->joint(j)->q() = reset_manip_pose[j];
    }
    robot->calcForwardKinematics();
    robot->calcCenterOfMass();
    std::shared_ptr<reachability_map_visualizer::ReachabilityMap> map = std::make_shared<reachability_map_visualizer::ReachabilityMap>();

    reachability_map_visualizer::readMap(ros::package::getPath("reachability_map_visualizer_sample") + "/config/manta_tail.yaml",map);
    //map->boxSize = cnoid::Vector3(map->posResolution, map->posResolution, 0.001);
    map->boxSize = cnoid::Vector3(0.05, 0.05, 0.05);
    viewer->objects(robot);
    reachability_map_visualizer::visualizeMap(map, viewer);
    viewer->drawObjects();
    while(true);
  }
}
