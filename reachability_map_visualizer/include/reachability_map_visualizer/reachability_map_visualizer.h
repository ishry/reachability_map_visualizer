#ifndef REACHABILITY_MAP_VISUALIZER_H
#define REACHABILITY_MAP_VISUALIZER_H

#include <choreonoid_viewer/choreonoid_viewer.h>
#include <prioritized_inverse_kinematics_solver2/prioritized_inverse_kinematics_solver2.h>
#include <ik_constraint2/ik_constraint2.h>

namespace reachability_map_visualizer {
  class EndEffector {
  public:
    cnoid::LinkPtr parent = nullptr;
    cnoid::Isometry3 localPose = cnoid::Isometry3::Identity();
  };
  class TorqueLimit {
  public:
    cnoid::LinkPtr joint = nullptr;
    double limit = 0.0;
    TorqueLimit(){};
    TorqueLimit(cnoid::LinkPtr joint, double limit) : joint(joint), limit(limit){};
  };
  class SupportPoint {
  public:
    cnoid::LinkPtr parent = nullptr;
    cnoid::Vector3 localPosition = cnoid::Vector3::Zero();
    SupportPoint(){};
    SupportPoint(cnoid::LinkPtr parent, const cnoid::Vector3& localPosition) : parent(parent), localPosition(localPosition){};
  };
  class ReachabilityMapParam {
  public:
    cnoid::BodyPtr robot;
    std::vector<std::vector<std::shared_ptr<ik_constraint2::IKConstraint> > > constraints;
    std::vector<cnoid::LinkPtr> variables;
    std::vector<cnoid::LinkPtr> torque_check_joints;
    std::vector<TorqueLimit> torque_limits;
    std::vector<SupportPoint> support_points;
    std::vector<EndEffector> endEffectors;
    double posResolution = 0.1;
    int testPerGrid = 100;
    int initialSolutionNum = 10;
    cnoid::Vector3 size = cnoid::Vector3(2.0,2.0,2.0);
    cnoid::Vector3 origin = cnoid::Vector3::Zero();
    cnoid::Vector6 weight; // ランダムに選んだ姿勢座標系
    prioritized_inverse_kinematics_solver2::IKParam pikParam;
    bool enable_torque_check = false;
    bool enable_support_polygon_check = false;
    double torque_limit = 0;
    ReachabilityMapParam(){
      pikParam.maxIteration = 30;
      weight << 1.0, 1.0, 1.0, 1.0, 1.0, 1.0; // IKはgridの中心位置に対して解くので、posResolutionを大きくすればするほど、たまたま中心では解けないがその近辺では解ける、ということが起こる. 位置に関する重みをprecision / posResolution倍しておくとよい.
    };
  };
  class ReachabilityMap {
  public:
    double posResolution = 0.1;
    cnoid::Vector3 origin = cnoid::Vector3::Zero();
    double transparency = 0.9;
    cnoid::Vector3 boxSize = cnoid::Vector3::Zero(); // defaultは立方体で埋め尽くす. Zを薄くすると横から見やすい.
    std::vector<std::pair<cnoid::Vector3, double>> reachabilityMap;
  };
  // origin中心としたsizeの立方体をposResolutionで区切ったグリッドそれぞれの位置について
  // ランダム姿勢を生成し、EndEffectorがそのPoseに到達するかどうかを調べる．これを全EndEffectorで調べる.
  // ランダム姿勢をtestPerGrid回生成して解けた割合を記録し、その割合に応じてvisualizeする.
  void createMap(const std::shared_ptr<ReachabilityMapParam>& param, const std::shared_ptr<ReachabilityMap>& map, const int thread_num=1);
  void visualizeMap(const std::shared_ptr<ReachabilityMap>& map, const std::shared_ptr<choreonoid_viewer::Viewer>& viewer);
  void writeMap(std::string outputFilePath, const std::shared_ptr<ReachabilityMap>& map);
  void readMap(std::string inputFilePath, const std::shared_ptr<ReachabilityMap>& map);
}

#endif
