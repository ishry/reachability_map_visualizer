#include <mutex>
#include <reachability_map_visualizer/reachability_map_visualizer.h>
#include <cnoid/MeshGenerator>
#include <cnoid/YAMLWriter>
#include <cnoid/YAMLReader>
#include <random>
#include <vector>
#include <iomanip>

namespace reachability_map_visualizer {
  void frame2Link(const std::vector<double>& frame, const std::vector<cnoid::LinkPtr>& links){
    unsigned int i=0;
    for(int l=0;l<links.size();l++){
      if(links[l]->isRevoluteJoint() || links[l]->isPrismaticJoint()) {
        links[l]->q() = frame[i];
        i+=1;
      }else if(links[l]->isFreeJoint()) {
        links[l]->p()[0] = frame[i+0];
        links[l]->p()[1] = frame[i+1];
        links[l]->p()[2] = frame[i+2];
        links[l]->R() = cnoid::Quaternion(frame[i+6],
                                          frame[i+3],
                                          frame[i+4],
                                          frame[i+5]).toRotationMatrix();
        i+=7;
      }
    }
  }
  void link2Frame(const std::vector<cnoid::LinkPtr>& links, std::vector<double>& frame){
    frame.clear();
    for(int l=0;l<links.size();l++){
      if(links[l]->isRevoluteJoint() || links[l]->isPrismaticJoint()) {
        frame.push_back(links[l]->q());
      }else if(links[l]->isFreeJoint()) {
        frame.push_back(links[l]->p()[0]);
        frame.push_back(links[l]->p()[1]);
        frame.push_back(links[l]->p()[2]);
        cnoid::Quaternion q(links[l]->R());
        frame.push_back(q.x());
        frame.push_back(q.y());
        frame.push_back(q.z());
        frame.push_back(q.w());
      }
    }
  }
  void randomFrame(const std::vector<cnoid::LinkPtr>& links, std::vector<double>& frame){
    frame.clear();
    std::random_device seed_gen;
    std::default_random_engine engine(seed_gen());
    for(int l=0;l<links.size();l++){
      if(links[l]->isRevoluteJoint() || links[l]->isPrismaticJoint()) {
        std::uniform_real_distribution<> dist(links[l]->q_lower(),links[l]->q_upper());
        frame.push_back(dist(engine));
      }else if(links[l]->isFreeJoint()) { // 適当に0.2mの範囲にしておく
        std::uniform_real_distribution<> dist(-0.2, 0.2);
        frame.push_back(dist(engine));
        frame.push_back(dist(engine));
        frame.push_back(dist(engine));
        cnoid::Quaternion q = Eigen::Quaterniond::UnitRandom();
        frame.push_back(q.x());
        frame.push_back(q.y());
        frame.push_back(q.z());
        frame.push_back(q.w());
      }
    }
  }

  void createMapSub(const std::shared_ptr<ReachabilityMapParam>& param, const std::shared_ptr<ReachabilityMap>& map,
                    const std::vector<cnoid::Vector3>& xyz_table, const std::vector<double>& init_pose, std::mutex& mutex, int thread_num) {
    cnoid::BodyPtr tmp_robot;
    {
     std::lock_guard<std::mutex> lock(mutex);
     tmp_robot = param->robot->clone();
    }
    std::vector<cnoid::LinkPtr> tmp_variables;
    for (int v_num = 0; v_num < param->variables.size(); v_num++) {
      tmp_variables.push_back(tmp_robot->joint(param->variables[v_num]->jointId()));
    }
    tmp_robot->rootLink()->T() = param->robot->rootLink()->T();
    tmp_robot->rootLink()->v() = param->robot->rootLink()->v();
    tmp_robot->rootLink()->w() = param->robot->rootLink()->w();
    for(int i=0;i<tmp_robot->numJoints();i++){
      tmp_robot->joint(i)->q() = param->robot->joint(i)->q();
    }

    for (int xyz_index = 0; xyz_index < xyz_table.size(); xyz_index++) {
      double solveCount = 0.0;
      cnoid::Isometry3 targetPose;
      targetPose.translation() = param->origin + xyz_table[xyz_index];
      for (int i=0; i<param->testPerGrid; i++) {
        bool solved = false;
        targetPose.linear() = Eigen::Quaterniond::UnitRandom().matrix();
        for (int e=0; e< param->endEffectors.size() && !solved; e++) {
          // joint limit
          std::vector<std::shared_ptr<ik_constraint2::IKConstraint> > constraints0;
          for(int j_id=0;j_id<tmp_robot->numJoints();j_id++){
            std::shared_ptr<ik_constraint2::JointLimitConstraint> constraint = std::make_shared<ik_constraint2::JointLimitConstraint>();
            constraint->joint() = tmp_robot->joint(j_id);
            constraints0.push_back(constraint);
          }
          // ee
          std::vector<std::shared_ptr<ik_constraint2::IKConstraint> > constraints1;
          std::shared_ptr<ik_constraint2::PositionConstraint> ee_constraint = std::make_shared<ik_constraint2::PositionConstraint>();
          ee_constraint->A_link() = tmp_robot->joint(param->endEffectors[e].parent->jointId());
          ee_constraint->A_localpos() = param->endEffectors[e].localPose;
          ee_constraint->B_link() = nullptr;
          ee_constraint->B_localpos() = targetPose;
          ee_constraint->eval_link() = nullptr;
          ee_constraint->eval_localR() = targetPose.linear();
          ee_constraint->precision() = param->posResolution;
          for (int w=0;w<3;w++) {
            ee_constraint->weight()[w+3] = param->weight[w+3] * param->posResolution / 1e-3;
          }
          constraints1.push_back(ee_constraint);
          std::vector<std::vector<std::shared_ptr<ik_constraint2::IKConstraint> > > all_constraints{constraints0, constraints1};
          std::vector<std::shared_ptr<prioritized_qp_base::Task> > prevTasks;

          frame2Link(init_pose,tmp_variables);
          tmp_robot->calcForwardKinematics();
          tmp_robot->calcCenterOfMass();
          solved = prioritized_inverse_kinematics_solver2::solveIKLoop(tmp_variables,
                                                                       all_constraints,
                                                                       prevTasks,
                                                                       param->pikParam
                                                                       );
          for (int sol=0;sol<param->initialSolutionNum && !solved;sol++) {
            std::vector<double> initialSolution;
            randomFrame(tmp_variables,initialSolution);
            frame2Link(initialSolution,tmp_variables);
            tmp_robot->calcForwardKinematics();
            tmp_robot->calcCenterOfMass();
            solved = prioritized_inverse_kinematics_solver2::solveIKLoop(tmp_variables,
                                                                         all_constraints,
                                                                         prevTasks,
                                                                         param->pikParam
                                                                         );
          }
          frame2Link(init_pose,tmp_variables);
          tmp_robot->calcForwardKinematics();
          tmp_robot->calcCenterOfMass();
        }
        if (solved) solveCount += 1;
      }
      double solvability = solveCount / param->testPerGrid;
      std::cerr << "thread : " << thread_num;
      std::cerr << std::fixed << std::setprecision(2);
      std::cerr << "  x : " << xyz_table[xyz_index][0] << " y : " << xyz_table[xyz_index][1] << " z : " << xyz_table[xyz_index][2];
      std::cerr << " solvability : " << solvability << std::flush;
      std::cerr << " progress: " << xyz_index << "/" << xyz_table.size() << std::endl;
      if (solvability>0.0){
        std::lock_guard<std::mutex> lock(mutex);
        map->reachabilityMap.push_back(std::pair<cnoid::Vector3, double>(xyz_table[xyz_index], solvability));
      }
    }
  }

  void createMap(const std::shared_ptr<ReachabilityMapParam>& param, const std::shared_ptr<ReachabilityMap>& map, int thread_num) {
    std::mutex mutex;

    map->reachabilityMap.clear();
    map->origin = param->origin;
    map->posResolution = param->posResolution;
    std::vector<double> initPose;
    link2Frame(param->variables, initPose);

    std::vector<cnoid::Vector3> xyz_table_all;
    std::vector<std::vector<cnoid::Vector3>> xyz_table_block(thread_num);
    int index = 0;
    int grid_x_num = static_cast<int>(param->size[0] / param->posResolution) + 1;
    int grid_y_num = static_cast<int>(param->size[1] / param->posResolution) + 1;
    int grid_z_num = static_cast<int>(param->size[2] / param->posResolution) + 1;
    int grid_all = grid_x_num * grid_y_num * grid_z_num;
    for (int index_x = 0; index_x < grid_x_num; index_x++) {
      for (int index_y = 0; index_y < grid_y_num; index_y++) {
        for (int index_z = 0; index_z < grid_z_num; index_z++) {
          double grid_x = - param->size[0]/2 + index_x * param->posResolution;
          double grid_y = - param->size[1]/2 + index_y * param->posResolution;
          double grid_z = - param->size[2]/2 + index_z * param->posResolution;
          xyz_table_block[index].push_back(cnoid::Vector3(grid_x, grid_y, grid_z));
          if (xyz_table_block[index].size() >= (grid_all / thread_num) + (index < grid_all % thread_num ? 1 : 0)) {
              index++;
          }
        }
      }
    }

    std::vector<std::thread> threads;
    for(size_t i=0; i<thread_num; ++i){
        threads.emplace_back(std::thread(createMapSub, std::ref(param), std::ref(map), std::ref(xyz_table_block[i]), std::ref(initPose), std::ref(mutex), i));
    }
    for(auto& thread : threads){
        thread.join();
    }
  }
  void visualizeMap(const std::shared_ptr<ReachabilityMap>& map, const std::shared_ptr<choreonoid_viewer::Viewer>& viewer) {
    cnoid::BodyPtr reachabilityMap = new cnoid::Body();
    cnoid::MeshGenerator meshGenerator;
    cnoid::LinkPtr rootLink = new cnoid::Link();
    for (int i=0;i<map->reachabilityMap.size();i++) {
      double solvability = map->reachabilityMap[i].second;
      float red = std::max(1.0 - 2*solvability, 0.0);
      float green = std::max(1.0 - std::abs(1 - 2*solvability), 0.0);
      float blue = std::max(2*solvability - 1.0, 0.0);

      cnoid::SgShapePtr shape = new cnoid::SgShape();
      if (map->boxSize == cnoid::Vector3::Zero()) shape->setMesh(meshGenerator.generateBox(cnoid::Vector3(map->posResolution,map->posResolution,map->posResolution)));
      else shape->setMesh(meshGenerator.generateBox(map->boxSize));
      cnoid::SgMaterialPtr material = new cnoid::SgMaterial();
      material->setTransparency(map->transparency);
      material->setDiffuseColor(cnoid::Vector3f(red, green, blue));
      shape->setMaterial(material);
      cnoid::SgPosTransformPtr posTransform = new cnoid::SgPosTransform();
      posTransform->translation() = map->origin + map->reachabilityMap[i].first;;
      posTransform->addChild(shape);
      rootLink->addShapeNode(posTransform);
    }
    reachabilityMap->setRootLink(rootLink);
    viewer->objects(reachabilityMap);
  }
  void writeMap(std::string outputFilePath, const std::shared_ptr<ReachabilityMap>& map) {
    cnoid::YAMLWriter writer(outputFilePath);
    writer.startMapping();
    writer.putKey("reachabilityMapParam");
    writer.startListing();
    {
      writer.startMapping();
      writer.putKey("origin");
      writer.startFlowStyleListing();
      for (int i=0; i<3; i++) writer.putScalar(map->origin[i]);
      writer.endListing();
      writer.putKeyValue("posResolution",map->posResolution);
      writer.endMapping();
    }
    writer.endListing();
    writer.endMapping();
    writer.startMapping();
    writer.putKey("reachabilityMap");
    writer.startListing();
    for (int i=0; i<map->reachabilityMap.size(); i++) {
      writer.startMapping();
      writer.startListing();
      {
        writer.putKey("translation");
        writer.startFlowStyleListing();
        for (int j=0; j<3; j++) writer.putScalar(map->reachabilityMap[i].first[j]);
        writer.endListing();
        writer.putKeyValue("solvability",map->reachabilityMap[i].second);
      }
      writer.endListing();
      writer.endMapping();
    }
    writer.endListing();
    writer.endMapping();

  }
  void readMap(std::string inputFilePath, const std::shared_ptr<ReachabilityMap>& map) {
    map->reachabilityMap.clear();
    cnoid::YAMLReader reader;
    cnoid::MappingPtr node;
    try {
      node = reader.loadDocument(inputFilePath)->toMapping();
    } catch(const cnoid::ValueNode::Exception& ex) {
      std::cerr << "cannot load config file" << std::endl;;
      return;
    }
    // load
    cnoid::Listing* reachabilityMapParam = node->findListing("reachabilityMapParam");
    {
      cnoid::Mapping* info = reachabilityMapParam->at(0)->toMapping();
      cnoid::ValueNodePtr translation_ = info->extract("origin");
      if(translation_){
        cnoid::ListingPtr translationTmp = translation_->toListing();
        if(translationTmp->size()==3){
          map->origin = cnoid::Vector3(translationTmp->at(0)->toDouble(), translationTmp->at(1)->toDouble(), translationTmp->at(2)->toDouble());
        }
      }
      cnoid::ValueNodePtr posResolution_ = info->extract("posResolution");
      if (posResolution_ && posResolution_->isScalar()){
        map->posResolution = posResolution_->toDouble();
      }
    }

    cnoid::Listing* reachabilityMap = node->findListing("reachabilityMap");
    if (!reachabilityMap->isValid()) {
      std::cerr << "cannot load config file value" << std::endl;;
      return;
    } else {
      for (int i=0; i<reachabilityMap->size();i++) {
        cnoid::Mapping* info = reachabilityMap->at(i)->toMapping();
        std::pair<cnoid::Vector3,double> param;
        cnoid::ValueNodePtr translation_ = info->extract("translation");
        if(translation_){
          cnoid::ListingPtr translationTmp = translation_->toListing();
          if(translationTmp->size()==3){
            param.first = cnoid::Vector3(translationTmp->at(0)->toDouble(), translationTmp->at(1)->toDouble(), translationTmp->at(2)->toDouble());
          }
        }
        cnoid::ValueNodePtr solvability_ = info->extract("solvability");
        if (solvability_ && solvability_->isScalar()){
          param.second = solvability_->toDouble();
        }
        map->reachabilityMap.push_back(param);
      }
    }
  }
}
