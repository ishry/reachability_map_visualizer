#include <reachability_map_visualizer/reachability_map_visualizer.h>
#include <choreonoid_viewer/choreonoid_viewer.h>
#include <cnoid/Body>
#include <cnoid/MeshGenerator>
#include <iostream>

namespace reachability_map_visualizer_sample
{
  std::shared_ptr<reachability_map_visualizer::ReachabilityMapParam> create_manta_tail_param();

  namespace
  {
    cnoid::SgMaterialPtr makeMaterial(const cnoid::Vector3f& color, double transparency)
    {
      cnoid::SgMaterialPtr material = new cnoid::SgMaterial();
      material->setDiffuseColor(color);
      material->setTransparency(transparency);
      return material;
    }

    cnoid::SgShapePtr makeBoxShape(cnoid::MeshGenerator& meshGenerator,
                                   const cnoid::Vector3& size,
                                   const cnoid::Vector3f& color,
                                   double transparency)
    {
      cnoid::SgShapePtr shape = new cnoid::SgShape();
      shape->setMesh(meshGenerator.generateBox(size));
      shape->setMaterial(makeMaterial(color, transparency));
      return shape;
    }

    cnoid::SgShapePtr makeSphereShape(cnoid::MeshGenerator& meshGenerator,
                                      double radius,
                                      const cnoid::Vector3f& color,
                                      double transparency)
    {
      cnoid::SgShapePtr shape = new cnoid::SgShape();
      shape->setMesh(meshGenerator.generateSphere(radius));
      shape->setMaterial(makeMaterial(color, transparency));
      return shape;
    }

    cnoid::BodyPtr makeSearchBox(const cnoid::Vector3& origin, const cnoid::Vector3& size)
    {
      cnoid::BodyPtr body = new cnoid::Body();
      cnoid::LinkPtr rootLink = new cnoid::Link();
      cnoid::MeshGenerator meshGenerator;

      cnoid::SgPosTransformPtr boxTransform = new cnoid::SgPosTransform();
      boxTransform->translation() = origin;
      boxTransform->addChild(makeBoxShape(meshGenerator, size, cnoid::Vector3f(0.2f, 0.6f, 1.0f), 0.85));
      rootLink->addShapeNode(boxTransform);

      cnoid::SgPosTransformPtr originTransform = new cnoid::SgPosTransform();
      originTransform->translation() = origin;
      originTransform->addChild(makeSphereShape(meshGenerator, 0.06, cnoid::Vector3f(1.0f, 0.9f, 0.0f), 0.0));
      rootLink->addShapeNode(originTransform);

      body->setRootLink(rootLink);
      return body;
    }

    cnoid::BodyPtr makeFrame(const cnoid::Isometry3& pose)
    {
      cnoid::BodyPtr body = new cnoid::Body();
      cnoid::LinkPtr rootLink = new cnoid::Link();
      cnoid::MeshGenerator meshGenerator;

      double length = 0.25;
      double width = 0.025;
      cnoid::SgPosTransformPtr frameTransform = new cnoid::SgPosTransform();
      frameTransform->T() = pose;

      cnoid::SgPosTransformPtr xTransform = new cnoid::SgPosTransform();
      xTransform->translation() = cnoid::Vector3(length / 2.0, 0.0, 0.0);
      xTransform->addChild(makeBoxShape(meshGenerator, cnoid::Vector3(length, width, width), cnoid::Vector3f(1.0f, 0.0f, 0.0f), 0.0));
      frameTransform->addChild(xTransform);

      cnoid::SgPosTransformPtr yTransform = new cnoid::SgPosTransform();
      yTransform->translation() = cnoid::Vector3(0.0, length / 2.0, 0.0);
      yTransform->addChild(makeBoxShape(meshGenerator, cnoid::Vector3(width, length, width), cnoid::Vector3f(0.0f, 1.0f, 0.0f), 0.0));
      frameTransform->addChild(yTransform);

      cnoid::SgPosTransformPtr zTransform = new cnoid::SgPosTransform();
      zTransform->translation() = cnoid::Vector3(0.0, 0.0, length / 2.0);
      zTransform->addChild(makeBoxShape(meshGenerator, cnoid::Vector3(width, width, length), cnoid::Vector3f(0.0f, 0.0f, 1.0f), 0.0));
      frameTransform->addChild(zTransform);

      rootLink->addShapeNode(frameTransform);
      body->setRootLink(rootLink);
      return body;
    }
  }

  void param_preview()
  {
    std::shared_ptr<choreonoid_viewer::Viewer> viewer = std::make_shared<choreonoid_viewer::Viewer>();
    std::shared_ptr<reachability_map_visualizer::ReachabilityMapParam> param = create_manta_tail_param();
    if (!param->robot) {
      std::cerr << "!robot" << std::endl;
      return;
    }

    viewer->objects(param->robot);
    viewer->objects(makeSearchBox(param->origin, param->size));

    for (int i = 0; i < param->endEffectors.size(); i++) {
      const reachability_map_visualizer::EndEffector& ee = param->endEffectors[i];
      if (!ee.parent) {
        std::cerr << "!end effector parent" << std::endl;
        continue;
      }
      cnoid::Isometry3 eePose = ee.parent->T() * ee.localPose;
      viewer->objects(makeFrame(eePose));
    }

    viewer->drawObjects();
    while(true);
  }
}
