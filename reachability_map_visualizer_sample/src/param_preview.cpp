#include <reachability_map_visualizer/reachability_map_visualizer.h>
#include <choreonoid_viewer/choreonoid_viewer.h>
#include <cnoid/Body>
#include <cnoid/MeshGenerator>
#include <cnoid/SceneDrawables>
#include <algorithm>
#include <iostream>
#include <string>
#include <vector>

namespace reachability_map_visualizer_sample
{
  std::shared_ptr<reachability_map_visualizer::ReachabilityMapParam> create_manta_tail_param();

  namespace
  {
    struct SupportPoint
    {
      std::string name;
      cnoid::Vector3 position;
    };

    const double previewPlaneZ = -0.2;

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

    cnoid::Vector3 calcCenterOfMass(cnoid::BodyPtr robot)
    {
      robot->calcForwardKinematics();
      return robot->calcCenterOfMass();
    }

    cnoid::BodyPtr makeCenterOfMassMarker(const cnoid::Vector3& centerOfMass)
    {
      cnoid::BodyPtr body = new cnoid::Body();
      cnoid::LinkPtr rootLink = new cnoid::Link();
      cnoid::MeshGenerator meshGenerator;

      cnoid::SgPosTransformPtr centerOfMassTransform = new cnoid::SgPosTransform();
      centerOfMassTransform->translation() = cnoid::Vector3(centerOfMass[0], centerOfMass[1], previewPlaneZ);
      centerOfMassTransform->addChild(makeSphereShape(meshGenerator, 0.02, cnoid::Vector3f(1.0f, 0.0f, 0.8f), 0.0));
      rootLink->addShapeNode(centerOfMassTransform);

      body->setRootLink(rootLink);
      return body;
    }

    bool appendSupportPoint(cnoid::BodyPtr robot,
                            const std::string& label,
                            const std::vector<std::string>& linkNameCandidates,
                            std::vector<SupportPoint>& supportPoints)
    {
      for (int i = 0; i < linkNameCandidates.size(); i++) {
        cnoid::LinkPtr link = robot->link(linkNameCandidates[i]);
        if (link) {
          supportPoints.push_back(SupportPoint{label, link->p()});
          std::cerr << "support point " << label << " (" << linkNameCandidates[i] << ") : ["
                    << link->p()[0] << ", " << link->p()[1] << ", " << link->p()[2] << "]" << std::endl;
          return true;
        }
      }

      std::cerr << "!support point link for " << label << std::endl;
      return false;
    }

    std::vector<SupportPoint> getWheelSupportPoints(cnoid::BodyPtr robot)
    {
      std::vector<SupportPoint> supportPoints;
      appendSupportPoint(robot, "right wheel", {"R_WHEEL_JOINT", "R_WHEEL"}, supportPoints);
      appendSupportPoint(robot, "left wheel", {"L_WHEEL_JOINT", "L_WHEEL"}, supportPoints);
      appendSupportPoint(robot, "omni wheel", {"OMNI_WHEEL_JOINT", "OMNI_WHEEL"}, supportPoints);
      return supportPoints;
    }

    double cross2d(const SupportPoint& origin, const SupportPoint& a, const SupportPoint& b)
    {
      return (a.position[0] - origin.position[0]) * (b.position[1] - origin.position[1])
           - (a.position[1] - origin.position[1]) * (b.position[0] - origin.position[0]);
    }

    std::vector<SupportPoint> calcSupportPolygon(std::vector<SupportPoint> supportPoints)
    {
      std::sort(supportPoints.begin(), supportPoints.end(), [](const SupportPoint& a, const SupportPoint& b) {
        if (a.position[0] == b.position[0]) {
          return a.position[1] < b.position[1];
        }
        return a.position[0] < b.position[0];
      });

      std::vector<SupportPoint> hull;
      for (int i = 0; i < supportPoints.size(); i++) {
        while (hull.size() >= 2 && cross2d(hull[hull.size() - 2], hull[hull.size() - 1], supportPoints[i]) <= 0.0) {
          hull.pop_back();
        }
        hull.push_back(supportPoints[i]);
      }

      int lowerHullSize = hull.size();
      for (int i = static_cast<int>(supportPoints.size()) - 2; i >= 0; i--) {
        while (hull.size() > lowerHullSize && cross2d(hull[hull.size() - 2], hull[hull.size() - 1], supportPoints[i]) <= 0.0) {
          hull.pop_back();
        }
        hull.push_back(supportPoints[i]);
      }

      if (!hull.empty()) {
        hull.pop_back();
      }
      return hull;
    }

    cnoid::BodyPtr makeSupportPolygon(const std::vector<SupportPoint>& supportPoints)
    {
      cnoid::BodyPtr body = new cnoid::Body();
      cnoid::LinkPtr rootLink = new cnoid::Link();
      cnoid::MeshGenerator meshGenerator;

      for (int i = 0; i < supportPoints.size(); i++) {
        cnoid::SgPosTransformPtr supportPointTransform = new cnoid::SgPosTransform();
        supportPointTransform->translation() = cnoid::Vector3(supportPoints[i].position[0], supportPoints[i].position[1], previewPlaneZ);
        supportPointTransform->addChild(makeSphereShape(meshGenerator, 0.02, cnoid::Vector3f(0.0f, 1.0f, 1.0f), 0.0));
        rootLink->addShapeNode(supportPointTransform);
      }

      std::vector<SupportPoint> polygon = calcSupportPolygon(supportPoints);
      if (polygon.size() >= 2) {
        cnoid::SgLineSetPtr lineSet = new cnoid::SgLineSet();
        lineSet->setLineWidth(4.0);
        lineSet->getOrCreateColors()->push_back(cnoid::Vector3f(0.0f, 1.0f, 0.2f));
        cnoid::SgVertexArray* vertices = lineSet->getOrCreateVertices();
        double polygonZ = previewPlaneZ + 0.01;

        for (int i = 0; i < polygon.size(); i++) {
          vertices->push_back(cnoid::Vector3f(polygon[i].position[0], polygon[i].position[1], polygonZ));
        }

        for (int i = 0; i < polygon.size(); i++) {
          int next = (i + 1) % polygon.size();
          lineSet->addLine(i, next);
          lineSet->colorIndices().push_back(0);
          lineSet->colorIndices().push_back(0);
        }

        rootLink->addShapeNode(lineSet);
      }

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
    cnoid::Vector3 centerOfMass = calcCenterOfMass(param->robot);
    std::cerr << "center of mass : [" << centerOfMass[0] << ", " << centerOfMass[1] << ", " << centerOfMass[2] << "]" << std::endl;
    viewer->objects(makeCenterOfMassMarker(centerOfMass));
    viewer->objects(makeSupportPolygon(getWheelSupportPoints(param->robot)));

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
