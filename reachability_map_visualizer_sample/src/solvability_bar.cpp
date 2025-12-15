#include <choreonoid_viewer/choreonoid_viewer.h>
#include <cnoid/Body>
#include <cnoid/MeshGenerator>

namespace reachability_map_visualizer_sample{
  void solvability_bar(){
    std::shared_ptr<choreonoid_viewer::Viewer> viewer = std::make_shared<choreonoid_viewer::Viewer>();
    cnoid::BodyPtr solvabilityBar = new cnoid::Body();
    cnoid::MeshGenerator meshGenerator;
    cnoid::LinkPtr rootLink = new cnoid::Link();
    for (int i=0;i<100;i++) {
      double solvability = i / 100.0;
      float red = std::max(1.0 - 2*solvability, 0.0);
      float green = std::max(1.0 - std::abs(1 - 2*solvability), 0.0);
      float blue = std::max(2*solvability - 1.0, 0.0);
      cnoid::SgShapePtr shape = new cnoid::SgShape();
      shape->setMesh(meshGenerator.generateBox(cnoid::Vector3(0.05, 0.05, 0.01)));
      cnoid::SgMaterialPtr material = new cnoid::SgMaterial();
      material->setTransparency(0.0);
      material->setDiffuseColor(cnoid::Vector3f(red, green, blue));
      shape->setMaterial(material);
      cnoid::SgPosTransformPtr posTransform = new cnoid::SgPosTransform();
      posTransform->translation() = cnoid::Vector3(0.0, 0.0, 0.01 * i);
      posTransform->addChild(shape);
      rootLink->addShapeNode(posTransform);
    }
    solvabilityBar->setRootLink(rootLink);
    viewer->objects(solvabilityBar);
    viewer->drawObjects();
    while(true);
  }
}
