#include <cnoid/Plugin>
#include <cnoid/ItemManager>

#include <choreonoid_viewer/choreonoid_viewer.h>

namespace reachability_map_visualizer_sample{
  void jaxon_hand();
  class jaxon_handItem : public choreonoid_viewer::ViewerBaseItem {
  public:
    static void initializeClass(cnoid::ExtensionManager* ext){ ext->itemManager().registerClass<jaxon_handItem>("jaxon_handItem"); }
  protected:
    virtual void main() override{ jaxon_hand(); return;}
  };
  typedef cnoid::ref_ptr<jaxon_handItem> jaxon_handItemPtr;

  void jaxon_foot();
  class jaxon_footItem : public choreonoid_viewer::ViewerBaseItem {
  public:
    static void initializeClass(cnoid::ExtensionManager* ext){ ext->itemManager().registerClass<jaxon_footItem>("jaxon_footItem"); }
  protected:
    virtual void main() override{ jaxon_foot(); return;}
  };
  typedef cnoid::ref_ptr<jaxon_footItem> jaxon_footItemPtr;

  void manta_tail();
  class manta_tailItem : public choreonoid_viewer::ViewerBaseItem {
  public:
    static void initializeClass(cnoid::ExtensionManager* ext){ ext->itemManager().registerClass<manta_tailItem>("manta_tailItem"); }
  protected:
    virtual void main() override{ manta_tail(); return;}
  };
  typedef cnoid::ref_ptr<manta_tailItem> manta_tailItemPtr;

  void visualize();
  class visualizeItem : public choreonoid_viewer::ViewerBaseItem {
  public:
    static void initializeClass(cnoid::ExtensionManager* ext){ ext->itemManager().registerClass<visualizeItem>("visualizeItem"); }
  protected:
    virtual void main() override{ visualize(); return;}
  };
  typedef cnoid::ref_ptr<visualizeItem> visualizeItemPtr;

  void param_preview();
  class param_previewItem : public choreonoid_viewer::ViewerBaseItem {
  public:
    static void initializeClass(cnoid::ExtensionManager* ext){ ext->itemManager().registerClass<param_previewItem>("param_previewItem"); }
  protected:
    virtual void main() override{ param_preview(); return;}
  };
  typedef cnoid::ref_ptr<param_previewItem> param_previewItemPtr;

  void solvability_bar();
  class solvability_barItem : public choreonoid_viewer::ViewerBaseItem {
  public:
    static void initializeClass(cnoid::ExtensionManager* ext){ ext->itemManager().registerClass<solvability_barItem>("solvability_barItem"); }
  protected:
    virtual void main() override{ solvability_bar(); return;}
  };
  typedef cnoid::ref_ptr<solvability_barItem> solvability_barItemPtr;

  class ReachabilityMapVisualizerSamplePlugin : public cnoid::Plugin
  {
  public:
    ReachabilityMapVisualizerSamplePlugin() : Plugin("ReachabilityMapVisualizerSamplePlugin")
    {
      require("Body");
    }
    virtual bool initialize() override
    {
      jaxon_handItem::initializeClass(this);
      jaxon_footItem::initializeClass(this);
      manta_tailItem::initializeClass(this);
      visualizeItem::initializeClass(this);
      param_previewItem::initializeClass(this);
      solvability_barItem::initializeClass(this);
      return true;
    }
  };
}

CNOID_IMPLEMENT_PLUGIN_ENTRY(reachability_map_visualizer_sample::ReachabilityMapVisualizerSamplePlugin)
