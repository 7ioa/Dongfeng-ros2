// Keep Ogre 1's generated shader passes in step with Gazebo VisualCmd updates.
// This plugin observes GUI materials only; it never commands lights or vehicles.
#include <chrono>
#include <regex>

#include <gz/common/Console.hh>
#include <gz/gui/Application.hh>
#include <gz/gui/GuiEvents.hh>
#include <gz/gui/MainWindow.hh>
#include <gz/gui/Plugin.hh>
#include <gz/plugin/Register.hh>
#include <gz/rendering/Geometry.hh>
#include <gz/rendering/RenderingIface.hh>
#include <gz/rendering/Scene.hh>
#include <gz/rendering/Visual.hh>
#include <gz/rendering/ogre/OgreMaterial.hh>

class TrafficLightMaterialSync : public gz::gui::Plugin
{
  private: std::chrono::steady_clock::time_point previous{};
  private: const std::regex bulbName{
    R"(dongfeng_sandbox::site::signal_[0-9]+_(red|yellow|green))"};

  public: void LoadConfig(const tinyxml2::XMLElement *) override
  {
    auto window = gz::gui::App()->findChild<gz::gui::MainWindow *>();
    if (!window)
    {
      gzerr << "TrafficLightMaterialSync requires the main Gazebo window.\n";
      return;
    }
    window->installEventFilter(this);
    gzmsg << "TrafficLightMaterialSync loaded for sandbox traffic lights.\n";
  }

  protected: bool eventFilter(QObject *object, QEvent *event) override
  {
    // Ogre objects must only be accessed on the GUI render thread.
    if (event->type() == gz::gui::events::Render::kType)
    {
      const auto now = std::chrono::steady_clock::now();
      if (now - this->previous >= std::chrono::milliseconds(50))
      {
        this->previous = now;
        this->SyncMaterials();
      }
    }
    return QObject::eventFilter(object, event);
  }

  private: void SyncMaterials()
  {
    auto scene = gz::rendering::sceneFromFirstRenderEngine();
    if (!scene)
      return;
    // Look up current visuals each time, so scene reloads cannot leave stale
    // Ogre pointers. Other models, lamp housings and Ogre 2 stay untouched.
    for (unsigned int i = 0; i < scene->VisualCount(); ++i)
    {
      auto visual = scene->VisualByIndex(i);
      if (!visual || !std::regex_match(visual->Name(), this->bulbName))
        continue;
      for (unsigned int g = 0; g < visual->GeometryCount(); ++g)
      {
        auto material = std::dynamic_pointer_cast<gz::rendering::OgreMaterial>(
          visual->GeometryByIndex(g)->Material());
        if (!material)
          continue;
        auto native = material->Material();
        if (native.isNull() || native->getNumTechniques() == 0 ||
            native->getTechnique(0)->getNumPasses() == 0)
          continue;
        // gz-rendering changes technique 0 / pass 0. Ogre's RTSS technique
        // keeps copied colors unless they are explicitly synchronized.
        const auto base = native->getTechnique(0)->getPass(0);
        for (unsigned short t = 0; t < native->getNumTechniques(); ++t)
        {
          const auto technique = native->getTechnique(t);
          for (unsigned short p = 0; p < technique->getNumPasses(); ++p)
          {
            auto pass = technique->getPass(p);
            if (pass == base)
              continue;
            if (pass->getAmbient() != base->getAmbient())
              pass->setAmbient(base->getAmbient());
            if (pass->getDiffuse() != base->getDiffuse())
              pass->setDiffuse(base->getDiffuse());
            if (pass->getSpecular() != base->getSpecular())
              pass->setSpecular(base->getSpecular());
            if (pass->getSelfIllumination() != base->getSelfIllumination())
              pass->setSelfIllumination(base->getSelfIllumination());
          }
        }
      }
    }
  }
};

GZ_ADD_PLUGIN(TrafficLightMaterialSync, gz::gui::Plugin)
