// Optional, read-only validation of the actual GUI rendering scene.
#include <chrono>
#include <atomic>
#include <iomanip>
#include <QImage>
#include <gz/sim/gui/GuiSystem.hh>
#include <gz/rendering/Image.hh>
#include <sstream>
#include <gz/gui/Application.hh>
#include <gz/gui/GuiEvents.hh>
#include <gz/gui/MainWindow.hh>
#include <gz/gui/Plugin.hh>
#include <gz/plugin/Register.hh>
#include <gz/rendering/RenderingIface.hh>
#include <gz/rendering/Scene.hh>
#include <gz/rendering/Visual.hh>
#include <gz/rendering/Geometry.hh>
#include <gz/rendering/Material.hh>
#include <gz/rendering/Camera.hh>
#include <gz/transport/Node.hh>
#include <gz/msgs/stringmsg.pb.h>
class GuiSignalProbe : public gz::sim::GuiSystem {
  std::atomic<double> simTime{0.};
  void Update(const gz::sim::UpdateInfo &info,gz::sim::EntityComponentManager &) override {
    simTime=std::chrono::duration<double>(info.simTime).count();
  }
  gz::transport::Node node;
  gz::transport::Node::Publisher pub;
  std::chrono::steady_clock::time_point previous{};
  std::string capture;
  unsigned int frame{0};
  std::string lastCapture,pendingCapture;
  std::chrono::steady_clock::time_point captureDue{};
  void LoadConfig(const tinyxml2::XMLElement *config) override {
    pub=node.Advertise<gz::msgs::StringMsg>("/evaluation/gui_signals");
    if(auto path=config->FirstChildElement("capture")) capture=path->GetText();
    gz::gui::App()->findChild<gz::gui::MainWindow *>()->installEventFilter(this);
  }
  bool eventFilter(QObject *object,QEvent *event) override {
    if(event->type()==gz::gui::events::Render::kType) {
      auto now=std::chrono::steady_clock::now();
      if(now-previous>std::chrono::milliseconds(250)) {
        previous=now;
        auto scene=gz::rendering::sceneFromFirstRenderEngine();
        if(scene) {
          std::ostringstream out;out<<std::setprecision(17)<<"{\"sim_time\":"<<simTime.load()<<",\"wall_time\":"<<std::chrono::duration<double>(now.time_since_epoch()).count()<<",\"bulbs\":{";bool first=true;
          for(unsigned int i=0;i<scene->VisualCount();++i) {
            auto visual=scene->VisualByIndex(i);auto name=visual->Name();
            auto start=name.find("signal_");
            if(start==std::string::npos || visual->GeometryCount()==0)continue;
            auto material=visual->GeometryByIndex(0)->Material();
            if(!material)continue;
            auto color=material->Emissive();
            if(!first)out<<",";first=false;
            out<<"\""<<name.substr(start)<<"\":["<<color.R()<<","<<color.G()<<","<<color.B()<<"]";
          }
          out<<"}}";gz::msgs::StringMsg message;message.set_data(out.str());pub.Publish(message);
          auto colors=out.str().substr(out.str().find("\"bulbs\""));
          if(colors!=pendingCapture) {pendingCapture=colors;captureDue=now+std::chrono::milliseconds(500);}
          // Copy pixels only after the material change has reached a rendered frame.
          if(!capture.empty() && !first && colors!=lastCapture && now>=captureDue) {
            lastCapture=colors;++frame;
            for(unsigned int i=0;i<scene->SensorCount();++i) {
              auto camera=std::dynamic_pointer_cast<gz::rendering::Camera>(scene->SensorByIndex(i));
              if(camera) {
                gz::rendering::Image pixels(camera->ImageWidth(),camera->ImageHeight(),gz::rendering::PF_R8G8B8);
                camera->Copy(pixels);
                QImage picture(pixels.Data<unsigned char>(),pixels.Width(),pixels.Height(),pixels.Width()*3,QImage::Format_RGB888);
                picture.save(QString::fromStdString(capture+"/gui_"+std::to_string(frame)+".png"));break;
              }
            }
          }
        }
      }
    }
    return QObject::eventFilter(object,event);
  }
};
GZ_ADD_PLUGIN(GuiSignalProbe,gz::gui::Plugin)
