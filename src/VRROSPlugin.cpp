#include "VRROSPlugin.h"

#include "VRROSManager.h"
#include "ScenePublisher.h"

#include <cnoid/PluginManager>
#include <cnoid/OptionManager>
#include <cnoid/App>

using namespace std;
using namespace cnoid;

namespace po = boost::program_options;

class VRROSPlugin::Impl
{
public:
    Impl() {};
    ~Impl() {};
    //
    VRROSManager *ros_manager;
    ScenePublisher *scene_publisher;

    void onSigOptionsParsed(po::variables_map& _v);
    void startROSNode();
};
void VRROSPlugin::Impl::onSigOptionsParsed(po::variables_map& _v)
{
    if(_v.count("vr-node")) {
        std::string _name = _v["vr-node"].as<std::string>();
        ros_manager = VRROSManager::instance();
        ros_manager->setNodeName(_name);
    }
    if(_v.count("scene-node")) {
        std::string _name = _v["scene-node"].as<std::string>();
        scene_publisher = ScenePublisher::instance();
        // ros_manager->setNodeName(_name);
    }
}
void VRROSPlugin::Impl::startROSNode()
{
    if (!!ros_manager) {
        ros_manager->startROSNode();
    }
    if (!!scene_publisher) {
        scene_publisher->startROSNode();
    }
}
////
VRROSPlugin::VRROSPlugin()
    : Plugin("VRROS")
{
    impl = new Impl();
}

bool VRROSPlugin::initialize()
{
    OptionManager& om = this->optionManager();
    om.addOption("vr-node", po::value<std::string>(), "VR node-name");
    om.addOption("scene-node", po::value<std::string>(), "VR scene node-name");

    om.sigOptionsParsed(1).connect(
        [&](po::variables_map& _v) { impl->onSigOptionsParsed(_v); } );

    App::sigExecutionStarted().connect([this](){  this->impl->startROSNode(); });

    return true;
}

CNOID_IMPLEMENT_PLUGIN_ENTRY(VRROSPlugin)
