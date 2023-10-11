#include "AssemblerROSPlugin.h"
#include "AssemblerROSManager.h"
#include <cnoid/PluginManager>
#include <cnoid/OptionManager>
#include <cnoid/App>

using namespace std;
using namespace cnoid;

namespace po = boost::program_options;

class AssemblerROSPlugin::Impl
{
public:
    Impl() {};
    ~Impl() {};
    //
    AssemblerROSManager *ros_manager;
    void onSigOptionsParsed(po::variables_map& _v);
};
void AssemblerROSPlugin::Impl::onSigOptionsParsed(po::variables_map& _v)
{
    if(_v.count("assembler-node")) {
        std::string _name = _v["assembler-node"].as<std::string>();
        ros_manager->setNodeName(_name);
    }
}
////
AssemblerROSPlugin::AssemblerROSPlugin()
    : Plugin("AssemblerROS")
{
    require("RobotAssembler");
    impl = new Impl();
}

bool AssemblerROSPlugin::initialize()
{
    impl->ros_manager = AssemblerROSManager::instance();

    OptionManager& om = this->optionManager();
    om.addOption("assembler-node", po::value<std::string>(), "load robot_assembler config file");
    om.sigOptionsParsed(1).connect(
        [&](po::variables_map& _v) { impl->onSigOptionsParsed(_v); } );

    App::sigExecutionStarted().connect([this](){  this->impl->ros_manager->startROSNode();  });

    return true;
}

CNOID_IMPLEMENT_PLUGIN_ENTRY(AssemblerROSPlugin)
