#include "AssemblerROSPlugin.h"
#include "AssemblerROSManager.h"
#include <cnoid/PluginManager>
#include <cnoid/OptionManager>
#include <cnoid/App>
#include <cnoid/Timer>

//#include <cnoid/ext/robot_assembler_plugin/RobotAssemblerPlugin.h>
#include <cnoid/ext/robot_assembler_plugin/AssemblerManager.h>

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/joy.hpp>

#include <thread>
#include <mutex>
#include <vector>
#include <deque>

struct joystate
{
    int button_size;
    int axis_size;
    int window_size;
    std::vector<int> pushed;
    std::vector<int> pressed;
    std::vector<double> axes;

    std::vector< std::deque<double> > axes_buf;
    std::vector<int> current_button;

    void initialize(int _b_size, int _a_size)
    {
        button_size = _b_size;
        axis_size = _a_size;
        pushed.resize(_b_size, 0);
        pressed.resize(_b_size, 0);
        current_button.resize(_b_size, 0);
        axes.resize(_a_size, 0.0);
        axes_buf.resize(_a_size);
        window_size = 5;
    }
    void preProcess()
    {
        pushed.resize(button_size, 0);
        pressed.resize(button_size, 0);
        //
        // clear axes
    }
    void postProcess()
    {
        for(int i = 0; i < axis_size; i++) {
            int sz = axes_buf.size();
            double ave=0.0;
            for(auto it = axes_buf[i].begin(); it != axes_buf[i].end(); it++) {
                ave += *it;
            }
            if (sz != 0) {
                axes[i] = ave/sz;
            } else {
                axes[i] = 0.0;
            }
        }
    }
};

using namespace std;
using namespace cnoid;

using std::placeholders::_1;

namespace po = boost::program_options;

#define ID_UNKNOWN 0
#define ID_JOY_LEFT  1
#define ID_JOY_RIGHT 2

class AssemblerROSPlugin::Impl
{
public:
    Impl() : manager(nullptr), node_name("assembler_node") {

        joyLeftState.initialize(5, 4);
        joyRightState.initialize(5, 4);
    };
    ~Impl() {};
    AssemblerManager *manager;
    std::string node_name;
    void startROSNode();

    rclcpp::Node::SharedPtr ros_node;
    rclcpp::Subscription<sensor_msgs::msg::Joy>::SharedPtr sub_left;
    rclcpp::Subscription<sensor_msgs::msg::Joy>::SharedPtr sub_right;
    rclcpp::Clock ros_clock;
    //
    Timer tm;
    rclcpp::executors::SingleThreadedExecutor ros_exec;
    //
    joystate joyLeftState, joyRightState;
    void joyCallback(int id, const sensor_msgs::msg::Joy &msg);
    void processState(joystate  &state, const sensor_msgs::msg::Joy &msg);

    //
    void onSigOptionsParsed(po::variables_map& _v);
};
void AssemblerROSPlugin::Impl::onSigOptionsParsed(po::variables_map& _v)
{
    if(_v.count("assembler-node")) {
        node_name = _v["assembler-node"].as<std::string>();
    }
}
void AssemblerROSPlugin::Impl::startROSNode()
{
    std::cerr << ";;;; startROSNode ;;;;" << std::endl;

    std::cerr << "AssemblerManager instance" << std::endl;
    AssemblerManager *instance = AssemblerManager::instance();
    if (!instance || !(instance->isInitialized())) {
        std::cerr << "Manager not initialized" << std::endl;
        return;
    }
    ros_node = rclcpp::Node::make_shared(node_name);
    {
        std::function< void(const sensor_msgs::msg::Joy &) > func = std::bind(&AssemblerROSPlugin::Impl::joyCallback, this, ID_JOY_LEFT,  _1);
        sub_left  = ros_node->create_subscription<sensor_msgs::msg::Joy>("joy_left",  5, func);
    }
    {
        std::function< void(const sensor_msgs::msg::Joy &) > func = std::bind(&AssemblerROSPlugin::Impl::joyCallback, this, ID_JOY_RIGHT, _1);
        sub_right = ros_node->create_subscription<sensor_msgs::msg::Joy>("joy_right", 5, func);
    }
    ros_exec.add_node(ros_node);
    ////
    tm.sigTimeout().connect( [this]() {
        joyLeftState.preProcess();
        joyRightState.preProcess();
        this->ros_exec.spin_all(std::chrono::nanoseconds(10000000000));
        joyLeftState.postProcess();
        joyRightState.postProcess();
        // manipulate xxx
    });
    tm.start(2);
}
void AssemblerROSPlugin::Impl::joyCallback(int id, const sensor_msgs::msg::Joy &msg)
{
    if (id == ID_JOY_LEFT) {
        processState(joyLeftState, msg);
    } else if (id == ID_JOY_RIGHT) {
        processState(joyRightState, msg);
    }
}
void AssemblerROSPlugin::Impl::processState(joystate &state, const sensor_msgs::msg::Joy &msg)
{
    for(int i = 0; i < msg.axes.size() && i < state.axis_size; i++) {
        while (state.axes_buf[i].size() > state.window_size) state.axes_buf[i].pop_front();
        state.axes_buf[i].push_back(msg.axes[i]);
    }
    for(int i = 0; i < msg.buttons.size() && i < state.button_size; i++) {
        int bt = msg.buttons[i];
        if (state.current_button[i] == 0 && bt > 0) {
            // up edge
            state.pressed[i] = 1;
        } else if (state.current_button[i] == 1 && bt < 1) {
            // down edge
            state.pressed[i] = 0;
            state.pushed[i]++;
        }
        state.current_button[i] = bt;
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

    OptionManager& om = this->optionManager();
    om.addOption("assembler-node", po::value<std::string>(), "load robot_assembler config file");
    om.sigOptionsParsed(1).connect(
        [&](po::variables_map& _v) { impl->onSigOptionsParsed(_v); } );

    App::sigExecutionStarted().connect([this](){  this->impl->startROSNode();  });

    return true;
}

CNOID_IMPLEMENT_PLUGIN_ENTRY(AssemblerROSPlugin)
