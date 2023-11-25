#include "VRROSManager.h"
#include <cnoid/Timer>
#include <cnoid/SceneView>
#include <cnoid/SceneWidget>
#include <cnoid/SceneGraph>
#include <cnoid/SceneDrawables>

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/joy.hpp>
// tf
#include <tf2_ros/transform_listener.h>
#include <tf2_ros/buffer.h>
#include <tf2_eigen/tf2_eigen.hpp>

//#include <thread>
//#include <mutex>
#include <vector>
#include <deque>

using namespace std;
using namespace cnoid;

using std::placeholders::_1;

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

#define ID_UNKNOWN 0
#define ID_JOY_LEFT  1
#define ID_JOY_RIGHT 2

class VRROSManager::Impl
{
public:
    Impl();
    ~Impl() {};

    std::string node_name;
    void startROSNode();

    rclcpp::Node::SharedPtr ros_node;
    rclcpp::Subscription<sensor_msgs::msg::Joy>::SharedPtr sub_left;
    rclcpp::Subscription<sensor_msgs::msg::Joy>::SharedPtr sub_right;
    rclcpp::Clock ros_clock;
    std::shared_ptr<tf2_ros::TransformListener> tf_listener_{nullptr};
    std::unique_ptr<tf2_ros::Buffer> tf_buffer_;
    //
    Timer tm;
    rclcpp::executors::SingleThreadedExecutor ros_exec;
    //
    joystate joyLeftState, joyRightState;
    void joyCallback(int id, const sensor_msgs::msg::Joy &msg);
    void processJoyState(joystate  &state, const sensor_msgs::msg::Joy &msg);
    void rosMain();
    SgPosTransformPtr renderAxis(const Isometry3 &_pos);

    SgPosTransformPtr left_trans;
    SgPosTransformPtr right_trans;

    Isometry3 current_left_joy;
    Isometry3 current_right_joy;
    Isometry3 world_offset;

    SgGroup *scene_root;
    SgGroupPtr our_root;
    SgGroupPtr our_root_l;
    SgGroupPtr our_root_r;
};
VRROSManager::Impl::Impl() : node_name("cnoid_vr_node"), left_trans(nullptr), right_trans(nullptr)
{
        joyLeftState.initialize(5, 4);
        joyRightState.initialize(5, 4);
}
void VRROSManager::Impl::startROSNode()
{
    std::cerr << "[VRROSManager] startROSNode" << std::endl;

    scene_root = SceneView::instance()->sceneWidget()->scene();//
    our_root = new SgGroup();
    scene_root->addChild(our_root);

    {
        our_root_l = new SgGroup();
        our_root_r = new SgGroup();
        std::vector<SceneView *> view_instances = SceneView::instances();
        if (view_instances.size() > 2) {
            view_instances.at(1)->sceneWidget()->scene()->addChild(our_root_l);
            view_instances.at(2)->sceneWidget()->scene()->addChild(our_root_r);
        }
    }
    //
    {
        AngleAxis q( 0.0, Vector3::UnitY());
        Eigen::Translation3d tr(0, -0.2, -1.4);
        world_offset = tr * q;
    }
    //
    ros_node = rclcpp::Node::make_shared(node_name);
    {
        std::function< void(const sensor_msgs::msg::Joy &) > func = std::bind(&VRROSManager::Impl::joyCallback, this, ID_JOY_LEFT,  _1);
        sub_left  = ros_node->create_subscription<sensor_msgs::msg::Joy>("/vive/controller_1WMHHB40JN2314_Controller_Left/joy",  5, func);
    }
    {
        std::function< void(const sensor_msgs::msg::Joy &) > func = std::bind(&VRROSManager::Impl::joyCallback, this, ID_JOY_RIGHT, _1);
        sub_right = ros_node->create_subscription<sensor_msgs::msg::Joy>("/vive/controller_1WMHHB40JN2314_Controller_Right/joy", 5, func);
    }
    ros_exec.add_node(ros_node);

    tf_buffer_ = std::make_unique<tf2_ros::Buffer>(ros_node->get_clock());
    tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_, ros_node);

    ////
    tm.sigTimeout().connect( [this]() {
        joyLeftState.preProcess();
        joyRightState.preProcess();
        this->ros_exec.spin_all(std::chrono::nanoseconds(10000000000));
        joyLeftState.postProcess();
        joyRightState.postProcess();
        this->rosMain();
    });
    tm.start(2);
}
void VRROSManager::Impl::joyCallback(int id, const sensor_msgs::msg::Joy &msg)
{
    if (id == ID_JOY_LEFT) {
        processJoyState(joyLeftState, msg);
    } else if (id == ID_JOY_RIGHT) {
        processJoyState(joyRightState, msg);
    }
}
void VRROSManager::Impl::processJoyState(joystate &state, const sensor_msgs::msg::Joy &msg)
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
void VRROSManager::Impl::rosMain()
{
    try {
        // X:right Y: back Z: down
        std::string toFrame = "world";
        std::string fromFrame = "controller_1WMHHB40JN2314_Controller_Left";
        geometry_msgs::msg::TransformStamped tmp = tf_buffer_->lookupTransform(toFrame, fromFrame, tf2::TimePointZero);
        current_left_joy = world_offset * tf2::transformToEigen(tmp);
        if (!!left_trans) {
            // display joy info
            left_trans->setPosition(current_left_joy);
            our_root->notifyUpdate(SgUpdate::ADDED | SgUpdate::MODIFIED);
            our_root_l->notifyUpdate(SgUpdate::ADDED | SgUpdate::MODIFIED);
            our_root_r->notifyUpdate(SgUpdate::ADDED | SgUpdate::MODIFIED);
        } else {
            left_trans = renderAxis(current_left_joy);
            our_root->addChild(left_trans, true);
            our_root_l->addChild(left_trans, true);
            our_root_r->addChild(left_trans, true);
        }
    } catch (const tf2::TransformException & ex) {
        if (!!left_trans) {
            std::cerr << "delPos" << std::endl;
            our_root->removeChild(left_trans);
            our_root_l->removeChild(left_trans);
            our_root_r->removeChild(left_trans);
            left_trans.reset();
            our_root->notifyUpdate(SgUpdate::ADDED | SgUpdate::MODIFIED);
            our_root_l->notifyUpdate(SgUpdate::ADDED | SgUpdate::MODIFIED);
            our_root_r->notifyUpdate(SgUpdate::ADDED | SgUpdate::MODIFIED);
        }
    }
    try {
        // X:right Y: back Z: down
        std::string toFrame = "world";
        std::string fromFrame = "controller_1WMHHB40JN2314_Controller_Right";
        geometry_msgs::msg::TransformStamped tmp = tf_buffer_->lookupTransform(toFrame, fromFrame, tf2::TimePointZero);
        current_right_joy = world_offset * tf2::transformToEigen(tmp);
        if (!!right_trans) {
            // display joy info
            right_trans->setPosition(current_right_joy);
            our_root->notifyUpdate(SgUpdate::ADDED | SgUpdate::MODIFIED);
            our_root_l->notifyUpdate(SgUpdate::ADDED | SgUpdate::MODIFIED);
            our_root_r->notifyUpdate(SgUpdate::ADDED | SgUpdate::MODIFIED);
        } else {
            right_trans = renderAxis(current_right_joy);
            our_root->addChild(right_trans, true);
            our_root_l->addChild(right_trans, true);
            our_root_r->addChild(right_trans, true);
        }
    } catch (const tf2::TransformException & ex) {
        if (!!right_trans) {
            std::cerr << "delPos" << std::endl;
            our_root->removeChild(right_trans);
            our_root_l->removeChild(right_trans);
            our_root_r->removeChild(right_trans);
            right_trans.reset();
            our_root->notifyUpdate(SgUpdate::ADDED | SgUpdate::MODIFIED);
            our_root_l->notifyUpdate(SgUpdate::ADDED | SgUpdate::MODIFIED);
            our_root_r->notifyUpdate(SgUpdate::ADDED | SgUpdate::MODIFIED);
        }
    }
}

SgPosTransformPtr VRROSManager::Impl::renderAxis(const Isometry3 &_pos)
{
    SgPosTransformPtr trans = new SgPosTransform();
    trans->setPosition(_pos);

    SgLineSetPtr lset = new SgLineSet();
    lset->setLineWidth(5.0);
    SgVertexArrayPtr vt = lset->getOrCreateVertices();
    SgColorArrayPtr cols = lset->getOrCreateColors();

    Vector3f org(0, 0, 0);
    Vector3f ax(1.0, 0, 0);
    Vector3f ay(0, 1.0, 0);
    Vector3f az(0, 0, 1.0);
    Vector3f cyan(0, 1.0, 1.0);
    vt->push_back(org);
    vt->push_back(ax*0.1);
    vt->push_back(ay*0.1);
    vt->push_back(az*0.1);
    vt->push_back(ay*-4.0); // ray
    cols->push_back(ax);
    cols->push_back(ay);
    cols->push_back(az);
    cols->push_back(cyan); // ray
    lset->addLine(0, 1);
    lset->addLine(0, 2);
    lset->addLine(0, 3);
    lset->addLine(0, 4); // ray

    SgIndexArray& cidx = lset->colorIndices();
    cidx.push_back(0); cidx.push_back(0);
    cidx.push_back(1); cidx.push_back(1);
    cidx.push_back(2); cidx.push_back(2);
    cidx.push_back(3); cidx.push_back(3);

    trans->addChild(lset, false);

    return trans;
}
////
bool VRROSManager::initializeClass()
{
    VRROSManager::instance();
    return true;
}
VRROSManager *VRROSManager::instance()
{
    static VRROSManager* instance_ = new VRROSManager();
    return instance_;
}
VRROSManager::VRROSManager()
{
    impl = new Impl();
}
VRROSManager::~VRROSManager()
{
}
void VRROSManager::startROSNode()
{
    impl->startROSNode();
}
void VRROSManager::setNodeName(const std::string &_name)
{
    impl->node_name = _name;
}
Isometry3 &VRROSManager::offset()
{
    return impl->world_offset;
}
const Isometry3 &VRROSManager::offset() const
{
    return impl->world_offset;
}
void VRROSManager::setOffset(Isometry3 &_T)
{
    impl->world_offset = _T;
}
