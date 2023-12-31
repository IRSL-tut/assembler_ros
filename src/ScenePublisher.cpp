/**

*/

#include "ScenePublisher.h"
#include <cnoid/Archive>
#include <cnoid/ItemManager>
#include <cnoid/PutPropertyFunction>
#include <cnoid/Timer>
#include <cnoid/SceneView>
#include <cnoid/SceneWidget>
#include <cnoid/SceneCameras>
#include <rclcpp/rclcpp.hpp>
#include <tf2_ros/transform_listener.h>
#include <tf2_ros/buffer.h>
#include <tf2_eigen/tf2_eigen.hpp>
#include <sensor_msgs/msg/compressed_image.hpp>
#include <image_transport/image_transport.hpp>
#include <cv_bridge/cv_bridge.h>
#include <opencv2/imgcodecs.hpp> // for compress
#include <iostream>
#include <cmath>
#include <cstring>
#include <thread>
#include <mutex>

#include <cnoid/SceneRenderer>

using namespace std;
using namespace cnoid;
using std::placeholders::_1;
using std::placeholders::_2;

#define MAX_THREAD 3

#if 0
#define FOV 1.5504
#define BASE_WIDTH  1280
#define BASE_HEIGHT 1280
#define NEW_WIDTH 1140
#define NEW_HEIGHT 1148
#define L_X_OFFSET 0
#define R_X_OFFSET 140
#define L_Y_OFFSET 46
#define R_Y_OFFSET 46
#endif

//1400x1420
//makeROI(1572, 1476, hmd_width=1400, hmd_height=1420)
//base_image width:1572 height:1476
//(0, 3, 1400, 1420); //Left
//(172, 53, 1400, 1420); //Right
//fov: 1.4733498959586218
#if 1 // 100%
#define FOV 1.4733498959586218
#define BASE_WIDTH  1572
#define BASE_HEIGHT 1476
#define NEW_WIDTH 1400
#define NEW_HEIGHT 1420
#define L_X_OFFSET 0
#define R_X_OFFSET 172
#define L_Y_OFFSET 53//3 or 53
#define R_Y_OFFSET 53//3 or 53
#endif

#if 0 // 100%
#define FOV 1.4693906448636416
#define BASE_WIDTH  2320
#define BASE_HEIGHT 2172
#define NEW_WIDTH 2064
#define NEW_HEIGHT 2096
#define L_X_OFFSET 1
#define R_X_OFFSET 255
#define L_Y_OFFSET 1//75
#define R_Y_OFFSET 1//75
#endif

namespace cnoid {

struct ThreadInfo
{
    QImage left;
    QImage right;
    bool finished;
    std::thread *thread;
    int id;
    ThreadInfo() : left(nullptr), right(nullptr), finished(false), thread(nullptr) {}

    ~ThreadInfo() {
        // std::cerr << "dest(" << this << ")[" << finished << "] : " << id << std::endl;
        if(!!thread) {
            delete thread;
        }
    }
};

class ScenePublisher::Impl : public rclcpp::Node
{
public:
    ScenePublisher *self;

    double publishingRate;
    int compressionLevel;
    double fov_;
    long counter;

    Impl(ScenePublisher *self);
    Impl(ScenePublisher *self, const Impl &org);
    ~Impl();

    void initialize();

    Timer tm;
    // ros publishers
    image_transport::Publisher pub;
    image_transport::CameraPublisher pub_cam;
    rclcpp::Publisher<sensor_msgs::msg::CompressedImage>::SharedPtr compress_pub;
    std::shared_ptr<tf2_ros::TransformListener> tf_listener_{nullptr};
    std::unique_ptr<tf2_ros::Buffer> tf_buffer_;
    rclcpp::Clock ros_clock;

    Eigen::Isometry3d hmd_pos;
    Eigen::Isometry3d hmd_offset;

    std::mutex pub_mutex;
    std::vector<ThreadInfo *>thread_list;

    void runCycle() {
        std::string toFrame = "world";
        //std::string fromFrame = "world_vive";
        std::string fromFrame = "hmd";
        try {
            geometry_msgs::msg::TransformStamped tmp = tf_buffer_->lookupTransform(toFrame, fromFrame, tf2::TimePointZero);
            //tmp.header.frame_id;
            //tmp.header.stamp;
            //tmp.header.child_frame_id;
            hmd_pos = tf2::transformToEigen(tmp);
            hmd_pos = hmd_offset * tf2::transformToEigen(tmp);
            Vector3 p(hmd_pos.translation());
            //RCLCPP_INFO(this->get_logger(), "cam_pos(p) %f %f %f",p.x(), p.y(), p.z());
            Quaternion q(hmd_pos.linear());
            //RCLCPP_INFO(this->get_logger(), "cam_pos(q) %f %f %f %f", q.x(), q.y(), q.z(), q.w());
        } catch (const tf2::TransformException & ex) {
            RCLCPP_INFO(
                this->get_logger(), "Could not transform %s to %s: %s",
                toFrame.c_str(), fromFrame.c_str(), ex.what());
            //return;
        }
        publishImage();
    }
    void publishImage() {
        std::vector<SceneView *> view_instances = SceneView::instances();
        if (view_instances.size() > 2) {
            //std::cerr << "sceneWidget: 0/" << view_instances.at(1)->name();
            //std::cerr << ", 1/" << view_instances.at(2)->name() << std::endl;
            {// left scene
                //view_instances.at(1)->sceneWidget()->setScreenSize(1280, 1280);
                view_instances.at(1)->sceneWidget()->builtinPerspectiveCamera()->setFieldOfView(fov_);
                //view_instances.at(1)->sceneWidget()->builtinPerspectiveCamera()->notifyUpdate();
                AngleAxis q( 0.0, Vector3::UnitY());
                Eigen::Translation3d tr(-0.0341, 0, 0);
                Isometry3 cam = tr * q;
                Isometry3 cur = hmd_pos * cam;
                {
                    Vector3 v(cur.translation());
                    Quaternion q(cur.linear());
                    //std::cerr << "lpos: " << v.x() << ", " << v.y() << ", " << v.z() << std::endl;
                    //std::cerr << "l  q: " << q.x() << ", " << q.y() << ", " << q.z() << ", " << q.w() << std::endl;
                }
                view_instances.at(1)->sceneWidget()->builtinCameraTransform()->setPosition(cur);
            }
            {// right scene
                //view_instances.at(2)->sceneWidget()->setScreenSize(1280, 1280);
                view_instances.at(2)->sceneWidget()->builtinPerspectiveCamera()->setFieldOfView(fov_);
                //view_instances.at(2)->sceneWidget()->builtinPerspectiveCamera()->notifyUpdate();
                AngleAxis q( 0.0, Vector3::UnitY());
                Eigen::Translation3d tr(0.0341, 0, 0);
                Isometry3 cam = tr * q;
                Isometry3 cur = hmd_pos * cam;
                {
                    Vector3 v(cur.translation());
                    Quaternion q(cur.linear());
                    //std::cerr << "rpos: " << v.x() << ", " << v.y() << ", " << v.z() << std::endl;
                    //std::cerr << "r  q: " << q.x() << ", " << q.y() << ", " << q.z() << ", " << q.w() << std::endl;
                }
                view_instances.at(2)->sceneWidget()->builtinCameraTransform()->setPosition(cur);
            }
            //
            //view_instances.at(0)->sceneWidget()->setScreenSize(1280, 720);
            //view_instances.at(0)->sceneWidget()->builtinPerspectiveCamera()->setFieldOfView(fov_);
            //view_instances.at(0)->sceneWidget()->builtinCameraTransform()->setPosition(hmd_pos);
            //view_instances.at(0)->sceneWidget()->renderScene(true);//
            // TODO:: render HMD, controllers
            view_instances.at(1)->sceneWidget()->renderScene(true);//
            view_instances.at(2)->sceneWidget()->renderScene(true);//
#if 0 // Raw Image
            publishCameraRaw(view_instances.at(1)->sceneWidget(),
                          view_instances.at(2)->sceneWidget());
#endif
#if 0 // Compressed Image
            publishCameraComperssed(view_instances.at(1)->sceneWidget(),
                                    view_instances.at(2)->sceneWidget());
#endif
#if 1 // Compress thread
            publishCameraCompressedThread(view_instances.at(1)->sceneWidget(),
                                          view_instances.at(2)->sceneWidget());
#endif
            counter++;
            return;
        }
#if 0
        ////
        QImage im = SceneView::instance()->sceneWidget()->getImage();

        sensor_msgs::msg::Image msg;
        msg.height = im.height();
        msg.width =  im.width();
        msg.encoding = "bgra8";
        msg.step = msg.width * 4;
        msg.data.resize(msg.height * msg.width * 4);
        for (int i = 0; i < msg.height * msg.width * 4; i++) {
            msg.data[i] = im.bits()[i];
        }
        pub.publish(msg);
#endif
    }
    bool makeImageMsg(sensor_msgs::msg::Image &msg, QImage &im_l, QImage &im_r)
    {
        if(im_l.height() != im_r.height()) {
            return false;
        }
        if(im_l.width() != im_r.width()) {
            return false;
        }
        im_r.convertTo(QImage::Format_RGB888);
        im_l.convertTo(QImage::Format_RGB888);
        if ((im_l.width() *  3) != im_l.bytesPerLine()) {
            std::cerr << "warn: Screen size may require multiple of 4" << std::endl;
            std::cerr << "org: " << im_l.width() << " x " << im_l.height() << " / " << im_l.bytesPerLine() << std::endl;
        }
        cv::Mat new_cv_l;
        cv::Mat new_cv_r;
        {
            cv::Mat cv_l(im_l.height(), im_l.width(), CV_8UC3, im_l.bits());
            cv::Mat cv_r(im_r.height(), im_r.width(), CV_8UC3, im_r.bits());
            cv::Rect roiL( L_X_OFFSET, L_Y_OFFSET, NEW_WIDTH, NEW_HEIGHT);
            cv::Rect roiR( R_X_OFFSET, R_Y_OFFSET, NEW_WIDTH, NEW_HEIGHT);
            cv::Mat(cv_l, roiL).copyTo(new_cv_l);
            cv::Mat(cv_r, roiR).copyTo(new_cv_r);
        }
        msg.width  = new_cv_l.cols;
        msg.height = new_cv_l.rows + new_cv_r.rows;
        msg.encoding = "rgb8";
        msg.step = msg.width * 3;
        msg.data.resize(msg.height * msg.width * 3);
        std::memcpy(msg.data.data(), new_cv_l.ptr(), msg.step * new_cv_l.rows);
        std::memcpy(msg.data.data()+(msg.step * new_cv_l.rows), new_cv_r.ptr(), msg.step * new_cv_r.rows);

        rclcpp::Time now = ros_clock.now();
        msg.header.stamp = now;
        msg.header.frame_id = "cnoid";

#if 0 // debug
        sensor_msgs::msg::CameraInfo info;
        info.header = msg.header;
        pub_cam.publish(msg, info);
#endif
        return true;
    }
    void publishCameraCompressed(SceneWidget *left, SceneWidget *right)
    {
        sensor_msgs::msg::Image msg;
        QImage im_l =  left->getImage();
        QImage im_r = right->getImage();
        if (makeImageMsg(msg, im_l, im_r)) {
            std::shared_ptr<sensor_msgs::msg::CompressedImage> compressed(new sensor_msgs::msg::CompressedImage());
            compressed->header = msg.header;
            compressed->format = msg.encoding;
            std::vector<int> params;
            params.reserve(2);
            params.emplace_back(cv::IMWRITE_PNG_COMPRESSION);
            params.emplace_back(compressionLevel);
            cv_bridge::CvImageConstPtr cv_ptr = cv_bridge::toCvShare(msg, compressed);
            cv::imencode(".png", cv_ptr->image, compressed->data, params);
            compress_pub->publish(*compressed);
        }
    }
    void publishCameraRaw(SceneWidget *left, SceneWidget *right)
    {
        sensor_msgs::msg::Image msg;
        QImage im_l =  left->getImage();
        QImage im_r = right->getImage();
        if (makeImageMsg(msg, im_l, im_r)) {
            //
            sensor_msgs::msg::CameraInfo info;
            info.header = msg.header;
            // [TODO] add camera_info
            //const double minLength = std::min(msg.width, msg.height/2);
            const double focalLength = msg.width / 2.0 / tan(0.5*fov_);
            //const double principalPointX = (info.width - 1.0) / 2.0;
            //const double principalPointY = (info.height - 1.0) / 2.0;
            info.k[0] = focalLength;
            info.k[4] = focalLength;
            //
            pub_cam.publish(msg, info);
        }
    }
    void publishCameraCompressedThread(SceneWidget *left, SceneWidget *right) {
        // update thread_list
        bool finished = false;
        while(!finished) {
            auto pos = thread_list.end();
            for(auto it = thread_list.begin(); it != thread_list.end(); it++) {
                {
                    std::lock_guard<std::mutex> lock(pub_mutex);
                    if((*it)->finished) {
                        pos = it;
                        break;
                    }
                }
            }
            if (pos == thread_list.end()) {
                finished = true;
            } else {
                ThreadInfo *tgt = (*pos);
                tgt->thread->join();
                thread_list.erase(pos);
                delete tgt;
            }
        }
        if (thread_list.size() > MAX_THREAD) {
            std::cerr << "skipping image publication : " << counter << std::endl;
            return;
        }
        //
        ThreadInfo *h = new ThreadInfo();
        h->left  =  left->getImage();
        h->right = right->getImage();
        h->thread = new std::thread(std::bind(&ScenePublisher::Impl::publish_, this, _1), h);
        h->id = counter;
        thread_list.push_back(h);
    }
    void publish_(ThreadInfo *h)
    {
        sensor_msgs::msg::Image msg;
        if (makeImageMsg(msg, h->left, h->right))  {
            std::shared_ptr<sensor_msgs::msg::CompressedImage> compressed(new sensor_msgs::msg::CompressedImage());
            compressed->header = msg.header;
            compressed->format = msg.encoding;
            std::vector<int> params;
            params.reserve(2);
            params.emplace_back(cv::IMWRITE_PNG_COMPRESSION);
            params.emplace_back(compressionLevel); //
            cv_bridge::CvImageConstPtr cv_ptr = cv_bridge::toCvShare(msg, compressed);
            cv::imencode(".png", cv_ptr->image, compressed->data, params);
            {
                std::lock_guard<std::mutex> lock(pub_mutex);
                compress_pub->publish(*compressed);
                h->finished = true;
                //std::cerr << "finish(" << h << ") : " << h->id << std::endl;
            }
        }
    }
};

}  // namespace cnoid

//void ScenePublisher::initializeClass(ExtensionManager *ext)
//{
//    ext->itemManager().registerClass<ScenePublisher>(N_("ScenePublisher"));
//    ext->itemManager().addCreationPanel<ScenePublisher>();
//}

ScenePublisher::Impl::Impl(ScenePublisher *self)
    : rclcpp::Node("scene_publisher", rclcpp::NodeOptions())
    , self(self),  tm(), ros_clock(RCL_ROS_TIME)
{
    counter = 0;
    std::cerr << "Created Impl" << std::endl;
    publishingRate = 30.0;
    compressionLevel = 2;

    initialize();
}

ScenePublisher::Impl::Impl(ScenePublisher *self, const Impl &org)
    : rclcpp::Node("scene_publisher", rclcpp::NodeOptions())
    , self(self)
{
    std::cerr << "Copied Impl" << std::endl;
    publishingRate = org.publishingRate;
    initialize();
}

void ScenePublisher::Impl::initialize()
{
    fov_ = FOV;
    //pub = image_transport::create_publisher(this, "scene/image");
    // raw
    pub_cam = image_transport::create_camera_publisher(this, "scene_camera/images");
    // compressed
    compress_pub = this->create_publisher<sensor_msgs::msg::CompressedImage>("scene_camera/compressed_images", 2);
    //
    tf_buffer_ = std::make_unique<tf2_ros::Buffer>(this->get_clock());
    tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_, this);

    hmd_pos = Eigen::Isometry3d::Identity();
    {
        AngleAxis q( 0.0, Vector3::UnitY());
        Eigen::Translation3d tr(0, -0.2, -1.4);
        hmd_offset = tr * q;
    }

    std::vector<SceneView *> view_instances = SceneView::instances();
    if (view_instances.size() > 2) {
      view_instances.at(1)->sceneWidget()->setScreenSize(BASE_WIDTH, BASE_HEIGHT);
      view_instances.at(2)->sceneWidget()->setScreenSize(BASE_WIDTH, BASE_HEIGHT);
    }
    tm.sigTimeout().connect( [this]() { this->runCycle(); });

    int interval_ms = 1000/publishingRate;
    tm.start(interval_ms);
}

ScenePublisher::Impl::~Impl()
{
}

bool ScenePublisher::initializeClass()
{
    return true;
}

ScenePublisher* ScenePublisher::instance()
{
    static ScenePublisher *_instance = new ScenePublisher();
    return _instance;
}

ScenePublisher::ScenePublisher()
{
    std::cerr << "ScenePublisher : created" << std::endl;
    impl = new Impl(this);
}
#if 0
ScenePublisher::ScenePublisher(const ScenePublisher &org)
    : Item(org)
{
    std::cerr << "ScenePublisher : copied" << std::endl;
    impl = new Impl(this, *org.impl);
}
#endif
ScenePublisher::~ScenePublisher()
{
    delete impl;
}

#if 0
Item *ScenePublisher::doDuplicate() const
{
    return new ScenePublisher(*this);
}

void ScenePublisher::doPutProperties(PutPropertyFunction &putProperty)
{
    putProperty.decimals(2).min(0.0)(_("image publishing rate"),
                                     impl->publishingRate);
    putProperty.decimals(0).min(0).max(9)(_("image compression level"),
                                          impl->compressionLevel);
}

bool ScenePublisher::store(Archive &archive)
{
    archive.write("scene_publishing_rate", impl->publishingRate);
    archive.write("image_compression_level", impl->compressionLevel);
    return true;
}

bool ScenePublisher::restore(const Archive &archive)
{
    archive.read("scene_publishing_rate", impl->publishingRate);
    archive.read("image_compression_level", impl->compressionLevel);
    return true;
}
#endif
