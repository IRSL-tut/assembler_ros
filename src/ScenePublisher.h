#ifndef __CNOID_ASSEMBLER_ROS_SCENE_PUBLISHER_H__
#define __CNOID_ASSEMBLER_ROS_SCENE_PUBLISHER_H__

#include <string>
#include <cnoid/EigenTypes>

namespace cnoid {

//class CNOID_EXPORT ScenePublisher
class ScenePublisher
{
public:
    static bool initializeClass();
    static ScenePublisher *instance();

    ScenePublisher();
    ~ScenePublisher();

    void startROSNode();
    void setNodeName(const std::string &_name);

    void setFrequency(double _hz);

    void resetOffset();

    Isometry3 &offset();
    void setOffset(const Isometry3 &_T);
private:
    class Impl;
    Impl* impl;
};

}
#endif // __CNOID_ASSEMBLER_ROS_SCENE_PUBLISHER_H__
