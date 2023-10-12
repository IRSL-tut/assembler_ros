#ifndef __CNOID_ASSEMBLER_ROS_SCENE_PUBLISHER_H__
#define __CNOID_ASSEMBLER_ROS_SCENE_PUBLISHER_H__

#include <string>

namespace cnoid {

class CNOID_EXPORT ScenePublisher
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
    void setOffset(Isometry3 &_T);
private:
    class Impl;
    Impl* impl;
};


}  // __CNOID_ASSEMBLER_ROS_SCENE_PUBLISHER_H__

#endif
