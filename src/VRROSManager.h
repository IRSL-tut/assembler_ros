#ifndef __CNOID_ASSEMBLER_ROS_MANAGER_H__
#define __CNOID_ASSEMBLER_ROS_MANAGER_H__

#include <string>
#include <cnoid/EigenTypes>

namespace cnoid
{

class VRROSManager
{
public:
    static bool initializeClass();
    static VRROSManager *instance();
    VRROSManager();
    ~VRROSManager();

    void startROSNode();
    void setNodeName(const std::string &_name);

    Isometry3& offset();
    const Isometry3& offset() const;
    void setOffset(Isometry3 &_T);
private:
    Isometry3 T_;
    class Impl;
    Impl *impl;
};

}
#endif // __CHOREONOID_ASSEMBLER_ROS_MANAGER_H__
