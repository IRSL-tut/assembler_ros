#ifndef __CHOREONOID_ASSEMBLER_ROS_MANAGER_PLUGIN_H__
#define __CHOREONOID_ASSEMBLER_ROS_MANAGER_PLUGIN_H__

#include <string>

namespace cnoid
{

class AssemblerROSManager
{
public:
    static bool initializeClass();
    static AssemblerROSManager *instance();
    AssemblerROSManager();
    ~AssemblerROSManager();

    void startROSNode();
    void setNodeName(const std::string &_name);
private:
    class Impl;
    Impl *impl;
};

}
#endif // __CHOREONOID_ASSEMBLER_ROS2PLUGIN_H__
