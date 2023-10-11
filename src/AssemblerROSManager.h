#ifndef __CHOREONOID_ASSEMBLER_ROS_MANAGER_PLUGIN_H__
#define __CHOREONOID_ASSEMBLER_ROS_MANAGER_PLUGIN_H__

// #include <cnoid/ext/robot_assembler_plugin/AssemblerManager.h>

namespace cnoid
{

class AssemblerROSManager
{
public:
    static bool initializeClass();
    static AssemblerROSManager *instance();
    AssemblerROSManager();
    ~AssemblerROSManager();
};

}
#endif // __CHOREONOID_ASSEMBLER_ROS2PLUGIN_H__
