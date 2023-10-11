#ifndef __CHOREONOID_ASSEMBLER_ROS_PLUGIN_H__
#define __CHOREONOID_ASSEMBLER_ROS_PLUGIN_H__

#include <cnoid/Plugin>

class AssemblerROSPlugin : public cnoid::Plugin
{
public:
    AssemblerROSPlugin();
    virtual bool initialize();
private:
    class Impl;
    Impl *impl;
};

#endif // __CHOREONOID_ASSEMBLER_ROS2PLUGIN_H__
