#ifndef __CHOREONOID_ASSEMBLER_ROS_PLUGIN_H__
#define __CHOREONOID_ASSEMBLER_ROS_PLUGIN_H__

#include <cnoid/Plugin>

class VRROSPlugin : public cnoid::Plugin
{
public:
    VRROSPlugin();
    virtual bool initialize();
private:
    class Impl;
    Impl *impl;
};

#endif // __CHOREONOID_ASSEMBLER_ROS2PLUGIN_H__
