#include "AssemblerROSManager.h"
#include <cnoid/ext/robot_assembler_plugin/AssemblerManager.h>

using namespace cnoid;

bool AssemblerROSManager::initializeClass()
{
    return true;
}
AssemblerROSManager *AssemblerROSManager::instance()
{
    return nullptr;
}
AssemblerROSManager::AssemblerROSManager()
{
}
AssemblerROSManager::~AssemblerROSManager()
{
}
