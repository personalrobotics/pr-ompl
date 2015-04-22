#include <boost/make_shared.hpp>
#include <openrave/plugin.h>
#include <or_ompl/OMPLPlanner.h>
#include <or_ompl/OMPLConversions.h>
#include <pr_ompl/RRTConnect.h>

using namespace OpenRAVE;

namespace {

template <class Planner>
ompl::base::Planner *CreatePlanner(ompl::base::SpaceInformationPtr space_info)
{
    return new Planner(space_info);
}

}

InterfaceBasePtr CreateInterfaceValidated(
        InterfaceType type, std::string const &interfacename,
        std::istream &sinput, EnvironmentBasePtr penv)
{
    if (type == PT_Planner && interfacename == "ompl_pr_rrtconnect") {
        return boost::make_shared<or_ompl::OMPLPlanner>(
            penv, &CreatePlanner<pr_ompl::RRTConnect>
        );
    } else {
        return InterfaceBasePtr();
    }
}

void GetPluginAttributesValidated(PLUGININFO &info)
{
    info.interfacenames[PT_Planner].push_back("OMPL_PR_RRTConnect");

    // Forward OMPL log messages to OpenRAVE.
    ompl::msg::setLogLevel(ompl::msg::LOG_DEBUG);
    ompl::msg::useOutputHandler(new or_ompl::OpenRAVEHandler);
}

RAVE_PLUGIN_API void DestroyPlugin()
{
}
