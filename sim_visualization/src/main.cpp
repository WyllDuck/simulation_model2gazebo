#include <trace3D.h>


int main(int argc, char **argv)
{
    // Init Node:
    ros::init(argc, argv, "visualization");

    // NOTE: First add the subscribe(s) topic and then the publish topic, namespace "vis"
    Trace *trace_truth = new Trace("/truth/global/linear/position", "/vis/truth/global/linear/position");

    ros::spin();
    return 0;
}