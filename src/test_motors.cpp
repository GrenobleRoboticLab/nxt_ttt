#include "TestBot2.h"

using namespace nxt_ttt;

int main(int argc, char ** argv)
{
    STARTLOG("log.txt");

    {
        ros::init(argc, argv, "test_motors");

        TestBot2 tb;
        tb.ask();

        ros::spin();
    }

    STOPLOG();
    return 0;
}
