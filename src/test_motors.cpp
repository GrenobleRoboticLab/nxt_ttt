#include "NT_TestBot.h"

using namespace nxt_ttt;

int main(int argc, char ** argv)
{
    STARTLOG("log.txt");

    {
        ros::init(argc, argv, "test_motors");

        TestBot tb;
        tb.ask();

        ros::spin();
    }

    STOPLOG();
    return 0;
}
