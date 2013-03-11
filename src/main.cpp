#include "nxt_ttt/NT_Application.h"
#include "nxt_ttt/NT_Helper.h"

int main(int argc, char ** argv)
{
    STARTLOG("log.txt");

    ros::init(argc, argv, "nxt_ttt");
    nxt_ttt::Application app;

    // set TTT

    LOG("Start application...\n");
    app.start();
    STOPLOG();
    return 0;
}
