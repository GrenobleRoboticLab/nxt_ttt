#include "nxt_ttt/NT_Application.h"

int main(int argc, char ** argv)
{
    ros::init(argc, argv, "nxt_ttt");
    nxt_ttt::Application app;

    // set TTT

    app.start();
    return 0;
}
