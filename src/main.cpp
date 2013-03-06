#include "nxt_ttt/NT_Application.h"

int main(int argc, char ** argv)
{
    ros::init(argc, argv, "nxt_ttt");
    nxt_ttt::Application app;
    app.start();
    return 0;
}
