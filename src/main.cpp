#include "nxt_ttt/NT_Application.h"
#include "nxt_ttt/NT_TTT.h"
#include "nxt_ttt/NT_Robot.h"
#include "nxt_ttt/NT_Helper.h"

int main(int argc, char ** argv)
{
    STARTLOG("log.txt");

    {
        ros::init(argc, argv, "nxt_ttt");
        LOG("INFO : main : Creating app, ttt and robot\n");
        nxt_ttt::Application    app;
        nxt_ttt::TTT            ttt;
        nxt_ttt::Robot          robot;

        LOG("INFO : main : Setting variables for app, ttt and robot\n");
        app.setRobot(&robot);
        app.setTTT(&ttt);
        ttt.setRobot(&robot);

        LOG("INFO : Start application...\n");
        app.start();
    }

    STOPLOG();
    return 0;
}
