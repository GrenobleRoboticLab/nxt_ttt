#ifndef NT_APPLICATION_H
#define NT_APPLICATION_H

#include <ros/ros.h>
#include "NT_AbstractTTT.h"

namespace nxt_ttt {
enum NT_USER {
    NT_PLAYER,
    NT_TTT
};


class Application {
public:
    Application();
    ~Application();

    void            start();
    void            stop();

    void            setTTT(AbstractTTT* pTTT);

    void            cbPlayed(NT_USER user);
    void            cbEnd(NT_USER winner);

private:
    NT_USER         m_CurrentPlayer;
    AbstractTTT*    m_pTTT;
    ros::NodeHandle m_NodeHandle;


}; //class Application

}

#endif // NT_APPLICATION_H
