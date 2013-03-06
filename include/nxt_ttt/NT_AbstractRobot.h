#ifndef NT_ABSTRACTROBOT_H
#define NT_ABSTRACTROBOT_H

#include "NT_Application.h"
#include "NT_AbstractTTT.h"

namespace nxt_ttt  {

class AbstractRobot {

public:
    AbstractRobot();
    ~AbstractRobot();

    void            setApplication(Application* pApp);
    void            setTTT(AbstractTTT* pTTT);
    virtual void    getColor(int x, int y) = 0;
    virtual void    dropBall(int x, int y) = 0;

private:
    Application*    m_pApplication;
    AbstractTTT*    m_pTTT;

}; //class AbstractRobot

} //namespace nxt_ttt

#endif // NT_ABSTRACTROBOT_H
