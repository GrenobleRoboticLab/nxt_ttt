#ifndef NT_ABSTRACTROBOT_H
#define NT_ABSTRACTROBOT_H

#include "nxt_ttt/NT_Application.h"
#include "nxt_ttt/NT_AbstractTTT.h"
#include "nxt_ttt/NT_Helper.h"

namespace nxt_ttt  {

class AbstractRobot {

public:
    AbstractRobot();
    virtual ~AbstractRobot();

    void            setApplication(Application* pApp);
    void            setTTT(AbstractTTT* pTTT);

    virtual void    getColor(int x, int y) = 0;
    virtual void    dropBall(int x, int y) = 0;
    virtual void    waitPlayerPlay() = 0;
    virtual void    actionPerformed(const std::string & sMotorName) = 0;

protected:
    Application*    m_pApplication;
    AbstractTTT*    m_pTTT;

}; //class AbstractRobot

} //namespace nxt_ttt

#endif // NT_ABSTRACTROBOT_H
