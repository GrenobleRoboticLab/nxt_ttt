#ifndef NT_ABSTRACTTTT_H
#define NT_ABSTRACTTTT_H

#include "nxt_ttt/NT_Helper.h"

namespace nxt_ttt {

class Application;

class AbstractRobot;

class AbstractTTT {

public:
    AbstractTTT();
    virtual ~AbstractTTT();

    void            setApplication(Application* pApp);
    void            setRobot(AbstractRobot* pRobot);

    virtual void    play() = 0;

    virtual void    cbColor(int x, int y, Color color) = 0;
    virtual void    cbDropped(int x, int y) = 0;

protected:
    Application*    m_pApplication;
    AbstractRobot*  m_pRobot;

}; //class AbstractTTT

} //namespace nxt_ttt

#include "nxt_ttt/NT_Application.h"
#include "nxt_ttt/NT_AbstractRobot.h"

#endif // NT_ABSTRACTTTT_H
