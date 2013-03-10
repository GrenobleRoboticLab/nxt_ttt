#ifndef NT_ABSTRACTTTT_H
#define NT_ABSTRACTTTT_H

#include "nxt_ttt/NT_Helper.h"

namespace nxt_ttt {

class Application;

class AbstractRobot;

class AbstractTTT {

public:
    AbstractTTT();
    ~AbstractTTT();

    void            setApplication(Application* pApp);

    virtual void    play() = 0;

    void            cbColor(int x, int y, Color* color);
    void            cbDropped(int x, int y);

private:
    Application*    m_pApplication;

    Color*          board[3][3];
    bool            boardEmpty;
    bool            firstTime;

    bool            forceDrop();
    void            bestDrop();

}; //class AbstractTTT

} //namespace nxt_ttt

#include "nxt_ttt/NT_Application.h"
#include "nxt_ttt/NT_AbstractRobot.h"

#endif // NT_ABSTRACTTTT_H
