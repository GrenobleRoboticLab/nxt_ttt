#ifndef NT_ABSTRACTTTT_H
#define NT_ABSTRACTTTT_H

namespace nxt_ttt {

class Application;

class AbstractTTT {

public:
    AbstractTTT();
    ~AbstractTTT();

    void            setApplication(Application* pApp);

    virtual void    play() = 0;

    void            cbColor(int x, int y, int color);
    void            cbDropped(int x, int y);

private:
    Application*    m_pApplication;

    int             m_nBoard[3][3];

}; //class AbstractTTT

} //namespace nxt_ttt

#include "NT_Application.h"

#endif // NT_ABSTRACTTTT_H
