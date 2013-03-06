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

private:
    Application*    m_pApplication;


}; //class AbstractTTT

} //namespace nxt_ttt


#include "NT_Application.h"
#endif // NT_ABSTRACTTTT_H
