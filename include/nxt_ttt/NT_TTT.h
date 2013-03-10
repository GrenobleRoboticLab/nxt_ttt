#ifndef NT_TTT_H
#define NT_TTT_H

#include "nxt_ttt/NT_AbstractTTT.h"
#include "nxt_ttt/NT_Helper.h"

namespace nxt_ttt
{

class TTT : public AbstractTTT
{

public:
    TTT() : AbstractTTT() {}

    virtual void  play();

private:

}; // class TTT

}

#endif // NT_TTT_H
