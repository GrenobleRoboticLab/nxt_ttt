#ifndef NT_TTT_H
#define NT_TTT_H

#include "nxt_ttt/NT_AbstractTTT.h"

namespace nxt_ttt
{

class TTT : public AbstractTTT
{
private:
    enum TTTState {
        TS_SCAN,
        TS_DROP,
        TS_NONE
    };

    enum CaseFlag {
        CF_WARNING,
        CF_CHOICE,
        CF_FREE,
        CF_WIN,
        CF_LOSE
    };

    struct CheckResult {
        int lastX;
        int lastY;
        CaseFlag flag;
    };

public:
    TTT();
    virtual ~TTT();

    virtual void    play();

    virtual void    cbColor(int x, int y, Color color);
    virtual void    cbDropped(int x, int y);

private:

    CheckResult     checkRow(int j);
    CheckResult     checkCol(int i);
    CheckResult     checkDiag(int i);

    Color           m_Board[3][3];
    bool            m_bBoardEmpty;
    bool            m_bFirstTime;

    int             m_nCurrentX;
    int             m_nCurrentY;
    TTTState        m_wCurrentState;

    void            scan();
    void            continueScan();
    void            treat();
    bool            applyResult(CheckResult check);
    void            bestDrop();

    void            printBoard();

}; // class TTT

}

#endif // NT_TTT_H
