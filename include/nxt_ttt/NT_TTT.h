#ifndef NT_TTT_H
#define NT_TTT_H

#include "nxt_ttt/NT_AbstractTTT.h"

namespace nxt_ttt
{


struct Point {
    Point(){ ; }
    Point(int _x, int _y) {
        x = _x;
        y = _y;
    }

    int x;
    int y;
};

// coordonnées des cases pour chaque ligne
const    Point LINES[8][3] = {
    {Point(0,0),Point(1,0),Point(2,0)}, //UPROW
    {Point(0,1),Point(1,1),Point(2,1)}, //MIDDLEROW
    {Point(0,2),Point(1,2),Point(2,2)}, //DOWNROW
    {Point(0,2),Point(0,1),Point(0,0)}, //LEFTCOL
    {Point(1,2),Point(1,1),Point(1,0)}, //MIDDLECOL
    {Point(2,2),Point(2,1),Point(2,0)}, //RIGHTCOL
    {Point(0,2),Point(1,1),Point(2,0)}, //UPDOWNDIAG
    {Point(0,0),Point(1,1),Point(2,2)}, //DOWNUPDIAG
};

class TTT : public AbstractTTT
{
private:
    enum TTTState {
        TS_SCAN,
        TS_DROP,
        TS_NONE
    };


    enum StateFlag {
        SF_BOTONE = 0,
        SF_EMPTY = 1,
        SF_PLAYERONE = 2,
        SF_ONEEACH = 3,
        SF_NONE = 4
    };

public:
    TTT();
    virtual ~TTT();

    virtual void    play();

    virtual void    cbColor(int x, int y, Color color);
    virtual void    cbDropped(int x, int y);

private:

    Color           m_Board[3][3];
    bool            m_bBoardEmpty;
    bool            m_bFirstTime;

    int             m_nCurrentX;
    int             m_nCurrentY;
    TTTState        m_wCurrentState;

    void            scan();
    void            continueScan();

    void            exploreLines();
    int             weightCase(Point point);
    Point           bestCase();

    void            treat();

    void            printBoard();


}; // class TTT


}

#endif // NT_TTT_H
