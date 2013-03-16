#ifndef NT_BOTBOARD_H
#define NT_BOTBOARD_H

namespace nxt_ttt {

class BotBoard {
public:
    BotBoard();
    ~BotBoard();

    double                  getPlatRotation(int x, int y);
    double                  getColorSlideRotation(int x, int y);
    double                  getDropSlideRotation(int x, int y);
}; // class BotBoard

}; // namespace nxt_ttt

#endif // NT_BOTBOARD_H
