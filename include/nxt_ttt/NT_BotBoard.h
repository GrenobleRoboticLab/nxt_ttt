#ifndef NT_BOTBOARD_H
#define NT_BOTBOARD_H

namespace nxt_ttt {

class BotBoard {
public:
    BotBoard();
    ~BotBoard();

    void                    updatePlatPos(double dPlatMotorPos);
    void                    updateSlidePos(double dSlideMotorPos);

    double                  getPlatRotation(int x, int y);
    double                  getColorSlideRotation(int x, int y);
    double                  getDropSlideRotation(int x, int y);

private:
    double                  m_dPlatAngle;
    double                  m_dSlideAngle;

    double                  m_dFirstPlatMotorPos;
    double                  m_dLastPlatMotorPos;

    double                  m_dFirstSlideMotorPos;
    double                  m_dLastSlideMotorPos;
}; // class BotBoard

}; // namespace nxt_ttt

#endif // NT_BOTBOARD_H
