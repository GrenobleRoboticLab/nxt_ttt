#ifndef NT_MOTOR_H
#define NT_MOTOR_H

#include "NT_AbstractRobot.h"
#include <nxt_msgs/JointCommand.h>
#include <ros/ros.h>

namespace nxt_ttt {

class Robot;

class MotorState {
public:
    MotorState(const std::string & sName = "", double dEffort = 0.0, double dPosition = 0.0, double dVelocity = 0.0);
    ~MotorState();

    const MotorState&       operator=(const MotorState & state);
    bool                    operator==(const MotorState & state);
    bool                    operator!=(const MotorState & state);

    void                    initFrom(const MotorState & state);
    bool                    compare(const MotorState & state);

    void                    setEffort(double dEffort)           { m_dEffort = dEffort;      }
    void                    setPosition(double dPosition)       { m_dPosition = dPosition;  }
    void                    setVelocity(double dVelocity)       { m_dVelocity = dVelocity;  }
    void                    setName(const std::string & sName)  { m_sName = sName;          }


    double                  getEffort()                   const { return m_dEffort;         }
    double                  getPosition()                 const { return m_dPosition;       }
    double                  getVelocity()                 const { return m_dVelocity;       }
    const std::string&      getName()                     const { return m_sName;           }

private:
    std::string             m_sName;

    double                  m_dPosition;
    double                  m_dEffort;
    double                  m_dVelocity;
}; // class MotorState

class Motor
{
private:
    Motor(const Motor&);
    const Motor& operator=(const Motor&);
public:
    Motor(ros::NodeHandle & nh, const std::string & sName, AbstractRobot * pParent = NULL);
    virtual ~Motor();

    bool                    update(const MotorState & state);
    bool                    rotate(double dRadAngle);
    bool                    rollback();
    void                    stop();

    void                    setName(const std::string & sName) { m_sName = sName;   }
    std::string             getName()                    const { return m_sName;    }
    double                  getPos()                     const { return m_fPos;     }

private:
    ros::Publisher          m_Publisher;
    nxt_msgs::JointCommand  m_Command;

    AbstractRobot*          m_pParent;
    bool                    m_bHasGoal;
    double                  m_dLastRotation;

    std::string             m_sName;
    float                   m_fPosDesi;
    float                   m_fPos;
    float                   m_fEffDesi;
    float                   m_fEff;
    float                   m_fVelocity;

    void                    checkGoal();
    void                    publish();
}; // class Motor

} // namespace nxt_ttt

#endif // NT_MOTOR_H
