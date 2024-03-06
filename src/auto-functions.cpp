#include "auto-functions.h"
#include "PID.h"
#include "basic-functions.h"
#include "chassis.h"
#include "my-timer.h"
#include "positionNew.h"
#include "trajPlanner.h"
#include <iomanip>
#include <iostream>
using namespace std;

void trajMove(Traj traj, double _power, directionType direction) {
    // 轮距 237.6mm
    double wheelDis = 237.6 / 10; // cm
    auto myTimer = MyTimer();
    Point pos = Position::getInstance()->getPos();
    /**********************************************************************/
    // CHKVAL 2000
    // 报告
    cout << fixed << setprecision(3);
    cout << "------- Traj: (" << traj.wayPoint[0]._x << "," << traj.wayPoint[0]._y << ") -> (" << traj.wayPoint[traj.wayPoint.size() - 1]._x << "," << traj.wayPoint[traj.wayPoint.size() - 1]._y << ") -------" << endl;
    cout << "Curr Postition: (" << pos._x << "," << pos._y << ")" << endl;
    while ((traj.getTarget() - pos).mod() > 3 && myTimer.getTime() < 20000) {

        pos = Position::getInstance()->getPos();
        Point gp = traj.getGoalPoint(pos._x, pos._y);
        double dir = IMUHeading();
        // report
        cout << pos._x << "," << pos._y << "," << gp._x << "," << gp._y << endl;
        if (direction == directionType::rev) {
            dir += 180;
        }
        double x1 = pos._x, x2 = gp._x, y1 = pos._y, y2 = gp._y;
        double theta = deg2rad(90 - dir);
        double ox = (sin(theta) * (x1 * x1 - x2 * x2 - y1 * y1 - y2 * y2 + 2 * y1 * y2) + cos(theta) * 2 * x1 * (y2 - y1)) /
                    (2 * cos(theta) * (y2 - y1) + 2 * sin(theta) * (x2 - x1));
        double oy = (2 * sin(theta) * (x1 * y1 - x2 * y1) + cos(theta) * (x1 * x1 - 2 * x1 * x2 + x2 * x2 - y1 * y1 + y2 * y2)) /
                    (2 * (cos(theta) * (y2 - y1) + sin(theta) * (x1 - x2)));
        Point O = Point(ox, oy);
        double b = (gp - O).mod(), c = (pos - O).mod(), a = (gp - pos).mod();
        double dthetha = rad2deg(acos((b * b + c * c - a * a) / (2 * b * c)));
        double tarDir;
        double tDir = 90 - (gp - pos).dir() - dir;
        while (tDir < -180)
            tDir += 360;
        while (tDir > 180)
            tDir -= 360;
        double omega;
        if (tDir < 0) {
            tarDir = dir - dthetha;
            omega = deg2rad(dthetha) / ControlCycle * 1000;
        } else {
            tarDir = dir + dthetha;
            omega = -deg2rad(dthetha) / ControlCycle * 1000;
        }
        double v = deg2rad(dthetha) * b / ControlCycle * 1000;
        double vL = v - wheelDis / 2 * omega;
        double vR = v + wheelDis / 2 * omega;
        if (direction == directionType::rev) {
            double temp = vL;
            vL = -vR;
            vR = -temp;
        }
        if(tDir<-10){
          vR+=10;
        }
        if(tDir>10){
          vL+=10;
        }
        double maxV = max(abs(vL), abs(vR));
        vL = vL / maxV * _power;
        vR = vR / maxV * _power;
        Chassis::getInstance()->autoSetWheelVel(vL, vR);
        this_thread::sleep_for(ControlCycle);
    }
    Chassis::getInstance()->chassisBrake(coast);
}

void timerForwardWhileAiming(float _power, int _duration, double _tarX, double _tarY) {
    auto myTimer = MyTimer();
    while (myTimer.getTime() < _duration) {
        Point pos = Position::getInstance()->getPos();
        double _tarAng = 90 - (Point(_tarX, _tarY) - pos).dir();
        _tarAng += 180;
        while (_tarAng >= 360)
            _tarAng -= 360;
        while (_tarAng < 0)
            _tarAng += 360;
        float headingError = _tarAng - IMUHeading();
        while (headingError > 180)
            headingError -= 360;
        while (headingError < -180)
            headingError += 360;
        float powerTurn = headingError * 1.0; // kp = 1.0
        if (abs(powerTurn) > 20)
            powerTurn = sign(powerTurn) * 20; // PLimit = 20
        Chassis::getInstance()->autoSetRobotVel(_power, powerTurn);
    }
}

void moveWhileAiming(float _power, double _tarX, double _tarY, double _offset) {
    // move forward with _power for _targetPos displacement while sticking to _targetHeading angle
    // does not stop base when finishing
    // use the sign of _power to determine direction
    Point tarPos = Point(_tarX, _tarY);
    while ((tarPos - Position::getInstance()->getPos()).mod() > abs(_offset)) {
        Point pos = Position::getInstance()->getPos();
        double _tarAng = 90 - (Point(_tarX, _tarY) - pos).dir();
        _tarAng += 180;
        while (_tarAng >= 360)
            _tarAng -= 360;
        while (_tarAng < 0)
            _tarAng += 360;
        float headingError = _tarAng - IMUHeading();
        while (headingError > 180)
            headingError -= 360;
        while (headingError < -180)
            headingError += 360;
        float powerTurn = headingError * 0.8; // kp = 1.0
        if (fabs(powerTurn) > 25)
            powerTurn = sign(powerTurn) * 25; // PLimit = 20
        Chassis::getInstance()->autoSetRobotVel(_power, powerTurn);
        this_thread::sleep_for(ControlCycle);
    }
}

void posForwardRelWhileAiming(float _power, float _targetPos, double _tarX, double _tarY) {
    // move forward with _power for _targetPos displacement while sticking to _targetHeading angle
    // does not stop base when finishing
    // use the sign of _power to determine direction
    Point startPos = Position::getInstance()->getPos();
    while ((startPos - Position::getInstance()->getPos()).mod() < abs(_targetPos)) {
        Point pos = Position::getInstance()->getPos();
        double _tarAng = 90 - (Point(_tarX, _tarY) - pos).dir();
        _tarAng += 180;
        while (_tarAng >= 360)
            _tarAng -= 360;
        while (_tarAng < 0)
            _tarAng += 360;
        float headingError = _tarAng - IMUHeading();
        while (headingError > 180)
            headingError -= 360;
        while (headingError < -180)
            headingError += 360;
        float powerTurn = headingError * 1.0; // kp = 1.0
        if (fabs(powerTurn) > 20)
            powerTurn = sign(powerTurn) * 20; // PLimit = 20
        Chassis::getInstance()->autoSetRobotVel(_power, powerTurn);
        this_thread::sleep_for(ControlCycle);
    }
}

void aimAt(double _x, double _y, double _offset) {
    Point _tarPos = Point(_x, _y);
    Point pos = Position::getInstance()->getPos();
    double _tarAng = 90 - (_tarPos - pos).dir() + _offset;
    _tarAng += 180;
    while (_tarAng >= 360)
        _tarAng -= 360;
    while (_tarAng < 0)
        _tarAng += 360;
    DirPID pid;
    pid.setCoefficient(0.95, 0.3, 1.6);
    pid.setJumpTime(200);
    pid.setErrorTolerance(0.5);
    pid.setIMax(15);
    pid.setIRange(30);
    pid.setTarget(_tarAng + _offset);
    MyTimer mytimer;
    while (!pid.targetArrived() && mytimer.getTime() <= 1000) {
        pid.update(IMUHeading());

        cout << "pid: " << pid.getOutput() << ";" << IMUHeading() << endl;
        double output = pid.getOutput();
        if (abs(output) > 100)
            output = sign(output) * 100;

        Chassis::getInstance()->autoSetRobotVel(0, output);
        delay(RefreshTime);
    }
    Chassis::getInstance()->autoSetRobotVel(0, 0);
    delay(100);
}

void turnTo(double _tarAng, double _maxSpeed) {
    while (_tarAng >= 360)
        _tarAng -= 360;
    while (_tarAng < 0)
        _tarAng += 360;
    // Trajectory *traj = TrajectoryFactory::getPerciseTurnTraj(_tarAng, _maxSpeed);
    DirPID pid;
    pid.setCoefficient(0.85, 0.08, 2);
    pid.setJumpTime(200);
    pid.setErrorTolerance(1);
    pid.setIMax(10);
    pid.setIRange(20);
    pid.setTarget(_tarAng);
    MyTimer mytimer;
    while (!pid.targetArrived() && mytimer.getTime() <= 1500) {
        pid.update(IMUHeading());

        cout << "pid: " << pid.getOutput() << ";" << IMUHeading() << endl;
        double output = pid.getOutput();
        if (abs(output) > _maxSpeed)
            output = sign(output) * _maxSpeed;

        Chassis::getInstance()->autoSetRobotVel(0, output);
        delay(RefreshTime);
    }
    Chassis::getInstance()->autoSetRobotVel(0, 0);
    delay(100);
}

void timerForward(float _power, int _duration) {
    // move forward with _power for _duration msec
    // does not stop base when finishing
    Chassis::getInstance()->autoSetRobotVel(_power, 0);
    this_thread::sleep_for(_duration);
}

void timerForwardWithHeading(float _power, int _duration, double _targetHeading) {
    auto myTimer = MyTimer();
    while (myTimer.getTime() < _duration) {
        float headingError = _targetHeading - IMUHeading();
        while (headingError > 180)
            headingError -= 360;
        while (headingError < -180)
            headingError += 360;
        float powerTurn = headingError * 0.7; // kp = 1.0
        if (abs(powerTurn) > 20)
            powerTurn = sign(powerTurn) * 20; // PLimit = 20
        Chassis::getInstance()->autoSetRobotVel(_power, powerTurn);
        this_thread::sleep_for(ControlCycle);
    }
}

void posForwardRel(float _power, float _target) {
    // move forward with _power for _target displacement
    // does not stop base when finishing
    // use the sign of _power to determine direction
    Point startPos = Position::getInstance()->getPos();
    Chassis::getInstance()->autoSetRobotVel(_power, 0);
    while ((startPos - Position::getInstance()->getPos()).mod() < abs(_target)) {
    }
    Chassis::getInstance()->autoSetRobotVel(0, 0);
}

void posForwardRelWithHeading(float _power, float _targetPos, float _targetHeading) {
    // move forward with _power for _targetPos displacement while sticking to _targetHeading angle
    // does not stop base when finishing
    // use the sign of _power to determine direction
    Point startPos = Position::getInstance()->getPos();
    while ((startPos - Position::getInstance()->getPos()).mod() < abs(_targetPos)) {
        float headingError = _targetHeading - IMUHeading();
        while (headingError > 180)
            headingError -= 360;
        while (headingError < -180)
            headingError += 360;
        float powerTurn = headingError * 0.6; // kp = 1.0
        if (fabs(powerTurn) > 20)
            powerTurn = sign(powerTurn) * 20; // PLimit = 20
        Chassis::getInstance()->autoSetRobotVel(_power, powerTurn);
        this_thread::sleep_for(ControlCycle);
    }
}

void PIDPosForwardRel(float _target, bool _clawF, bool _clawB) {
    // move forward for _target displacement
    // stops base when finishing
    auto myTimer = MyTimer(); //////////
    auto pid = PID();
    pid.setCoefficient(0.22, 0.002, 1.1);
    pid.setTarget(_target);
    pid.setIMax(25);
    pid.setIRange(30);
    pid.setErrorTolerance(10);
    pid.setDTolerance(10);
    pid.setJumpTime(100);
    Point startPos = Position::getInstance()->getPos();
    while (!pid.targetArrived()) { // && myTimer.getTime() < 1000 + abs(_target * 10)) {
        pid.update((Position::getInstance()->getPos() - startPos).mod());
        Chassis::getInstance()->autoSetRobotVel(pid.getOutput(), 0);
        this_thread::sleep_for(RefreshTime);
    }
}

void softStartTimerForward(float _powerInit, float _powerFinal, int _duration) {
    // move forward with power gradually increase from _powerInit to _powerFinal within _duration msec
    // does not stop base when finishing
    auto myTimer = MyTimer();
    float step = (_powerFinal - _powerInit) / _duration;
    while (myTimer.getTime() < _duration) {
        Chassis::getInstance()->autoSetRobotVel(_powerInit + myTimer.getTime() * step, 0);
    }
}

void timerRotate(float _power, int _duration) {
    // rotate clockwise with _power for _duration msec
    // does not stop base when finishing
    Chassis::getInstance()->autoSetRobotVel(0, _power);
    this_thread::sleep_for(_duration);
}

void angleRotateRel(float _power, float _target) {
    // rotate clockwise with _power for _target angle
    // does not stop base when finishing
    float startAngle = IMUHeading();
    float power = sign(_target) * fabs(_power);
    Chassis::getInstance()->autoSetRobotVel(0, power);
    while (abs(IMUHeading() - startAngle) < abs(_target)) {
    }
}

void angleRotateAbs(float _power, float _target) {
    // rotate clockwise with _power to _target angle
    // does not stop base when finishing
    angleRotateRel(_power, _target - IMUHeading());
}

void PIDAngleRotateRel(float _target) {
    // rotate clockwise with _power for _target angle
    // stops base when finishing
    PIDAngleRotateAbs(IMUHeading() + _target);
}

void PIDAngleRotateAbs(float _target) {
    // rotate clockwise with _power to _target angle
    // stops base when finishing

    // auto myTimer = MyTimer();
    auto pid = PID();
    pid.setCoefficient(0.85, 0.02, 2.5);
    pid.setTarget(_target);
    pid.setIMax(25);
    pid.setIRange(30);
    pid.setErrorTolerance(1);
    pid.setDTolerance(5);
    pid.setJumpTime(100);
    while (!pid.targetArrived()) { // &&  myTimer.getTime() < 1000 + abbs(target * 3)) {
        pid.update(IMUHeading());
        Chassis::getInstance()->autoSetRobotVel(0, pid.getOutput());
        this_thread::sleep_for(RefreshTime);
    }
}

void softStartTimerRotate(float _powerInit, float _powerFinal, int _duration) {
    // rotate clockwise with power gradually increase from _powerInit to _powerFinal within _duration msec
    // does not stop base when finishing
    auto myTimer = MyTimer();
    float step = (_powerFinal - _powerInit) / _duration;
    while (myTimer.getTime() < _duration) {
        Chassis::getInstance()->autoSetRobotVel(0, _powerInit + myTimer.getTime() * step);
    }
}