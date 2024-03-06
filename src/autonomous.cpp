#include "autonomous.h"
#include "auto-functions.h"
#include "basic-functions.h"
#include "chassis.h"
#include "trajPlanner.h"
#include "vex.h"
#include <iostream>
using namespace std;
using namespace vex;

// void autonomous()
// {
//     turnTo(90);
//     while ((true))
//     {
//         this_thread::sleep_for(10);
//     }
// }

void autonomous() {
    Brain.Screen.setCursor(5, 5);
    Brain.Screen.print("autonomous");

    TrajPlanner planner;
    planner.addWayPoint(0, 0, 0);
    planner.addWayPoint(100, 100, 0);
    // planner.addWayPoint(0, 0, 0);
    Traj traj = planner.generateBezierTraj(200,true);
    trajMove(traj, 20, fwd);
    Chassis::getInstance()->chassisBrake(brake);

    // // moveIntaker(-100);
    // posForwardRelWithHeading(-80, 40, 180);
    // Chassis::getInstance()->chassisBrake(coast);
    // this_thread::sleep_for(1000);
    // moveIntaker(-100);
    // timerForwardWithHeading(-90, 130, 180);
    // Chassis::getInstance()->autoSetRobotVel(0, 0);
    // Chassis::getInstance()->chassisBrake(coast);
    // this_thread::sleep_for(500);
    // moveIntaker(0);

    // posForwardRelWithHeading(80, 20, 0);
    // Chassis::getInstance()->chassisBrake(coast);
    // this_thread::sleep_for(500);

    // aimAt(-38, 65, 0);
    // Chassis::getInstance()->chassisBrake(coast);
    // this_thread::sleep_for(500);

    // Brain.Screen.setCursor(1, 1);
    // Brain.Screen.print(" %.3lf", IMUHeading());
    // Brain.Screen.setCursor(5, 1);

    // moveIntaker(100);
    // // this_thread::sleep_for(100);
    // moveWhileAiming(-30, -40 - 2, 57 + 3 + 2, 11 - 1 - 1);
    // // posForwardRelWithHeading(60, 10, 82);
    // Chassis::getInstance()->chassisBrake(coast);
    // this_thread::sleep_for(1000);
    // turnTo(190);
    // timerForwardWithHeading(-60, 300, 190);
    // Chassis::getInstance()->chassisBrake(coast);
    // this_thread::sleep_for(500);
    // timerForwardWithHeading(-80, 130, 180);
    // Chassis::getInstance()->chassisBrake(coast);
    // moveIntaker(-100);
    // this_thread::sleep_for(200);
    // Chassis::getInstance()->autoSetWheelVel(50, 20);
    // // while (abs(IMUHeading() - 270) > 5)
    // while (abs(IMUHeading() - 240) > 5)
    // {
    //     this_thread::sleep_for(10);
    // }
    // Chassis::getInstance()->autoSetWheelVel(0, 0);
    // turnTo(190 - 5);
    // moveIntaker(0);

    // moveIntaker(100);
    // timerForwardWithHeading(-50, 400, 185 - 10);
    // Chassis::getInstance()->chassisBrake(coast);
    // this_thread::sleep_for(500);
    // turnTo(345 + 0);
    // setPistonLB(true);
    // setPistonF(true);
    // this_thread::sleep_for(500);
    // // timerForwardWithHeading(50, 200, 350);
    // // Chassis::getInstance()->chassisBrake(coast);

    // timerForwardWithHeading(100, 2000 - 500 + 100, 335);
    // Chassis::getInstance()->chassisBrake(coast);
    // this_thread::sleep_for(500);

    // turnTo(0);
    // Chassis::getInstance()->chassisBrake(coast);
    // this_thread::sleep_for(500);

    // // posForwardRelWithHeading(-60, 15, 0);
    // timerForwardWithHeading(-60, 300, 180);
    // Chassis::getInstance()->chassisBrake(coast);
    // this_thread::sleep_for(500);
    // turnTo(180);
    // this_thread::sleep_for(500);
    // moveIntaker(-100);
    // setPistonF(false);
    // setPistonLB(false);
    // // this_thread::sleep_for(100);
    // // posForwardRelWithHeading(-60, 20, 180);
    // timerForwardWithHeading(-60, 300, 180);
    // Chassis::getInstance()->chassisBrake(coast);
    // this_thread::sleep_for(500);
    // setPistonF(false);
    // setPistonLB(false);
    // turnTo(180);
    // this_thread::sleep_for(500);
    // moveIntaker(0);
    // setPistonF(true);
    // setPistonLB(true);

    // timerForwardWithHeading(30, 1600 + 100 + 500, 90);
    // Chassis::getInstance()->chassisBrake(coast);
    // this_thread::sleep_for(500);
    // turnTo(0);
    // this_thread::sleep_for(500);
    // timerForwardWithHeading(30, 500, 0);
    // Chassis::getInstance()->chassisBrake(coast);
    // this_thread::sleep_for(500);
    // turnTo(330);
    // setPistonLB(true);
    // timerForwardWithHeading(30, 700, 330);
    // Chassis::getInstance()->chassisBrake(coast);

    // turnTo(270, 60);
    // Chassis::getInstance()->chassisBrake(coast);
    // this_thread::sleep_for(500);
    // timerForwardWithHeading(80, 500, 260);
    // Chassis::getInstance()->chassisBrake(coast);
    // this_thread::sleep_for(500);

    // timerForwardWithHeading(-30, 200 - 50, 270);
    // Chassis::getInstance()->chassisBrake(coast);
    // this_thread::sleep_for(500);
    // turnTo(180);
    // Chassis::getInstance()->chassisBrake(coast);
    // this_thread::sleep_for(500);

    // timerForwardWithHeading(30, 1000, 185);
    // Chassis::getInstance()->chassisBrake(coast);
    // this_thread::sleep_for(500);
    // turnTo(135);
    // Chassis::getInstance()->chassisBrake(coast);
    // this_thread::sleep_for(500);

    // posForwardRelWithHeading(100, 20, 90);

    // timerForwardWithHeading(100, 300, 180);
    // this_thread::sleep_for(1000);
    // timerForwardWithHeading(-30, 250, 180);
    // Chassis::getInstance()
    //     ->autoSetRobotVel(0, 0);

    // Chassis::getInstance()->setStopBrakeType(hold);

    // while (true) {
    //     this_thread::sleep_for(10);
    // }
}