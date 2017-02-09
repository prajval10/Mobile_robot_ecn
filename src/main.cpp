#include <iostream>
#include <math.h>
#include <cmath>

#include <robot.h>
#include <envir.h>
#include <sensor_range.h>
#include <sensor_bearing.h>

using namespace std;
using namespace arpro;

int main()
{
    // default environment with moving target
    Environment envir;
    // sensors gets measurements from this environment
    Sensor::setEnvironment(envir);

    // init robot at (0,0,0)
    Robot robot("R2D2", 0,0,0);
    envir.addRobot(robot);
    robot.initWheels(0.3,0.07,10);
    RangeSensor _range_sensor(robot,0.1,0.0,0.0); //Range Sensor placed at (0.1,0,0) in the robot frame

    // init robot at (0,0,0)
    Robot robot2("Robot2", 2,3,-M_PI);
    envir.addRobot(robot2);    //
    robot2.initWheels(0.3,0.05,10); // Initialising the wheel base, radius and maximum allowed velocity
    BearingSensor _bearing_sensor(robot2,0.1,0.0,0.0);  //Bearing sensor placed at (0.1,0,0) in the robot2 frame


    for(unsigned int i=0;i<10000;++i)
    {
        cout << "---------------------" << endl;

        // update target position
        envir.updateTarget();        

        //R2D2 will try to follow target
        robot.goTo(envir.target);

        //Robot2 moves with bearing sensor, it is given a horizontal velocity of 0.4m/s and twist with its sensor output
        robot2.moveWithSensor(Twist(0.4,0,0));
    }

    // plot trajectory
    envir.plot();
    
}
