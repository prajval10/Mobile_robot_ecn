#include <iostream>
#include <math.h>
#include <robot.h>
#include <sensor.h>
#include <sensor_range.h>

using namespace arpro;
using namespace std;

Environment* Sensor::envir_ = NULL;

Robot::Robot(string _name, double _x, double _y, double _theta)
{    
    pose_.x = _x;
    pose_.y = _y;
    pose_.theta = _theta;

    name_ = _name;

    // init position history
    x_history_.push_back(_x);
    y_history_.push_back(_y);

    // default sampling time: 1/100 s
    dt_ = .01;


}


void Robot::moveXYT(double _vx, double _vy, double _omega)
{
    // update position
    pose_.x += _vx*dt_;
    pose_.y += _vy*dt_;
    pose_.theta += _omega*dt_;

    // store position history
    x_history_.push_back(pose_.x);
    y_history_.push_back(pose_.y);
}

void Robot::initWheels(double b, double r, double wmax)
{
    r_=r;
    b_=b;
    wmax_=wmax;
    if(r_!=0&&b_!=0)
        bool wheels_init_=true;
}

void Robot::rotateWheels(double _left, double _right)
{if(wheels_init_)
    {
        double v= r_*(_left+_right)/2;  // to fill up after defining an initWheel method
        double omega=r_*(_left-_right)/2*b_;
        Robot::moveXYT(v*cos(pose_.theta),v*sin(pose_.theta),omega);
    }
}


// move robot with linear and angular velocities
void Robot::moveVW(double _v, double _omega)
{
    double vx, vy, theta;
    theta = pose_.theta;
    vx = _v*cos(theta);   // to fill up
    vy = _v*sin(theta);
    double w_l,w_r;
    w_l=(_v+b_*_omega)/r_;
    w_r=(_v-b_*_omega)/r_;
    double a_;
    a_=std::max(w_l/wmax_,w_r/wmax_);  //Scale Wheel velocities with max value
    if(a_<1)
        a_=1;
    // Robot::moveXYT(vx,vy,_omega);
    Robot::rotateWheels(w_l/a_,w_r/a_);
}




// try to go to a given x-y position
void Robot::goTo(const Pose &_p)
{
    // error in robot frame
    Pose error = _p.transformInverse(pose_);

    // try to do a straight line with sensor constraints
    moveWithSensor(Twist(error.x, error.y, 0));
}


void Robot::moveWithSensor(Twist _twist)
{

    //Call update method of all sensors:
    for(auto &s: sensors_)
    {
        Pose p= p.transformDirect(s.pose);               // pose_ : Robot-to-world pose
                                                         // s.pose :sensor to robot pose
        s.sensor->update(p);                             // Update each of the sensor to get measurement to the walls
        s.sensor->checkRobotTwist(_twist,p);
    }


    // uses X-Y motion (perfect but impossible in practice)
    //    moveXYT(_twist.vx, _twist.vy,_twist.w);

    //Get velocity and omega from twist
    double alpha=20;
    double _v=0,_omega=0;
    _v+=(_twist.vx);
    _omega+=alpha*(_twist.vy)+_twist.w;
    Robot::moveVW(_v,_omega);   // to fill up, use V-W motion when defined
}


void Robot::printPosition()
{
    cout << "Current position: " << pose_.x << ", " << pose_.y << endl;
}

