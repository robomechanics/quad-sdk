/**
MIT License (modified)

Copyright (c) 2018 Ghost Robotics
Authors:
Avik De <avik@ghostrobotics.io>
Tom Jacobs <tom.jacobs@ghostrobotics.io>

Permission is hereby granted, free of charge, to any person obtaining a copy
of this **file** (the "Software"), to deal
in the Software without restriction, including without limitation the rights
to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
copies of the Software, and to permit persons to whom the Software is
furnished to do so, subject to the following conditions:

The above copyright notice and this permission notice shall be included in all
copies or substantial portions of the Software.

THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
SOFTWARE.
*/
// #include <Behavior.h>
// #include <Motor.h>
#include <SDK.h>
// #include <math.h>
#include <stdio.h>
gr::Robot robot;
enum FHMode
{
  FH_STAND = 0,
  FH_CROUCH,
  FH_LEAP,
  FH_LAND
};

bool isAd(int i)
{
  if (i>=8)
    return true;
  else
    return false;
}
bool isHip(int i)
{
  if (i==0 || i==2 || i==4 || i==6)
    return true;
  else
    return false;
}
bool isKnee(int i)
{
  if (i==1 || i==3 || i==5 || i==7)
    return true;
  else
    return false;
}
bool isFront(int i)
{
  if (i==0 || i==1 || i==4 || i==5 || i==8 || i==10)
    return true;
  else
    return false;
}

class FirstHop : public Behavior
{
public:

  FHMode mode = FH_STAND; //Current state within state-machine
  uint32_t tLast; //int used to store system time at various events
  uint32_t tLastUpdate;

  void begin()
  {
    mode = FH_STAND;
    tLast = S->millis;
  }

  void end() {}

  void update()
  {

    // tLastUpdate = S->millis;
    tLastUpdate = robot.S.millis;

    for (int j = 0; j < robot.P.joints_count; ++j)
    {
      C->mode = RobotCommand_Mode_JOINT;
      joint[j].setOpenLoopMode(JointMode_PWM);

      joint[j].setGain(0.5,0.02);
      if (isAd(j) == true)
      {
        joint[j].setPosition(0.0);
      } 

      if (mode == FH_STAND)
      {              
          float hip_pos = 0.5;
          float knee_pos = 1.0;
          if (isHip(j) == true)
            joint[j].setPosition(hip_pos);
          else if (isKnee(j) == true)
            joint[j].setPosition(knee_pos);

        if ((S->millis - tLast) >= 40000)
        {
          mode = FH_CROUCH;
          tLast = S->millis;
        }
      }
      else if (mode == FH_CROUCH)
      {
        // if (isHip(j) == true)
        // {
        //   if (isFront(j) == true)
        //   {
        //     joint[j].setPosition(0.2);
        //   } else {
        //     joint[j].setPosition(-0.2);
        //   }
        // }
        // else if (isKnee(j) == true)
        // {
        //   joint[j].setPosition(1.0);
        // }

        float hip_pos = -0.2;
        float knee_pos = 1.0;
        if (isHip(j) == true)
          joint[j].setPosition(hip_pos);
        else if (isKnee(j) == true)
          joint[j].setPosition(knee_pos);


        if ((S->millis - tLast) >= 2000)
        {
          mode = FH_LEAP;
          tLast = S->millis;
        }
      }
      else if (mode == FH_LEAP)
      {
        if (isHip(j) == true)
        {
          // joint[j].setOpenLoopMode(JointMode_CURRENT);
          joint[j].setOpenLoop(1.0);
        }
        else if (isKnee(j) == true)
        {
          joint[j].setOpenLoop(1.0);
        }

        float ext = joint[j].getPosition();
        if ((ext >= 2.7 && isKnee(j)) || (S->millis - tLast) >= 600)
        // if (ext >= 1.5 && isKnee(j))
        {
          mode = FH_STAND;
          tLast = S->millis;
        }
      }
      else if (mode == FH_LAND)
      {
        if ((S->millis - tLast) >= 2000)
        {
          mode = FH_STAND;
          tLast = S->millis;
        }
      }
    }


    // // All joints in current control mode
    // for (int j = 0; j < P->joints_count; ++j)
    //   joint[j].setOpenLoopMode(JointMode_CURRENT);

    // // Limb control mode
    // C->mode = RobotCommand_Mode_LIMB;
    
    // // Stop the robot if no MAVLink message has been received in one second
    // uint32_t time_msec = clockTimeUS() / 1000;
    // if(time_msec > last_mavlink_time_msec + 1000)
    // {
    //   // No moving for you, robot
    //   C->mode = RobotCommand_Mode_JOINT;
    //   for (int j = 0; j < P->joints_count; ++j)
    //     C->joints[j].mode = JointMode_OFF;
    // }
    
    // // For all limbs
    // for (int i = 0; i < 4; ++i)
    // {
    //   // For all joints on this limb
    //   for (int j = 0; j < 3; ++j)
    //   {
    //     // Set gains from MAVLink packet
    //     limb[i].setGain(j, limbCmd[i].Kp[j], limbCmd[i].Kd[j]);
        
    //     // Set positions from MAVLink packet
    //     limb[i].setPosition(j, limbCmd[i].pos[j]);
    //   }
    // }
    
    // // Send back data to computer
    // float userData[10];
    // userData[0] = (float) clockTimeUS();  // Send time in microseconds
    // userData[1] = limbCmd[0].pos[2];      // Echo back commanded toe z position
    // userData[2] = (C->mode == RobotCommand_Mode_LIMB) ? 1 : 0; // Report limbs active/inactive state
    // mavlinkUserTxPack(userData); // Pointer to 10 floats or 40 bytes of anything
  }
};

FirstHop first_hop;

void debug() {
  // Check test boolean
  printf("%d, tLast = %d, tLastUpdate = %d, mode = %d\t", S->millis - first_hop.tLast,first_hop.tLast,first_hop.tLastUpdate, first_hop.mode);

  // Joints
  for (uint8_t i = 0; i < P->joints_count; ++i) {
    printf("%.2f\t", joint[i].getPosition());
  }
  // // IMU
  // printf("imu %.2f\t%.2f\t%.2f\t", S->imu.euler.x, S->imu.euler.y, S->imu.euler.z);

  printf("\n");
}

int main(int argc, char *argv[]) {
  RobotParams_Type robot =
#ifdef ROBOT_SPIRIT
      RobotParams_Type_SPIRIT;
#else
      RobotParams_Type_NGR;
#endif
#ifdef STARTUP_DELAY
  STARTUP_DELAY_MS = STARTUP_DELAY;
#endif
#ifdef ROBOT_VERSION
  ROBOT_VERSION_OVERRIDE = ROBOT_VERSION;
#endif
  init(robot, argc, argv);

  // Hardware configuration
  JoyType joyType = JoyType_FRSKY_XSR;
  ioctl(JOYSTICK_FILENO, IOCTL_CMD_JOYSTICK_SET_TYPE, &joyType);

  // Disable built-in soft CROUCH up
  P->behaviorConfig.softStart = false;

  // Remove default behaviors from behaviors vector, create, add, and start ours
  behaviors.clear();
  behaviors.push_back(&first_hop);
  first_hop.begin();

  setDebugRate(4);

  return begin();
}
