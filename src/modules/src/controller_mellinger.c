/*
The MIT License (MIT)

Copyright (c) 2018 Wolfgang Hoenig and James Alan Preiss

Permission is hereby granted, free of charge, to any person obtaining a copy
of this software and associated documentation files (the "Software"), to deal
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

/*
This controller is based on the following publication:

Daniel Mellinger, Vijay Kumar:
Minimum snap trajectory generation and control for quadrotors.
IEEE International Conference on Robotics and Automation (ICRA), 2011.

We added the following:
 * Integral terms (compensates for: battery voltage drop over time, unbalanced center of mass due to asymmmetries, and uneven wear on propellers and motors)
 * D-term for angular velocity
 * Support to use this controller as an attitude-only controller for manual flight
*/

#include <math.h>
//#include "openmv.h"

#include "param.h"
#include "log.h"
#include "position_controller.h"
#include "controller_mellinger.h"
#include "physicalConstants.h"
#include "debug.h"
#include <stdint.h>
#include <stdlib.h>


#define PI ((float) 3.14159265)
#define gamma ((float) .95)

// Global state variable used in the
// firmware as the only instance and in bindings
// to hold the default values
static controllerMellinger_t g_self = {
  .mass = CF_MASS,
  .massThrust = 132000,

  // XY Position PID
  .kp_xy = 0.4,       // P
  .kd_xy = 0.2,       // D
  .ki_xy = 0.05,      // I
  .i_range_xy = 2.0,
  

  // Z Position
  .kp_z = .8,       // P
  .kd_z = 0.4,        // D
  .ki_z = 0.05,       // I
  .i_range_z  = 0.4,

  // Attitude
  .kR_xy = 0, // P
  .kw_xy = .01, // D
  .ki_m_xy = 0.0, // I
  .i_range_m_xy = 1.0,

  // Yaw
  .kR_z = 1, // P
  .kw_z = .04, // D
  .ki_m_z = 500, // I
  .i_range_m_z  = 1500,

  // roll and pitch angular velocity
  .kd_omega_rp = 200, // D

  .lx = 1,
  .ly = 1,


  // Helper variables
  .i_error_x = 0,
  .i_error_y = 0,
  .i_error_z = 0,

  .i_error_m_x = 0,
  .i_error_m_y = 0,
  .i_error_m_z = 0,

  //ball
  .ballpfx = .1,
  .balldfx = 1,
  .ballifx = 1,
  .ballptx = 1,
  .balldtx = 1,
  .ballpfz = 1,
  .balldfz = 1,
  .ballptz = 1,
  .balldtz = 1,
  .goalHeight = 0,
  .goalDist = 0
};



void controllerMellingerReset(controllerMellinger_t* self)
{
  self->i_error_x = 0;
  self->i_error_y = 0;
  self->i_error_z = 0;
  self->i_error_m_x = 0;
  self->i_error_m_y = 0;
  self->i_error_m_z = 0;
  self->pitch = 0;
  self->roll = 0;
  
  self->bicoptermode = 0;
  self->groundasl = -1;
  self->setasl = -1;
  self->baseYaw = 0;
  self->desiredHeight = -1;
  self->absRoll = 0;
  self->absPitch = 0;
  self->oldGoalDist = 0;
  self->goalDisti = 0;

}

void controllerMellingerInit(controllerMellinger_t* self)
{


  // copy default values (bindings), or does nothing (firmware)
  *self = g_self;

  controllerMellingerReset(self);
}

bool controllerMellingerTest(controllerMellinger_t* self)
{
  return true;
}

void controllerMellinger(controllerMellinger_t* self, control_t *control, const setpoint_t *setpoint,
                                         const sensorData_t *sensors,
                                         const state_t *state,
                                         const uint32_t tick)
{
  //sensor suite setup
  if (!RATE_DO_EXECUTE(ATTITUDE_RATE, tick)) {
      return;
  }
  if (setpoint->bicopter.mode != 2){
    return;
  }
  float height = state->position.z;
  float dt = (float)(1.0f/ATTITUDE_RATE);

  if (self->bicoptermode == 0){// initialization
    self->bicoptermode = 1;
    self->groundasl = height; 
    self->yawrate = radians(sensors->gyro.z);
    self->rollrate = radians(sensors->gyro.x);
    self->pitchrate = radians(sensors->gyro.y);
    self->setasl = height; 
    self->desiredHeight = height;
    self->absRoll = radians(state->attitude.roll);
    self->absPitch = radians(state->attitude.pitch);
    self->zrate = state->velocity.z;
    self->pitch = radians(state->attitude.pitch);
    self->roll = radians(state->attitude.roll);
    self->i_error_x = 0;
    self->i_error_y = 0;
  }

  self->setasl = self->setasl * gamma + height * (1-gamma);
  self->zrate = self->zrate * gamma + state->velocity.z * (1-gamma);
  self->i_error_x = clamp(self->i_error_x + sensors->acc.x, -20,20);
  self->i_error_y = clamp(self->i_error_y + sensors->acc.y, -20,20);
  self->pitch = self->pitch * gamma + radians(state->attitude.pitch) * (1-gamma);
  self->roll = self->roll * gamma + radians(state->attitude.roll)* (1-gamma);

  self->yawrate = self->yawrate * gamma + radians(sensors->gyro.z) * (1-gamma);
  self->rollrate = self->rollrate * gamma + radians(sensors->gyro.x) * (1-gamma);
  self->pitchrate = self->pitchrate * gamma + radians(sensors->gyro.y) * (1-gamma);
  //float dt = (float)(1.0f/ATTITUDE_RATE);
  //choosing desired movement
  float flag = setpoint->sausage.flag;
  float wfx = 0;
  float wfy = 0;
  float wfz = 0;
  float wtx = 0;
  float wty = 0;
  float wtz = 0;
  if (tick%100 == 0) {
    self->oldGoalDist = self->openmv_state.w;
    if (self->openmv_state.t == 0) {
      self->goalDisti = clamp(self->goalDisti + clamp(g_self.goalDist - self->openmv_state.w, -5,5), -200, 200);
    }
    omvGetState(&self->openmv_state);
  }


  if (flag == 0) {

    if (setpoint->sausage.goZ == 1) {
      self->goalHeight = self->desiredHeight;
    }
    else if (setpoint->sausage.goZ == 2) {
      self->desiredHeight = self->goalHeight;
    }
    wfx = setpoint->sausage.fx;
    wfy = setpoint->sausage.fy;
    self->desiredHeight += setpoint->sausage.absz*dt;
    wfz = self->desiredHeight;
    wtx = setpoint->sausage.taux;
    wty = setpoint->sausage.tauy;
    wtz = setpoint->sausage.tauz;
  } else if (flag == 1) { 

    self->desiredHeight = self->groundasl + self->goalHeight;//+= ((float)g_self.ballpfz * (goal_y))*dt;
    wfz = self->desiredHeight;
    int flagauto = self->openmv_state.flag;
    int goal_t = self->openmv_state.t;
    if (flagauto == 1 && goal_t < 2) {
      float goal_x = (float)self->openmv_state.x/128;
      //float goal_y = (float)self->openmv_state.y/128;
      float goal_dist = (float)self->openmv_state.w;
      float goal_side = (float)self->openmv_state.h/128;
      wfx = (float)clamp(g_self.ballpfx * (g_self.goalDist-goal_dist) - 
                    g_self.balldfx * clamp(self->oldGoalDist - self->openmv_state.w, -4, 4) -
                    g_self.ballifx * self->goalDisti,-1,1);
      wtx = (float)g_self.ballptx * goal_side;
      wtz = goal_x* g_self.ballptz * goal_dist;
      
    } 
    else if (flag == 0){
      float horizontal = (float)self->openmv_state.x/128 - 1/2;
      //float vertical = (float)current_state.y/128 - 1/2;
      float size = self->openmv_state.w;//math.max(current_state.w,current_state.h);
      if (size !=0) {
        //self->desiredHeight += (float)vertical/128  - 1/2;
        if (size < 100 && abs(horizontal) < (float).22){
          wfx = (float) g_self.ballpfx;
        }
        else {
          wfx = 0;
        }
        wtz = horizontal* g_self.ballptz /(size);
        if (wtz > (float).2){
            wtz = (float).2;
        }
        wfy = 0;
        wfz = self->desiredHeight;
        wtx = 0;
        wty = 0;
      }
    }
    else {
      wfx = 0;
      wfy = 0;
      wfz = self->desiredHeight;
      wtx = 0;
      wty = 0;
      wtz = 0;
    }
    
  }
  if (tick%500 == 0){
    float goal_x = (float)self->openmv_state.x;
    float goal_y = (float)self->openmv_state.y;
    float goal_dist = (float)self->openmv_state.w;
    float goal_side = (float)self->openmv_state.h;
    DEBUG_PRINT("(OMV) x%f, y%f, d%f, s%f, t%d\n (relative to ground) z%f, dez%f\n\n", 
      (double)goal_x, (double)goal_y, (double)goal_dist, (double)goal_side, (int)self->openmv_state.t,
     (double)(self->setasl - self->groundasl), (double)(self->desiredHeight- self->groundasl));
  }

  float tempz = clamp((wfz  - self->setasl)*(float)g_self.kp_z- (self->zrate) * (float)g_self.kd_z,-2,2);
  if (wtz == 0 ){
    self->i_error_m_z = clamp(self->i_error_m_z + self->yawrate, -200,200);
  } 
  float desiredYawrate = wtz * g_self.kR_z + self->yawrate*g_self.kw_z + self->i_error_m_z*g_self.ki_m_z;
  float cosr = (float) cos(-self->roll);
  float sinr = (float) sin(-self->roll);
  float cosp = (float) cos(self->pitch);
  float sinp = (float) sin(self->pitch);
  float tfx = wfx*cosp + wfy*sinr*sinp - tempz*cosr*sinp;
  float tfy = wfy*cosr + tempz*sinr;
  float tfz = wfx*sinp - wfy*sinr*cosp + tempz*cosr*cosp;
  /**
  float tfx = wfx*(cosr- sinr)/2 + 
              wfy*sinp*sinr/2 +
              tempz*(cosr-cosp*sinr)/2;
  float tfy = wfy*cosp/2 + tempz*sinp/2;
  float tfz = -1* wfy*cosr*sinp/2+ 
              wfx * (cosr+ sinr)/2+
              tempz*(cosp*cosr+sinr)/2;*/
  if (g_self.i_range_z != 0) {
    tfx = wfx;
    tfy = wfy;
    tfz = tempz;
  }
  
  int id = setpoint->sausage.id;
  
  // choose type of flight
  if (id == 0) {
    float desiredRoll = wtx - self->roll*(float)g_self.kR_xy - self->rollrate *(float)g_self.kw_xy;
    //float desiredPitch = wty - self->pitch*(float)g_self.kR_xy - self->pitchrate *(float)g_self.kw_xy;
    cosp = (float) cos(-self->pitch);
    sinp = (float) sin(-self->pitch);
    cosr = (float) cos(self->roll);
    // sinr = (float) sin(self->roll);
    wfx = wfx + g_self.ki_xy * self->i_error_x;
    float l = g_self.lx; //.3
    float tfx = wfx*cosp + tempz*sinp;
    //float tfy = fy*cosp/2 + tempz*sinp/2;
    float tfz = (wfx*sinp + tempz * cosp)/cosr;
    float fx = clamp(tfx, -1 , 1);//setpoint->bicopter.fx;
    float fz = clamp(tfz, -.5 , 1.5);//setpoint->bicopter.fz;
    float taux = clamp(desiredRoll, -l + (float)0.01 , l - (float) 0.01);
    float tauz = clamp(desiredYawrate, -.3 , .3);// limit should be .25 setpoint->bicopter.tauz; //- stateAttitudeRateYaw

    float term1 = l*l*fx*fx + l*l*fz*fz + taux*taux + tauz*tauz;
    float term2 = 2*fz*l*taux - 2*fx*l*tauz;
    float term3 = sqrt(term1+term2);
    float term4 = sqrt(term1-term2);

    float f1 = term3/(2*l); // in unknown units
    float f2 = term4/(2*l);
    float t1 = PI/2;
    float t2 = PI/2;
    if ((f1 != 0) || (f2 != 0)) {
      t1 = atan2((fz*l - taux)/term3, (fx*l + tauz)/term3 );// in radians
      t2 = atan2((fz*l + taux)/term4, (fx*l - tauz)/term4 );
    }
  
    while (t1 < -PI/4) {
      t1 = t1 + 2 * PI;
    }
    while (t1 > 2*PI-PI/4) {
      t1 = t1 - 2 * PI;
    }
    while (t2 < -PI/4) {
      t2 = t2 + 2 * PI;
    }
    while (t2 > 2*PI-PI/4) {
      t2 = t2 - 2 * PI;
    }
    
    control->bicopter.s1 = 1 - clamp(t1, 0, PI*3/2)/(PI*3/2);// cant handle values between PI and 2PI
    control->bicopter.s2 = clamp(t2, 0, PI*3/2)/(PI*3/2);
    control->bicopter.m1 = clamp(f1, 0, 1);
    control->bicopter.m2 = clamp(f2, 0, 1);
    
    if (tick % 10000 == 0) {
    DEBUG_PRINT("(mel)%f, %f, %f, %f \n(ssmm)%f, %f, %f, %f\n(atitude)r%f, p%f, x%f, y%f\n(mode)%d, %d\n(mv)%f\n\n", 
          (double)fx, (double)fz, (double)taux, (double)tauz,
           (double)t1, (double)t2, (double)f1, (double)f2,
           (double)self->roll, (double)self->pitch, (double)state->velocity.x,(double)state->velocity.y,
           (int)flag, (int)id,
           (double) self->openmv_state.w);
    }
  }
    else if (id == 7){    
    self->bicoptermode = 0;
    self->desiredHeight = self->groundasl; 
    float f1 = 0; 
    float f2 = 0;
    float t1 = PI/2;
    float t2 = PI/2;
    control->bicopter.s1 = 1 - clamp(t1, 0, PI*3/2)/(PI*3/2);// cant handle values between PI and 2PI
    control->bicopter.s2 = clamp(t2, 0, PI*3/2)/(PI*3/2);
    control->bicopter.m1 = clamp(f1, 0, 1);
    control->bicopter.m2 = clamp(f2, 0, 1);

    }
   else {
    self->absRoll = clamp(self->absRoll + wtx*dt, -PI/2, PI/2);
    self->absPitch = clamp(self->absPitch + wty*dt, -PI/2, PI/2);
    float desiredRoll = (self->absRoll + self->roll)*(float)g_self.kR_xy - self->rollrate *(float)g_self.kw_xy;
    float desiredPitch = (self->absPitch - self->pitch)*(float)g_self.kR_xy - self->pitchrate *(float)g_self.kw_xy;
    float fx = clamp(tfx, -3.5, 3.5);
    float fy = clamp(tfy, -3.5, 3.5);
    float fz = clamp(tfz,-1,4);
    float tx = clamp(desiredRoll,-1, 1); //tau x
    float ty = clamp(desiredPitch,-1, 1); //tau y
    float tz = clamp(desiredYawrate,-1, 1); //tau z

    float f1 = 0;
    float f2 = 0;
    float t1 = 0;
    float t2 = 0;
    if (id == 1){// f1, f2, t1, t2
      float lx = .6;
      float ly = 1;
      float term1 = lx*lx*ly*ly;
      float term2 = fz*lx*ly + lx*tx;
      float term3 = lx*lx*(float)pow(-fx*ly + tz, 2);
      f1 = (float)sqrt(((float)pow(term2 - ly*ty, 2) + term3)/term1)/4;
      f2 = (float)sqrt(((float)pow(term2 + ly*ty, 2) + term3)/term1)/4;

      t1 = atan2(fz/4 + tx/(4*ly)-ty/(4*lx), fx/4-tz/(4*ly));
      t2 = atan2(fz/4 + tx/(4*ly)+ty/(4*lx), fx/4-tz/(4*ly));

      while (t1 < -PI/4) {
        t1 = t1 + 2 * PI;
      }
      while (t1 > 2*PI-PI/4) {
        t1 = t1 - 2 * PI;
      }
      while (t2 < -PI/4) {
        t2 = t2 + 2 * PI;
      }
      while (t2 > 2*PI-PI/4) {
        t2 = t2 - 2 * PI;
      }
      
      control->bicopter.s1 = clamp(t1, 0, PI*3/2)/(PI*3/2);// cant handle values between PI and 2PI
      control->bicopter.s2 = clamp(t2, 0, PI*3/2)/(PI*3/2);
      
    } else if (id == 2){// f3, f4, t3, t4
      float lx = .6;
      float ly = 1;
      float term1 = lx*lx*ly*ly;
      float term2 = - fz*lx*ly + lx*tx;
      float term3 = lx*lx*(float)pow(fx*ly + tz, 2);
      
      f1 = (float)sqrt(((float)pow(term2 + ly*ty, 2) + term3)/term1)/4;
      f2 = (float)sqrt(((float)pow(-term2 + ly*ty, 2) + term3)/term1)/4;

    
      t1 = atan2(fz/4 - tx/(4*ly)-ty/(4*lx), fx/4+tz/(4*ly));
      t2 = atan2(fz/4 - tx/(4*ly)+ty/(4*lx), fx/4+tz/(4*ly));

      while (t1 < -PI/4) {
        t1 = t1 + 2 * PI;
      }
      while (t1 > 2*PI-PI/4) {
        t1 = t1 - 2 * PI;
      }
      while (t2 < -PI/4) {
        t2 = t2 + 2 * PI;
      }
      while (t2 > 2*PI-PI/4) {
        t2 = t2 - 2 * PI;
      }
      control->bicopter.s1 = clamp(t1, 0, PI*3/2)/(PI*3/2);// cant handle values between PI and 2PI
      control->bicopter.s2 = clamp(t2, 0, PI*3/2)/(PI*3/2);

    } else if (id == 3) {
      float lx = g_self.lx;
      float ly = g_self.ly;
      float lx2ly2 = lx*lx+ly*ly;
      
      f2 = (float)sqrt((float)pow(fz-(2*ty/lx), 2) + 4* (float)pow(fy+(lx*tz)/lx2ly2,2))/4;
      f1 = (float)sqrt((float)pow(fz+(2*tx/ly), 2) + 4* (float)pow(fx-(ly*tz)/lx2ly2,2))/4;

    
      t2 = atan2((fz- (2*ty)/lx)/4, (fy+(lx*tz)/lx2ly2));
      t1 = atan2((fz+ (2*tx)/ly)/4, (fx-(ly*tz)/lx2ly2));

      while (t1 < -PI/4) {
        t1 = t1 + 2 * PI;
      }
      while (t1 > 2*PI-PI/4) {
        t1 = t1 - 2 * PI;
      }
      while (t2 < -PI/4) {
        t2 = t2 + 2 * PI;
      }
      while (t2 > 2*PI-PI/4) {
        t2 = t2 - 2 * PI;
      }
      control->bicopter.s1 = 1- clamp(t1, 0, PI*3/2)/(PI*3/2);// cant handle values between PI and 2PI
      control->bicopter.s2 =  clamp(t2, 0, PI*3/2)/(PI*3/2);
    } else if (id == 4) {
      float lx = g_self.lx;
      float ly = g_self.ly;
      float lx2ly2 = lx*lx+ly*ly;
      
      f2 = (float)sqrt((float)pow(fz+(2*ty/lx), 2) + 4* (float)pow(fy-(lx*tz)/lx2ly2,2))/4;
      f1 = (float)sqrt((float)pow(fz-(2*tx/ly), 2) + 4* (float)pow(fx+(ly*tz)/lx2ly2,2))/4;

    
      t2 = atan2((fz+ (2*ty)/lx)/4, (fy-(lx*tz)/lx2ly2));
      t1 = atan2((fz- (2*tx)/ly)/4, (fx+(ly*tz)/lx2ly2));

      while (t1 < -PI/4) {
        t1 = t1 + 2 * PI;
      }
      while (t1 > 2*PI-PI/4) {
        t1 = t1 - 2 * PI;
      }
      while (t2 < -PI/4) {
        t2 = t2 + 2 * PI;
      }
      while (t2 > 2*PI-PI/4) {
        t2 = t2 - 2 * PI;
      }

      control->bicopter.s1 =  clamp(t1, 0, PI*3/2)/(PI*3/2);// cant handle values between PI and 2PI
      control->bicopter.s2 = 1 - clamp(t2, 0, PI*3/2)/(PI*3/2);
    } else if (id == 8) {
      //float lx = g_self.lx;
      //float ly = g_self.ly;
      //float lxy = floatsqrt(lx*lx+ly*ly);
      
      f2 = 0;//(float) sqrt(pow(fz + tx/ly - ty/lx,2)+ pow(-sqrt(2)* fx*lxy + sqrt(2)*fy*lxy + tz,2)/(lxy*lxy))/4;
      f1 = 0;

    
      t2 = 0;//atan2((fz+ (2*ty)/lx)/4, (fy-(lx*tz)/lxy));
      t1 = 0;//atan2((fz- (2*tx)/ly)/4, (fx+(ly*tz)/lxy));

      while (t1 < -PI/4) {
        t1 = t1 + 2 * PI;
      }
      while (t1 > 2*PI-PI/4) {
        t1 = t1 - 2 * PI;
      }
      while (t2 < -PI/4) {
        t2 = t2 + 2 * PI;
      }
      while (t2 > 2*PI-PI/4) {
        t2 = t2 - 2 * PI;
      }

      control->bicopter.s1 =  clamp(t1, 0, PI*3/2)/(PI*3/2);// cant handle values between PI and 2PI
      control->bicopter.s2 = 1 - clamp(t2, 0, PI*3/2)/(PI*3/2);
    }else if (id == 9) {
      float lx = g_self.lx;
      float ly = g_self.ly;
      float lx2ly2 = lx*lx+ly*ly;
      
      f2 = (float)sqrt((float)pow(fz+(2*ty/lx), 2) + 4* (float)pow(fy-(lx*tz)/lx2ly2,2))/4;
      f1 = (float)sqrt((float)pow(fz-(2*tx/ly), 2) + 4* (float)pow(fx+(ly*tz)/lx2ly2,2))/4;

    
      t2 = atan2((fz+ (2*ty)/lx)/4, (fy-(lx*tz)/lx2ly2));
      t1 = atan2((fz- (2*tx)/ly)/4, (fx+(ly*tz)/lx2ly2));

      while (t1 < -PI/4) {
        t1 = t1 + 2 * PI;
      }
      while (t1 > 2*PI-PI/4) {
        t1 = t1 - 2 * PI;
      }
      while (t2 < -PI/4) {
        t2 = t2 + 2 * PI;
      }
      while (t2 > 2*PI-PI/4) {
        t2 = t2 - 2 * PI;
      }

      control->bicopter.s1 =  clamp(t1, 0, PI*3/2)/(PI*3/2);// cant handle values between PI and 2PI
      control->bicopter.s2 = 1 - clamp(t2, 0, PI*3/2)/(PI*3/2);
    }else if (id == 5){
      self->bicoptermode = 0;
      self->desiredHeight = self->groundasl; 
      t1 = PI/2;
      t2 = PI/2;
      control->bicopter.s1 = 1-  clamp(t1, 0, PI*3/2)/(PI*3/2);// cant handle values between PI and 2PI
      control->bicopter.s2 =  clamp(t2, 0, PI*3/2)/(PI*3/2);
      f1 = 0;
      f2 = 0;
    } else if (id == 6){
      self->bicoptermode = 0;
      self->desiredHeight = self->groundasl; 
      t1 = PI/2;
      t2 = PI/2;
      control->bicopter.s1 =  clamp(t1, 0, PI*3/2)/(PI*3/2);// cant handle values between PI and 2PI
      control->bicopter.s2 = 1 -clamp(t2, 0, PI*3/2)/(PI*3/2);
      f1 = 0;
      f2 = 0;
    } else {
      return;
    }
    control->bicopter.m1 = clamp(f1, 0, 1);
    control->bicopter.m2 = clamp(f2, 0, 1);
    if (tick % 10000 == 0) {
    DEBUG_PRINT("(input)%f, %f, %f, %f, %f \n(t1,t2,f1,f2)%f, %f, %f, %f\n(desireyaw,pitch,roll)%f, %f, %f\n(desiredheight,height,ground)%f, %f, %f\n\n", 
          (double)fx, (double)fz, (double)tx, (double)ty, (double)tz,
           (double)t1, (double)t2, (double)f1, (double)f2,
           (double)desiredYawrate, (double)self->pitch, (double)self->roll,
           (double)self->desiredHeight, (double)self->setasl, (double)self->groundasl);
    }
  }



}


void controllerMellingerFirmwareInit(void)
{
  controllerMellingerInit(&g_self);
}

bool controllerMellingerFirmwareTest(void)
{
  return controllerMellingerTest(&g_self);
}

void controllerMellingerFirmware(control_t *control, const setpoint_t *setpoint,
                                         const sensorData_t *sensors,
                                         const state_t *state,
                                         const uint32_t tick)
{
  controllerMellinger(&g_self, control, setpoint, sensors, state, tick);
}


/**
 * Tunning variables for the full state Mellinger Controller
 */



PARAM_GROUP_START(ctrlMel)


/**
 * @brief goal height set point
 */
PARAM_ADD_CORE(PARAM_FLOAT | PARAM_PERSISTENT, ballifx, &g_self.ballifx)

/**
 * @brief goal height set point
 */
PARAM_ADD_CORE(PARAM_FLOAT | PARAM_PERSISTENT, goalDist, &g_self.goalDist)
/**
 * @brief goal height set point
 */
PARAM_ADD_CORE(PARAM_FLOAT | PARAM_PERSISTENT, goalHeight, &g_self.goalHeight)
/**
 * @brief moment arm on lx
 */
PARAM_ADD_CORE(PARAM_FLOAT | PARAM_PERSISTENT, lx, &g_self.lx)
/**
 * @brief moment arm on ly
 */
PARAM_ADD_CORE(PARAM_FLOAT | PARAM_PERSISTENT, ly, &g_self.ly)

/**
 * @brief ball fx P-gain (horizontal xy plane)
 */
PARAM_ADD_CORE(PARAM_FLOAT | PARAM_PERSISTENT, ballpfx, &g_self.ballpfx)
/**
 * @brief ball fx d-gain (horizontal xy plane)
 */
PARAM_ADD_CORE(PARAM_FLOAT | PARAM_PERSISTENT, balldfx, &g_self.balldfx)

/**
 * @brief ball fx P-gain (horizontal xy plane)
 */
PARAM_ADD_CORE(PARAM_FLOAT | PARAM_PERSISTENT, ballptx, &g_self.ballptx)
/**
 * @brief ball fx d-gain (horizontal xy plane)
 */
PARAM_ADD_CORE(PARAM_FLOAT | PARAM_PERSISTENT, balldtx, &g_self.balldtx)

/**
 * @brief ball fx P-gain (horizontal xy plane)
 */
PARAM_ADD_CORE(PARAM_FLOAT | PARAM_PERSISTENT, ballpfz, &g_self.ballpfz)
/**
 * @brief ball tz P-gain (horizontal xy plane)
 */
PARAM_ADD_CORE(PARAM_FLOAT | PARAM_PERSISTENT, balldfz, &g_self.balldfz)

/**
 * @brief ball fx P-gain (horizontal xy plane)
 */
PARAM_ADD_CORE(PARAM_FLOAT | PARAM_PERSISTENT, ballptz, &g_self.ballptz)
/**
 * @brief ball tz P-gain (horizontal xy plane)
 */
PARAM_ADD_CORE(PARAM_FLOAT | PARAM_PERSISTENT, balldtz, &g_self.balldtz)

/**
 * @brief Position P-gain (horizontal xy plane)
 */
PARAM_ADD_CORE(PARAM_FLOAT | PARAM_PERSISTENT, kp_xy, &g_self.kp_xy)
/**
 * @brief Position D-gain (horizontal xy plane)
 */
PARAM_ADD_CORE(PARAM_FLOAT | PARAM_PERSISTENT, kd_xy, &g_self.kd_xy)
/**
 * @brief Position I-gain (horizontal xy plane)
 */
PARAM_ADD_CORE(PARAM_FLOAT | PARAM_PERSISTENT, ki_xy, &g_self.ki_xy)
/**
 * @brief Attitude maximum accumulated error (roll and pitch)
 */
PARAM_ADD(PARAM_FLOAT | PARAM_PERSISTENT, i_range_xy, &g_self.i_range_xy)
/**
 * @brief Position P-gain (vertical z plane)
 */
PARAM_ADD_CORE(PARAM_FLOAT | PARAM_PERSISTENT, kp_z, &g_self.kp_z)
/**
 * @brief Position D-gain (vertical z plane)
 */
PARAM_ADD_CORE(PARAM_FLOAT | PARAM_PERSISTENT, kd_z, &g_self.kd_z)
/**
 * @brief Position I-gain (vertical z plane)
 */
PARAM_ADD_CORE(PARAM_FLOAT | PARAM_PERSISTENT, ki_z, &g_self.ki_z)
/**
 * @brief Position maximum accumulated error (vertical z plane)
 */
PARAM_ADD(PARAM_FLOAT | PARAM_PERSISTENT, i_range_z, &g_self.i_range_z)
/**
 * @brief total mass [kg]
 */
PARAM_ADD_CORE(PARAM_FLOAT | PARAM_PERSISTENT, mass, &g_self.mass)
/**
 * @brief Force to PWM stretch factor
 */
PARAM_ADD_CORE(PARAM_FLOAT | PARAM_PERSISTENT, massThrust, &g_self.massThrust)
/**
 * @brief Attitude P-gain (roll and pitch)
 */
PARAM_ADD_CORE(PARAM_FLOAT | PARAM_PERSISTENT, kR_xy, &g_self.kR_xy)
/**
 * @brief Attitude P-gain (yaw)
 */
PARAM_ADD_CORE(PARAM_FLOAT | PARAM_PERSISTENT, kR_z, &g_self.kR_z)
/**
 * @brief Attitude D-gain (roll and pitch)
 */
PARAM_ADD_CORE(PARAM_FLOAT | PARAM_PERSISTENT, kw_xy, &g_self.kw_xy)
/**
 * @brief Attitude D-gain (yaw)
 */
PARAM_ADD_CORE(PARAM_FLOAT | PARAM_PERSISTENT, kw_z, &g_self.kw_z)
/**
 * @brief Attitude I-gain (roll and pitch)
 */
PARAM_ADD_CORE(PARAM_FLOAT | PARAM_PERSISTENT, ki_m_xy, &g_self.ki_m_xy)
/**
 * @brief Attitude I-gain (yaw)
 */
PARAM_ADD_CORE(PARAM_FLOAT | PARAM_PERSISTENT, ki_m_z, &g_self.ki_m_z)
/**
 * @brief Angular velocity D-Gain (roll and pitch)
 */
PARAM_ADD_CORE(PARAM_FLOAT | PARAM_PERSISTENT, kd_omega_rp, &g_self.kd_omega_rp)
/**
 * @brief Attitude maximum accumulated error (roll and pitch)
 */
PARAM_ADD(PARAM_FLOAT | PARAM_PERSISTENT, i_range_m_xy, &g_self.i_range_m_xy)
/**
 * @brief Attitude maximum accumulated error (yaw)
 */
PARAM_ADD(PARAM_FLOAT | PARAM_PERSISTENT, i_range_m_z, &g_self.i_range_m_z)
PARAM_GROUP_STOP(ctrlMel)

/**
 * Logging variables for the command and reference signals for the
 * Mellinger controller
 */
LOG_GROUP_START(ctrlMel)
LOG_ADD(LOG_FLOAT, cmd_thrust, &g_self.cmd_thrust)
LOG_ADD(LOG_FLOAT, cmd_roll, &g_self.cmd_roll)
LOG_ADD(LOG_FLOAT, cmd_pitch, &g_self.cmd_pitch)
LOG_ADD(LOG_FLOAT, cmd_yaw, &g_self.cmd_yaw)
LOG_ADD(LOG_FLOAT, r_roll, &g_self.r_roll)
LOG_ADD(LOG_FLOAT, r_pitch, &g_self.r_pitch)
LOG_ADD(LOG_FLOAT, r_yaw, &g_self.r_yaw)
LOG_ADD(LOG_FLOAT, accelz, &g_self.accelz)
LOG_ADD(LOG_FLOAT, zdx, &g_self.z_axis_desired.x)
LOG_ADD(LOG_FLOAT, zdy, &g_self.z_axis_desired.y)
LOG_ADD(LOG_FLOAT, zdz, &g_self.z_axis_desired.z)
LOG_ADD(LOG_FLOAT, i_err_x, &g_self.i_error_x)
LOG_ADD(LOG_FLOAT, i_err_y, &g_self.i_error_y)
LOG_ADD(LOG_FLOAT, i_err_z, &g_self.i_error_z)
LOG_GROUP_STOP(ctrlMel)
