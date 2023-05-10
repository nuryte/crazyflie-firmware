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
  .ballptx = 1,
  .balldtx = 1,
  .ballpfz = 1,
  .balldfz = 1,
  .ballptz = 1,
  .balldtz = 1,

  .ir = 1,
  .ip = 1,
  .rpswap = 1,
  
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

void initializeSensorReadings(controllerMellinger_t* self,
                                         const sensorData_t *sensors,
                                         const state_t *state) 
{
  float height = state->position.z;
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
}

void updateSensorReadings(controllerMellinger_t* self,
                                         const sensorData_t *sensors,
                                         const state_t *state) 
{

  self->setasl = self->setasl * gamma + state->position.z * (1-gamma);
  self->zrate = self->zrate * gamma + state->velocity.z * (1-gamma);
  self->pitch = self->pitch * gamma + radians(state->attitude.pitch) * (1-gamma);
  self->roll = self->roll * gamma + radians(state->attitude.roll)* (1-gamma);

  self->yawrate = self->yawrate * gamma + radians(sensors->gyro.z) * (1-gamma);
  self->rollrate = self->rollrate * gamma + radians(sensors->gyro.x) * (1-gamma);
  self->pitchrate = self->pitchrate * gamma + radians(sensors->gyro.y) * (1-gamma);
}

void bicopterControl(controllerMellinger_t* self, control_t *control, 
                                         behavior_t *behavior,const uint32_t tick)
{
  if (behavior->type_identity%10 == 1){

    float tempz = (behavior->fz  - self->setasl)*(float)g_self.kp_z- (self->zrate) * (float)g_self.kd_z + behavior->absz;
    float desiredYawrate = behavior->tauz * g_self.kR_z + self->yawrate*g_self.kw_z;
    float desiredRoll = behavior->taux - self->roll*(float)g_self.kR_xy - self->rollrate *(float)g_self.kw_xy;
    //float desiredPitch = wty - self->pitch*(float)g_self.kR_xy - self->pitchrate *(float)g_self.kw_xy;
    float cosp = (float) cos(-self->pitch);
    float sinp = (float) sin(-self->pitch);
    float cosr = (float) cos(self->roll);
    // sinr = (float) sin(self->roll);
    float l = g_self.lx; //.3
    float tfx = behavior->fx*cosp + tempz*sinp;
    //float tfy = fy*cosp/2 + tempz*sinp/2;
    float tfz = (behavior->fx*sinp + tempz * cosp)/cosr;
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
    
    
  } else {
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
  // if (tick % 10000 == 0) {
  //   DEBUG_PRINT("(mel)%f, %f, %f, %f \n(ssmm)%f, %f, %f, %f\n(atitude)r%f, p%f, x%f, y%f\n(mv)%f\n\n", 
  //         (double)fx, (double)fz, (double)taux, (double)tauz,
  //          (double)t1, (double)t2, (double)f1, (double)f2,
  //          (double)self->roll, (double)self->pitch, (double)state->velocity.x,(double)state->velocity.y,
  //          (double) self->openmv_state.w);
  //   }
}

void biQuadCopterControl(controllerMellinger_t* self, control_t *control, 
                                         behavior_t *behavior,const uint32_t tick) 
{
    
  
  float fx = behavior->fx;
  //float fy = behavior->fy;
  float fz = behavior->fz;
  
  float tx = behavior->taux;
  float ty = behavior->tauy;
  float tz = behavior->tauz;
  int id = behavior->type_identity;

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

  }
}

void quadSausageControl(controllerMellinger_t* self, control_t *control, 
                                         behavior_t *behavior,const uint32_t tick)
{

  float fx = behavior->fx;
  float fy = behavior->fy;
  float fz = behavior->fz;
  
  float tx = behavior->taux;
  float ty = behavior->tauy;
  float tz = behavior->tauz;
  int id = behavior->type_identity;

  float f1 = 0;
  float f2 = 0;
  float t1 = PI/2;
  float t2 = PI/2;
  if (id == 3) {
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
    } else if (id == 5){
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
    }
}

void skynetControl(controllerMellinger_t* self, control_t *control, 
                                         behavior_t *behavior,const uint32_t tick)
{
    float lx = g_self.lx;
    float ly = g_self.ly;
    float lx2ly2 = lx*lx+ly*ly;

    float fx = behavior->fx;
    float fy = behavior->fy;
    float fz = behavior->fz;
    
    float tx = behavior->taux;
    float ty = behavior->tauy;
    float tz = behavior->tauz;
    int id = behavior->type_identity;

    float f1 = 0;
    float f2 = 0;
    float t1 = PI/2;
    

    if (id == 41){
      f1 = (float)sqrt((float)pow(fz-(2*ty/lx), 2) + 4* (float)pow(fy+(lx*tz)/lx2ly2,2))/4;
      t1 = atan2((fz- (2*ty)/lx)/4, (fy+(lx*tz)/lx2ly2));
      control->bicopter.s1 = 1 - clamp(t1, 0, PI*3/2)/(PI*3/2);// cant handle values between PI and 2PI
      control->bicopter.s2 = 1 - clamp(t1, 0, PI*3/2)/(PI*3/2);// cant handle values between PI and 2PI
      
    }
    if (id == 42){
      f1 = (float)sqrt((float)pow(fz+(2*tx/ly), 2) + 4* (float)pow(fx-(ly*tz)/lx2ly2,2))/4;
      t1 = atan2((fz+ (2*tx)/ly)/4, (fx-(ly*tz)/lx2ly2));
      control->bicopter.s1 = clamp(t1, 0, PI*3/2)/(PI*3/2);// cant handle values between PI and 2PI
      control->bicopter.s2 = clamp(t1, 0, PI*3/2)/(PI*3/2);// cant handle values between PI and 2PI
      
    }
    if (id == 43){
      f1 = (float)sqrt((float)pow(fz+(2*ty/lx), 2) + 4* (float)pow(fy-(lx*tz)/lx2ly2,2))/4;
      t1 = atan2((fz+ (2*ty)/lx)/4, (fy-(lx*tz)/lx2ly2));
      control->bicopter.s1 =  clamp(t1, 0, PI*3/2)/(PI*3/2);// cant handle values between PI and 2PI
      control->bicopter.s2 =  clamp(t1, 0, PI*3/2)/(PI*3/2);// cant handle values between PI and 2PI
      
    }
    if (id == 44){
      f1 = (float)sqrt((float)pow(fz-(2*tx/ly), 2) + 4* (float)pow(fx+(ly*tz)/lx2ly2,2))/4;
      t1 = atan2((fz- (2*tx)/ly)/4, (fx+(ly*tz)/lx2ly2));
      control->bicopter.s1 = 1 - clamp(t1, 0, PI*3/2)/(PI*3/2);// cant handle values between PI and 2PI
      control->bicopter.s2 = 1 - clamp(t1, 0, PI*3/2)/(PI*3/2);// cant handle values between PI and 2PI
      
    }
    if (id == 45){
      self->bicoptermode = 0;
      self->desiredHeight = self->groundasl; 
      f1 = 0;
      t1 = PI/2;
      control->bicopter.s1 = 1 - clamp(t1, 0, PI*3/2)/(PI*3/2);
      control->bicopter.s2 = 1 - clamp(t1, 0, PI*3/2)/(PI*3/2);
      
    }
    if (id == 46){
      self->bicoptermode = 0;
      self->desiredHeight = self->groundasl; 
      f1 = 0;
      t1 = PI/2;
      control->bicopter.s1 = clamp(t1, 0, PI*3/2)/(PI*3/2);
      control->bicopter.s2 = clamp(t1, 0, PI*3/2)/(PI*3/2);
      
    }
    if (id == 47){
      self->bicoptermode = 0;
      self->desiredHeight = self->groundasl; 
      f1 = 0;
      t1 = PI/2;
      control->bicopter.s1 = clamp(t1, 0, PI*3/2)/(PI*3/2);
      control->bicopter.s2 = clamp(t1, 0, PI*3/2)/(PI*3/2);
      
    }
    if (id == 48){
      self->bicoptermode = 0;
      self->desiredHeight = self->groundasl; 
      f1 = 0;
      t1 = PI/2;
      control->bicopter.s1 = 1 - clamp(t1, 0, PI*3/2)/(PI*3/2);
      control->bicopter.s2 = 1 - clamp(t1, 0, PI*3/2)/(PI*3/2);
      
    }
    f2 = f1;
    control->bicopter.m1 = clamp(f1, 0, 1);
    control->bicopter.m2 = clamp(f2, 0, 1);
    return;

}

void fourFliesControl(controllerMellinger_t* self, control_t *control, 
                                         behavior_t *behavior,const uint32_t tick) {
  float wfx = behavior->fx;
  float wfy = behavior->fy;
  float wfz = behavior->fz;
  float absz = behavior->absz;
  float wtx = behavior->taux;
  float wty = behavior->tauy;
  float wtz = behavior->tauz;
  
  float tempz = (wfz - self->setasl)*(float)g_self.kp_z- (self->zrate) * (float)g_self.kd_z + absz;
  float desiredYawrate = wtz * g_self.kR_z + self->yawrate*g_self.kw_z;
  
  float cosr = (float) cos(g_self.ir*self->roll);
  float sinr = (float) sin(g_self.ir*self->roll);
  float cosp = (float) cos(g_self.ip*self->pitch);
  float sinp = (float) sin(g_self.ip*self->pitch);

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
  if (g_self.ki_xy == 0) {
    tfx = wfx;
    tfy = wfy;
    tfz = tempz;
  }
  self->absRoll = wtx; //clamp(self->absRoll + wtx*dt, -PI/2, PI/2);
  self->absPitch = wty; //clamp(self->absPitch + wty*dt, -PI/2, PI/2);
  float r = g_self.ir*self->roll;
  float rr = g_self.ir*self->rollrate;
  float p = g_self.ip*self->pitch;
  float pp = g_self.ip*self->pitchrate;
  
  float desiredRoll = (self->absRoll) + (r)*(float)g_self.kR_xy + rr *(float)g_self.kw_xy;
  float desiredPitch = (self->absPitch) + (p)*(float)g_self.kR_xy + pp *(float)g_self.kw_xy;

  behavior->fx = clamp(tfx, -3.5, 3.5);
  behavior->fy = clamp(tfy, -3.5, 3.5);
  behavior->fz = clamp(tfz,-1,4);
  behavior->taux = clamp(desiredRoll,-1, 1); //tau x
  behavior->tauy = clamp(desiredPitch,-1, 1); //tau y
  behavior->tauz = clamp(desiredYawrate,-1, 1); //tau z

}

void controllerMellinger(controllerMellinger_t* self, control_t *control, 
                                         behavior_t *behavior,
                                         const sensorData_t *sensors,
                                         const state_t *state,
                                         const uint32_t tick)
{
  //sensor suite setup
  if (!RATE_DO_EXECUTE(ATTITUDE_RATE, tick)) {
      return;
  }
  if (behavior->type_identity == 0) {
    return;
  }
  //float height = state->position.z;
  //float dt = (float)(1.0f/ATTITUDE_RATE);

  if (self->bicoptermode == 0){// initialization
    self->bicoptermode = 1;
    initializeSensorReadings(self, sensors, state);
  } else {
    updateSensorReadings(self, sensors, state);
  }
  //float dt = (float)(1.0f/ATTITUDE_RATE);
  //choosing desired movement
  //int id = behavior->type_identity%10 ;
  int type = (int)behavior->type_identity/10;

  
  
  
  // choose type of flight
  if (type == 1) {
    bicopterControl(self, control, behavior, tick);

  } else if (type == 2){ 
    fourFliesControl(self, control, behavior, tick);
    biQuadCopterControl(self, control, behavior, tick);

  } else if (type == 3){ 
    fourFliesControl(self,control, behavior, tick);
    quadSausageControl(self, control, behavior, tick);

  } else if (type == 4){ 
    fourFliesControl(self, control, behavior, tick);
    skynetControl(self, control, behavior, tick);

  } else {
    control->bicopter.m1 = 0;
    control->bicopter.m2 = 0;
    control->bicopter.s1 = 0;
    control->bicopter.s2 = 0;
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

void controllerMellingerFirmware(control_t *control, behavior_t *behavior,
                                         const sensorData_t *sensors,
                                         const state_t *state,
                                         const uint32_t tick)
{
  controllerMellinger(&g_self, control, behavior, sensors, state, tick);
}


/**
 * Tunning variables for the full state Mellinger Controller
 */



PARAM_GROUP_START(ctrlMel)

/**
 * @brief moment arm on lx
 */
PARAM_ADD_CORE(PARAM_FLOAT | PARAM_PERSISTENT, lx, &g_self.lx)
/**
 * @brief moment arm on ly
 */
PARAM_ADD_CORE(PARAM_FLOAT | PARAM_PERSISTENT, ly, &g_self.ly)

/**
 * @brief moment arm on lx
 */
PARAM_ADD_CORE(PARAM_FLOAT | PARAM_PERSISTENT, ir, &g_self.ir)
/**
 * @brief moment arm on ly
 */
PARAM_ADD_CORE(PARAM_FLOAT | PARAM_PERSISTENT, ip, &g_self.ip)
/**
 * @brief moment arm on ly
 */
PARAM_ADD_CORE(PARAM_FLOAT | PARAM_PERSISTENT, rpswap, &g_self.rpswap)

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
