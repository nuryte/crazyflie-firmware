/*
Edward Jeffs
Start: May 2 2023

*/

/*
This code is meant to pick the behaviour and control what is revcieved by the a matrix in order to accomplish its goal. 
For now the behavior picks between manual and automatic control.
*/

#include <math.h>
//#include "openmv.h"

#include "param.h"
#include "log.h"
#include "physicalConstants.h"
#include "debug.h"
#include "behavior.h"
#include <stdint.h>
#include <stdlib.h>


#define PI ((float) 3.14159265)
#define gamma ((float) .95)

static behaviorState_t g_self = {

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
  .goalDist = 30,
};



void behaviorReset(behaviorState_t* self)
{
    return;
}

bool behaviorTest(behaviorState_t* self)
{
  return true;
}

void behaviorInit(behaviorState_t* self)
{


  // copy default values (bindings), or does nothing (firmware)
  *self = g_self;

  behaviorReset(self);
}

void behaviorOpenmv(behaviorState_t *self, const setpoint_t *setpoint, behavior_t *behavior) {
    behavior->fx = 0;
    behavior->fy = 0;
    behavior->fz = 0;
    behavior->absz = setpoint->absz;
    behavior->taux = 0;
    behavior->tauy = 0;
    behavior->tauz = 0;
    
    
    int flagauto = self->openmv_state.flag;
    int goal_t = self->openmv_state.t;
    if (flagauto == 1 && goal_t < 2) {
      float goal_x = (float)self->openmv_state.x/128;
      //float goal_y = (float)self->openmv_state.y/128;
      float goal_dist = (float)self->openmv_state.w;
      float goal_side = (float)self->openmv_state.h/128;
      behavior->fx = (float)clamp(g_self.ballpfx * (g_self.goalDist-goal_dist) - 
                    g_self.balldfx * clamp(self->oldGoalDist - self->openmv_state.w, -4, 4) -
                    g_self.ballifx * self->goalDisti,-1,1);
      behavior->taux = (float)g_self.ballptx * goal_side;
      behavior->tauz = goal_x* g_self.ballptz * goal_dist;
    } 
    return;
}

void behaviorManual(const setpoint_t *setpoint, behavior_t *behavior) {

    behavior->fx = setpoint->fx;
    behavior->fy = setpoint->fy;
    behavior->fz = setpoint->fz;
    behavior->absz = setpoint->absz;
    
    behavior->taux = setpoint->taux;
    behavior->tauy = setpoint->tauy;
    behavior->tauz = setpoint->tauz;

    behavior->type_identity = setpoint->type_identity;
}


void behaviorSelect(behaviorState_t* self, const setpoint_t *setpoint,
                                         behavior_t *behavior,
                                         const sensorData_t *sensors,
                                         const state_t *state,
                                         const uint32_t tick)
{

  int flag = setpoint->behavior_flag;
  
  if (tick%100 == 0) {
    self->oldGoalDist = self->openmv_state.w;
    if (self->openmv_state.t == 0) {
      self->goalDisti = clamp(self->goalDisti + clamp(g_self.goalDist - self->openmv_state.w, -5,5), -200, 200);
    }
    omvGetState(&self->openmv_state);
  }


  if (flag == 0) { //manual control
    behaviorManual(setpoint, behavior);

  } else if (flag != 0) { //automatic control
    behaviorOpenmv(self, setpoint, behavior);
    
    
  }

//   if (tick%500 == 0){
//     float goal_x = (float)self->openmv_state.x;
//     float goal_y = (float)self->openmv_state.y;
//     float goal_dist = (float)self->openmv_state.w;
//     float goal_side = (float)self->openmv_state.h;
//     DEBUG_PRINT("(OMV) x%f, y%f, d%f, s%f, t%d\n (relative to ground) z%f, dez%f\n\n", 
//       (double)goal_x, (double)goal_y, (double)goal_dist, (double)goal_side, (int)self->openmv_state.t,
//      (double)(self->setasl - self->groundasl), (double)(self->desiredHeight- self->groundasl));
//   }


}

void behaviorFirmwareInit(void)
{
  behaviorInit(&g_self);
}

bool behaviorFirmwareTest(void)
{
  return behaviorTest(&g_self);
}

void behaviorSelectFirmware(const setpoint_t *setpoint, const behavior_t *behavior,
                                         const sensorData_t *sensors,
                                         const state_t *state,
                                         const uint32_t tick)
{
  behaviorSelect(&g_self, setpoint, behavior, sensors, state, tick);
}


/**
 * Tunning variables for the full state Mellinger Controller
 */



PARAM_GROUP_START(behavior)

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


//PARAM_ADD(PARAM_FLOAT | PARAM_PERSISTENT, i_range_m_z, &g_self.i_range_m_z)
PARAM_GROUP_STOP(behavior)

