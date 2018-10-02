
#include "flightplan.h"
#include "filter.h"
#include "ransac.h"
#include "std.h"
#include "stdio.h"
#include "state.h"

struct dronerace_fp_struct dr_fp;

struct jungle_gate_struct jungleGate;
void checkJungleGate(void);
void generate_waypoints_from_gates(void);

int flagHighOrLowGate;

// X, Y, ALT, PSI
/*
#define MAX_GATES 1
const struct dronerace_flightplan_item_struct gates[MAX_GATES] = {
    {0.0, 0.0, 1.5, RadOfDeg(0)},
};
*/



// Note: pprz has positive Z here, while jevois has negative Z
// both_side: bool in Jevois code, 0 or 1 here.
const struct dronerace_flightplan_item_struct gates[MAX_GATES] = {
    //  X-coordinate  Y-coordinate  Z-coordinate  Psi-gate          Type-of-gate  Brake-at-gate   Distance-after gate       both side
    {   3.5,          0.0,          -2.4,          RadOfDeg(0),      REGULAR,      NO_BRAKE,       0.1,                      1},
    {   7.7,          0.0,          -2.4,          RadOfDeg(0),      REGULAR,      BRAKE,          2.5,                      0},
    {   10.0,         2.7,          -2.4,          RadOfDeg(90),     REGULAR,      BRAKE,          1.5,                      0},
    {   7.4,          4.9,          -2.4,          RadOfDeg(180),    REGULAR,      NO_BRAKE,       0.1,                      0},
    {   2.6,          4.9,          -2.4,          RadOfDeg(180),    REGULAR,      BRAKE,          3.0,                      0},
    {   0.0,          3.0,          -2.4,          RadOfDeg(-90),    VIRTUAL,      BRAKE,          0.0,                      0},
    {   1.5,          3.0,          -2.4,          RadOfDeg(0),      JUNGLE,       BRAKE,          2.0,                      0},
    {   5.3,          2.0,          -1.2,          RadOfDeg(-90),    REGULAR,      BRAKE,          2.0,                      0},
    {   3.5,          0.0,          -2.4,          RadOfDeg(180),    REGULAR,      BRAKE,          3.5,                      0}
};

struct dronerace_flightplan_item_struct waypoints_dr[MAX_GATES];

static void update_gate_setpoints(void)
{
  dr_fp.gate_x   = gates[dr_fp.gate_nr].x;
  dr_fp.gate_y   = gates[dr_fp.gate_nr].y;
  dr_fp.gate_z = gates[dr_fp.gate_nr].z;
  dr_fp.gate_psi = gates[dr_fp.gate_nr].psi;
}



void flightplan_reset()
{
  // Current Gate
  dr_fp.gate_nr = 0;
  update_gate_setpoints();

  // Navigation Setpoint
  dr_fp.x_set = 3;
  dr_fp.y_set = 0;
  dr_fp.z_set = 0;
  dr_fp.psi_set = 0;

  resetJungleGate();
  generate_waypoints_from_gates();
}


#define DISTANCE_GATE_NOT_IN_SIGHT  1.5f
#define DISTANCE_ARRIVED_AT_WP    1.0f

void flightplan_run(void)
{
  float dist = 0.0;
  float correctedX, correctedY;
  float dist_2_gate;

  // Get current gate position
  update_gate_setpoints();

  dr_fp.x_set = waypoints_dr[dr_fp.gate_nr].x;
  dr_fp.y_set = waypoints_dr[dr_fp.gate_nr].y;
  dr_fp.z_set = dr_fp.gate_z;

  checkJungleGate();

  // Estimate distance to the gate
  correctedX = dr_state.x+dr_ransac.corr_x;
  correctedY = dr_state.y+dr_ransac.corr_y;
  dist = (waypoints_dr[dr_fp.gate_nr].x - correctedX)*(waypoints_dr[dr_fp.gate_nr].x- correctedX) + (waypoints_dr[dr_fp.gate_nr].y- correctedY)*(waypoints_dr[dr_fp.gate_nr].y - correctedY);

  printf("Gate nr: %d, vision count = %d, nr msm in buffer = %d, (x,y) = (%f, %f), (cx, cy) = (%f, %f), (real_x, real_y) = (%f, %f)\n", dr_fp.gate_nr, dr_vision.cnt, dr_ransac.buf_size, dr_state.x, dr_state.y, correctedX, correctedY, stateGetPositionNed_f()->x, stateGetPositionNed_f()->y);

  // Align with current gate
  dr_fp.psi_set = dr_fp.gate_psi;

  dist_2_gate =  (dr_fp.gate_x - correctedX)*(dr_fp.gate_x - correctedX) + (dr_fp.gate_y - correctedY)*(dr_fp.gate_y - correctedY);

  // If too close to the gate to see the gate, heading to next gate
  if (dist_2_gate < DISTANCE_GATE_NOT_IN_SIGHT * DISTANCE_GATE_NOT_IN_SIGHT)
  {
    if ((dr_fp.gate_nr+1) < MAX_GATES)
    {
      dr_fp.psi_set = gates[dr_fp.gate_nr+1].psi;
    }
  }


  // If close to desired position, switch to next
  if (dist < DISTANCE_ARRIVED_AT_WP * DISTANCE_ARRIVED_AT_WP)
  {
    dr_fp.gate_nr ++;
    if (dr_fp.gate_nr >= MAX_GATES)
    {
      dr_fp.gate_nr = (MAX_GATES -1);
    }

    //printf("\n\n*** RESET DUE TO NEXT GATE ***\n\n");
    // correct the state predictions, refresh the ransac buffer:
    correct_state();
  }
}


#define MAX_TIME_JUNGLE_GATE_DETECTION 1.0
void checkJungleGate()
{
  // get the time when enter jungle gate logic
  if(gates[dr_fp.gate_nr].type == JUNGLE && jungleGate.flagInJungleGate == false)
  {
    jungleGate.flagInJungleGate = 1;
    jungleGate.timeStartJungleGate = dr_state.time; // TODO: this will compile but don't know if it is correct
  }


  // if there is no detection within 1s, it is likely to be a low gate
  // When determine the gate is in high or low position, send controller desired altitude
  if(gates[dr_fp.gate_nr].type == JUNGLE && jungleGate.flagJungleGateDetected == 1)
  {
    // TODO: altitudes in the simulator are positive... is this also the case for the real bebop?
    if(flagHighOrLowGate == UPPER_GATE)
      dr_fp.z_set = -1.6;
    else
      dr_fp.z_set = -0.6;
  }

}


void resetJungleGate()
{
  jungleGate.flagJungleGateDetected = false;
  jungleGate.numJungleGateDetection = 0;
  jungleGate.jungleGateHeight = 0;
  jungleGate.sumJungleGateHeight = 0;
  jungleGate.flagInJungleGate = false;
}


// #define DEBUG_WP_GENERATION
void generate_waypoints_from_gates()
{

#ifdef DEBUG_WP_GENERATION
  if(debug) {

    char filename[128];
    FILE* fp;
    sprintf(filename,"%06d.txt",1111);
    fp = fopen(filename,"w");
    fprintf(fp,"gate_x,gate_y,gate_z,gate_psi,gate_tpye,gate_brake,gate_after_distance\n");
    for (int i=0;i<MAX_GATES;i++)
    {
      fprintf(fp,"%f,%f,%f,%f,%d,%d,%f\n",gates[i].x,
              gates[i].y,
              gates[i].z,
              gates[i].psi,
              gates[i].type,
              gates[i].brake,
              gates[i].distance_after_gate
      );
    }

    fprintf(fp,"\n\n\n");
    fclose(fp);
  }
#endif

  for(int i = 0; i<MAX_GATES;i++)
  {
    float d = gates[i].distance_after_gate;
    if(gates[i].type == VIRTUAL)
    {
      waypoints_dr[i].x = gates[i].x;
      waypoints_dr[i].y = gates[i].y;
    }
    else
    {
      waypoints_dr[i].x = cos(gates[i].psi)*d + gates[i].x;
      waypoints_dr[i].y = sin(gates[i].psi)*d + gates[i].y;
    }
    waypoints_dr[i].z = gates[i].z;
    waypoints_dr[i].psi= gates[i].psi;
    waypoints_dr[i].type= gates[i].type;
    waypoints_dr[i].brake= gates[i].brake;
    waypoints_dr[i].distance_after_gate= gates[i].distance_after_gate;
    waypoints_dr[i].both_side = gates[i].both_side;
  }

#ifdef DEBUG_WP_GENERATION
  fp = fopen(filename,"a");
  fprintf(fp,"wp_x,wp_y,wp_z,wp_psi,wp_tpye,wp_brake,wp_after_distance\n");
  for (int i=0;i<MAX_GATES;i++)
  {
    fprintf(fp,"%f,%f,%f,%f,%d,%d,%f\n",waypoints_dr[i].x,
            waypoints_dr[i].y,
            waypoints_dr[i].z,
            waypoints_dr[i].psi,
            waypoints_dr[i].type,
            waypoints_dr[i].brake,
            waypoints_dr[i].distance_after_gate
    );
  }

  fclose(fp);
#endif

}