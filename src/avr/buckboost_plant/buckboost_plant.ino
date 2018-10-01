#include <SPI.h>
#include <mcp_can.h>
#include <mcp_can_dfs.h>


#include "arduino_buckboost.h"
#include "arduino_buckboost_private.h"
#include "rtwtypes.h"
#include "MW_ArduinoHWInit.h"
#include "arduinoAVRScheduler.h"


/* Named constants for Chart: '<Root>/DC-to-DC Converter' */
#define arduino_buck_IN_NO_ACTIVE_CHILD ((uint8_T)0U)
#define arduino_buckbo_IN_Discontinuous ((uint8_T)3U)
#define arduino_buckboos_IN_Discharging ((uint8_T)2U)
#define arduino_buckboost_IN_Charging  ((uint8_T)1U)


  /* 
   *  Project:      LANSCE Experiment 2018
   *  Author:       Chris Shum (University of Waterloo, Real-time Embedded Software Group)
   *  Description:  Cruise control application, plant module (controller/plant implementation)
   *  Updated:      2018-09-20
   */
  
  /* System Configuration
   *  SERIAL_BAUD : Serial connection baud rate (e.g. 115200)
   *  SERIAL_DISPLAY : Serial monitor output enabled/disabled [true|false]
   *  CAN_ENABLE : CAN communications enabled/disabled [true|false]
   */
  #define SERIAL_BAUD 115200        // Define Serial Monitor baud rate [115200]
  #define SERIAL_DISPLAY true       // Enable Serial Monitor output [true|false]
  #define CAN_ENABLE false           // Enable CAN traffic [true|false]
  #define SPI_CS_PIN 9              // Define SPI CS Pin (Default: 9)
  #define TIMEOUT_THRESH 5          // Loops to wait before stepping plant
  #define SIMULATION_DURATION 15    // Simulation stop time [s]
  #define SPEED_INITIAL 0           // Plant initial speed, km/h

  MCP_CAN CAN(SPI_CS_PIN);          // Set CS pin
  
  unsigned char payload[8];         // CAN frame data payload
  const char payload_zero[8] = {0,0,0,0,0,0,0,0};  // used to clear CAN frame data payload  
 
  /* Plant Parameters
   *  Assumptions:
   *  - Longitudinal dynamics
   *  - Particle mass
   *  - Neglect drivetrain losses
   *  - Neglect slip (perfect grip)
   *  - Only forces acting on vehicle are wheel torque, aerodynamic drag
   */

   /* Vehicle Parameters
    *  2017 Toyota Corolla
    *  m = 2860 / 2.25; % kg
    *  Cd = 0.33;
    *  width = 69.9 * 25.4/1000; % mm
    *  height = 57.3 * 25.4/1000; % mm
    *  CdA = Cd * width * height;
    */
   static float veh_mass_kg = 2860/2.25;                           // Vehicle mass [kg]
   static float veh_drag_coeff = 0.33;                             // Vehicle aerodynamic drag coefficient [N/m^2.s^2]
   static float veh_height = 57.3 * 25.4/1000;                     // Vehicle height [m]
   static float veh_width = 69.9 * 25.4/1000;                      // Vehicle width [m]
   static float veh_drag_xA = veh_width*veh_height;                // Effective front cross-sectional area [m^2]
   static float CdA = veh_drag_coeff * veh_drag_xA;
   static float veh_whl_radius = 0.315;                            // Vehicle tire radius [m]
   static float gear_ratio[6] = {3.54, 1.91, 1.31, 0.97, 0.71, 0.62};  // Transmission gear ratios
   static float fd_ratio = 4.21;                                   // Final drive gear ratio
   static float eng_trq_max = 174;                                 // Maximum engine torque [Nm] (ignore speed rating)
   static float veh_force_max = eng_trq_max*gear_ratio[3]*fd_ratio/veh_whl_radius; // assume 4th gear   

  /* Simulation Global Variables
   *  Ego vehicle state (position, velocity)
   *  Lead vehicle state (position, velocity)
   *  Moments (driving 
   */
   static float r_w=veh_whl_radius;// Radius, wheel [m]
   static float F_ego; 
   static float D_ego;
   static float F_net;
   static float m = veh_mass_kg;
   static float x_ego, v_ego;      // Distance (x-axis) for ego [m], Velocity (x-axis for ego [m/s]
   static float x_lead, v_lead;    // Distance (x-axis) for lead [m], Velocity (x-axis for lead [m/s] 
//   float M_d, M_b, M_r;     // Moments, Driving force [Nm], Braking force [Nm], Rolling Resistance [Nm] (neglect RR)
//   float R_air, R_c;        // Resistance, aerodynamic drag [N]

  /* Simulation Parameters
   *  - Lead vehicle behaviour
   *  - Initial conditions
   */
   static bool simulation_stop = false;             // Simulation STOP flag - stops loop()
   static float simulation_time;                    // Simulation clock [s]
   static float simulation_timestep = 0.010;        // Simulation timestep [s]
   static float simulation_duration = SIMULATION_DURATION;       // Simulation stop time [s]
//   static float sch_cycle_lead[] = {0.1};           //{0,1,2,3,4,5,6,7,8,9,10,9,8,7,6,5,4,3,2,1,0};   // Lead vehicle drive schedule [m/s]


volatile int IsrOverrun = 0;
static boolean_T OverrunFlag = 0;
volatile boolean_T stopRequested = false;


/* Continuous states */
X_arduino_buckboost_T arduino_buckboost_X;

/* Block signals and states (default storage) */
DW_arduino_buckboost_T arduino_buckboost_DW;

/* Real-time model */
RT_MODEL_arduino_buckboost_T arduino_buckboost_M_;
RT_MODEL_arduino_buckboost_T *const arduino_buckboost_M = &arduino_buckboost_M_;



void setup() {
  pinMode(8, OUTPUT);   // initialize the LED pin as an output:
  pinMode(9, OUTPUT);
  
  pinMode(11, OUTPUT);   // initialize the LED pin as an output:
  pinMode(10, OUTPUT);

  digitalWrite(10, HIGH);
  digitalWrite(11, HIGH);

  
  if (SERIAL_DISPLAY) {
    //Initialize serial and wait for port to open:
    Serial.begin(SERIAL_BAUD);
    while (!Serial) {;} // wait for serial port to connect. Needed for native USB
    // Print simulation parameters
    Serial.println("Serial Connected, Hello World!");    
  }
  if (CAN_ENABLE) {
    // Initialize CAN board
    while (CAN_OK != CAN.begin(CAN_500KBPS)) {              // init can bus : baudrate = 500k
      Serial.println("CAN BUS Shield init fail");
      Serial.println("Init CAN BUS Shield again");
      delay(100);
    }
    Serial.println("CAN BUS Shield init ok!");
  }

  MW_Arduino_Init();
}


bool LOOP_FOREVER = true;
void loop() {
  while (1) {
    digitalWrite(8, LOW);
    digitalWrite(9, LOW);
    digitalWrite(10, HIGH);
    digitalWrite(11, HIGH);
    delay(20);
    digitalWrite(8, LOW);
    digitalWrite(9, LOW);
    digitalWrite(10, LOW);
    digitalWrite(11, LOW);
  }
  
  Serial.println("IN LOOP");

  volatile boolean_T runModel = true;
  float modelBaseRate = 2.0E-6;
  float systemClock = 0;
  init();
  MW_Arduino_Init();
  rtmSetErrorStatus(arduino_buckboost_M, 0);
  arduino_buckboost_initialize();
  configureArduinoAVRTimer();
  runModel =
    (rtmGetErrorStatus(arduino_buckboost_M) == (NULL)) && !rtmGetStopRequested
    (arduino_buckboost_M);

#ifndef _MW_ARDUINO_LOOP_

  sei();

#endif

  Serial.println("AFTER MODEL STARTUP");

  sei ();
  while (runModel) {
    stopRequested = !(
                      (rtmGetErrorStatus(arduino_buckboost_M) == (NULL)) &&
                      !rtmGetStopRequested(arduino_buckboost_M));
    runModel = !(stopRequested);
    runModel = runModel && MW_Arduino_Loop();

      Serial.println("RUNNING MODEL LOOP");
  }

  /* Disable rt_OneStep() here */

  /* Terminate model */
  arduino_buckboost_terminate();
  cli();
//  return 0;









  
  while (LOOP_FOREVER) {
    // CAN Communications
    unsigned char len = 0;
    unsigned char buf[8];
    unsigned int canID;
  
    bool RECEIVED_PID_PARAM = false;
    
    if (CAN_ENABLE) {
      // Sync Controller (RESET command sent via CAN)
      // CAN Tx 0x010 - CTRL_RESET
      memcpy(payload, payload_zero, sizeof(payload)); // initialize data payload with zeroes
      payload[0] = 0xFF; 
      CAN.sendMsgBuf(0x100, 0, 8, payload);
      delay(10);  
    }
  
    while (!RECEIVED_PID_PARAM) {
      if (CAN_ENABLE) {
        while (!(CAN_MSGAVAIL == CAN.checkReceive())) {
          ; // wait for CAN Rx
        }
        CAN.readMsgBuf(&len, buf);
        canID = CAN.getCanId();
         
        // CAN Rx 0x230 - PID Control Gains
        if (canID==0x230){
          RECEIVED_PID_PARAM = true;
        }
      }
    }
    
    // Initialize simulation (configuration, state variables)
    simulation_time = 0;
    x_ego, v_ego = 0;
    F_ego, D_ego, F_net = 0;  
  
    // Rolling start
    v_ego = SPEED_INITIAL/3.6;
    
    if (SERIAL_DISPLAY) {
      Serial.println("---");
      Serial.println("!!! Simulation Starting");
      Serial.println("---");
    }
    bool ALLOW_STEP_PLANT = true;
    while ((simulation_time <= simulation_duration) && (!simulation_stop)) {
      while (!ALLOW_STEP_PLANT) {
      // Program Flow Control Flags
        bool TRANSMITTED_EGO_SPEED = false;
        bool RECEIVED_TORQUE_COMMAND = false;
        bool RECEIVED_SPEED_ECHO = false;
  
       
  
        if (CAN_ENABLE) {
          // CAN Tx 0x0130 - Timestamp
          if (SERIAL_DISPLAY) {Serial.println(simulation_time);}
          memcpy(payload, payload_zero, sizeof(payload)); // initialize data payload with zeroes
          payload[0] = (long) (simulation_time/simulation_timestep) >> (8*3) & 0xFF;
          payload[1] = (long) (simulation_time/simulation_timestep) >> (8*2) & 0xFF;
          payload[2] = (long) (simulation_time/simulation_timestep) >> (8*1) & 0xFF;
          payload[3] = (long) (simulation_time/simulation_timestep) >> (8*0) & 0xFF;
          CAN.sendMsgBuf(0x130, 0, 8, payload);
    
          // CAN Tx 0x132 - Ego Vehicle Speed [m/s]
          memcpy(payload, payload_zero, sizeof(payload)); // initialize data payload with zeroes
          payload[0] = (long) (v_ego * 100) >> (8*3) & 0xFF;
          payload[1] = (long) (v_ego * 100) >> (8*2) & 0xFF;
          payload[2] = (long) (v_ego * 100) >> (8*1) & 0xFF;
          payload[3] = (long) (v_ego * 100) >> (8*0) & 0xFF;
          CAN.sendMsgBuf(0x132, 0, 8, payload);
          
          TRANSMITTED_EGO_SPEED = true;
        }  
  
        int CAN_TIMEOUT=0;
        while (!RECEIVED_TORQUE_COMMAND) {
          if (CAN_ENABLE) {
            while (!(CAN_MSGAVAIL == CAN.checkReceive()) & simulation_time>simulation_timestep & !(CAN_TIMEOUT>TIMEOUT_THRESH) ) {
  //            if (SERIAL_DISPLAY) {Serial.println("Waiting for CAN Message from CONTROLLER...");}
                CAN_TIMEOUT++;
                if (CAN_TIMEOUT>TIMEOUT_THRESH) {
                  RECEIVED_TORQUE_COMMAND = true; // hold last on timeout
                }
              ; // wait for CAN Rx
            }
            CAN.readMsgBuf(&len, buf);
            canID = CAN.getCanId();
            
            // CAN Tx 0x210 - ACC Torque Command [Nm]
            if (canID==0x210){
              memcpy(payload,buf,sizeof(buf));
  //            F_ego = (long) ((payload[0] << (8*3)) + (payload[1] << (8*2)) + (payload[2] << (8*1)) + (payload[3] << (8*0))) / 100;
  //            Serial.print("Received F_ego: ");Serial.println(F_ego);
  //            if (F_ego > veh_force_max) { 
  //              F_ego = veh_force_max; // Saturate controller output at maximum physical limit
  //            }
              Serial.print("Post-limit F_ego: ");Serial.println(F_ego);
              if (true) {          
                for (int i = 0; i<len; i++) {    // print the data
                    Serial.print(buf[i], HEX);
                    Serial.print("\t");
                }     
                Serial.print("\n");
                long F_cmd[4];
                for (int i = 0; i<4; i++) {    // print the data
                    Serial.print((long) buf[i] << (8*(3-i)));
                    Serial.print("\t");
                }     
                Serial.println(" ");
                for (int i = 0; i<4; i++) {    // print the data
                    F_cmd[i] = (long) buf[i] << (8*(3-i));
                    Serial.print(F_cmd[i]);  
                    Serial.print("\t");
                }     
                long F_cmd_total = 0;
                F_cmd_total = F_cmd[0] + F_cmd[1] + F_cmd[2] + F_cmd[3];
                F_cmd_total = F_cmd_total / 100;
                Serial.print("F_cmd: ");
                Serial.println(F_cmd_total);
  
  //              F_ego = ((long) (buf[0] << (8*3)) + (long) (buf[1] << (8*2)) + (long) (buf[2] << (8*1)) + (long) (buf[3] << (8*0))) / 100;  // Figure out why this is broken
                F_ego = F_cmd_total;
  //              if (F_ego<0) {
  //                F_ego=0;
  //              }
                Serial.print("Received F_ego: ");Serial.println(F_ego);
              }
              RECEIVED_TORQUE_COMMAND = true;
            }
          }
        }
        
        /* --- Plant Loop ---
         * Calculate forces on ego vehicle
         * Sum of wheel force (F_w), drag force (F_d)
         */
        if (SERIAL_DISPLAY) {     
          Serial.print("v_ego (pre-force): ");
          Serial.println(v_ego);
        }
        
        x_ego += v_ego * simulation_timestep; // Ego vehicle motion
        D_ego = CdA * pow(v_ego,2);// * (v_ego/abs(v_ego)); // Aerodynamic drag force, ego vehicle
        if (v_ego > 0) {
          F_net = F_ego - D_ego;
        } else {
          F_net = F_ego + D_ego;
        }
          
        if (SERIAL_DISPLAY) {
          Serial.print("m: ");Serial.println(m);
          Serial.print("CdA: ");Serial.println(CdA);
          Serial.print("F_ego: ");Serial.println(F_ego);
          Serial.print("D_ego: ");Serial.println(D_ego);
          Serial.print("F_net: ");Serial.println(F_net);
        }
        v_ego += ((F_net)/m * simulation_timestep);
        
        if (SERIAL_DISPLAY) {    
          Serial.print("v_ego (post-force): ");
          Serial.println(v_ego);
        }
         
        if (RECEIVED_TORQUE_COMMAND && TRANSMITTED_EGO_SPEED) {
          ALLOW_STEP_PLANT = true;
          simulation_time += simulation_timestep;        
        }
      }
      ALLOW_STEP_PLANT = false;    
      Serial.println("--- STEP COMPLETE ---");
      delay(10);
    }
    LOOP_FOREVER = false;
  }  
}




















void rt_OneStep(void)
{
  /* Check for overrun. Protect OverrunFlag against preemption */
  if (OverrunFlag++) {
    IsrOverrun = 1;
    OverrunFlag--;
    return;
  }

//#ifndef _MW_ARDUINO_LOOP_

  sei();

//#endif

  arduino_buckboost_step();

  /* Get model outputs here */
//#ifndef _MW_ARDUINO_LOOP_

  cli();

//#endif

  OverrunFlag--;
}








/*
 * This function updates continuous states using the ODE3 fixed-step
 * solver algorithm
 */
static void rt_ertODEUpdateContinuousStates(RTWSolverInfo *si )
{
  /* Solver Matrices */
  static const real_T rt_ODE3_A[3] = {
    1.0/2.0, 3.0/4.0, 1.0
  };

  static const real_T rt_ODE3_B[3][3] = {
    { 1.0/2.0, 0.0, 0.0 },

    { 0.0, 3.0/4.0, 0.0 },

    { 2.0/9.0, 1.0/3.0, 4.0/9.0 }
  };

  time_T t = rtsiGetT(si);
  time_T tnew = rtsiGetSolverStopTime(si);
  time_T h = rtsiGetStepSize(si);
  real_T *x = rtsiGetContStates(si);
  ODE3_IntgData *id = (ODE3_IntgData *)rtsiGetSolverData(si);
  real_T *y = id->y;
  real_T *f0 = id->f[0];
  real_T *f1 = id->f[1];
  real_T *f2 = id->f[2];
  real_T hB[3];
  int_T i;
  int_T nXc = 3;
  rtsiSetSimTimeStep(si,MINOR_TIME_STEP);

  /* Save the state values at time t in y, we'll use x as ynew. */
  (void) memcpy(y, x,
                (uint_T)nXc*sizeof(real_T));

  /* Assumes that rtsiSetT and ModelOutputs are up-to-date */
  /* f0 = f(t,y) */
  rtsiSetdX(si, f0);
  arduino_buckboost_derivatives();

  /* f(:,2) = feval(odefile, t + hA(1), y + f*hB(:,1), args(:)(*)); */
  hB[0] = h * rt_ODE3_B[0][0];
  for (i = 0; i < nXc; i++) {
    x[i] = y[i] + (f0[i]*hB[0]);
  }

  rtsiSetT(si, t + h*rt_ODE3_A[0]);
  rtsiSetdX(si, f1);
  arduino_buckboost_step();
  arduino_buckboost_derivatives();

  /* f(:,3) = feval(odefile, t + hA(2), y + f*hB(:,2), args(:)(*)); */
  for (i = 0; i <= 1; i++) {
    hB[i] = h * rt_ODE3_B[1][i];
  }

  for (i = 0; i < nXc; i++) {
    x[i] = y[i] + (f0[i]*hB[0] + f1[i]*hB[1]);
  }

  rtsiSetT(si, t + h*rt_ODE3_A[1]);
  rtsiSetdX(si, f2);
  arduino_buckboost_step();
  arduino_buckboost_derivatives();

  /* tnew = t + hA(3);
     ynew = y + f*hB(:,3); */
  for (i = 0; i <= 2; i++) {
    hB[i] = h * rt_ODE3_B[2][i];
  }

  for (i = 0; i < nXc; i++) {
    x[i] = y[i] + (f0[i]*hB[0] + f1[i]*hB[1] + f2[i]*hB[2]);
  }

  rtsiSetT(si, tnew);
  rtsiSetSimTimeStep(si,MAJOR_TIME_STEP);
}

/* Model step function */
void arduino_buckboost_step(void)
{
  boolean_T stateChanged;
  if (rtmIsMajorTimeStep(arduino_buckboost_M)) {
    /* set solver stop time */
    if (!(arduino_buckboost_M->Timing.clockTick0+1)) {
      rtsiSetSolverStopTime(&arduino_buckboost_M->solverInfo,
                            ((arduino_buckboost_M->Timing.clockTickH0 + 1) *
        arduino_buckboost_M->Timing.stepSize0 * 4294967296.0));
    } else {
      rtsiSetSolverStopTime(&arduino_buckboost_M->solverInfo,
                            ((arduino_buckboost_M->Timing.clockTick0 + 1) *
        arduino_buckboost_M->Timing.stepSize0 +
        arduino_buckboost_M->Timing.clockTickH0 *
        arduino_buckboost_M->Timing.stepSize0 * 4294967296.0));
    }
  }                                    /* end MajorTimeStep */

  /* Update absolute time of base rate at minor time step */
  if (rtmIsMinorTimeStep(arduino_buckboost_M)) {
    arduino_buckboost_M->Timing.t[0] = rtsiGetT(&arduino_buckboost_M->solverInfo);
  }

  /* Chart: '<Root>/DC-to-DC Converter' incorporates:
   *  Constant: '<Root>/Vref'
   *  Constant: '<Root>/Vtol'
   *  Constant: '<Root>/i0'
   *  Constant: '<Root>/v0'
   */
  if (rtmIsMajorTimeStep(arduino_buckboost_M)) {
    stateChanged = 0;

    /* Gateway: DC-to-DC Converter */
    /* During: DC-to-DC Converter */
    if (arduino_buckboost_DW.is_active_c1_arduino_buckboost == 0U) {
      /* Entry: DC-to-DC Converter */
      arduino_buckboost_DW.is_active_c1_arduino_buckboost = 1U;

      /* Entry Internal: DC-to-DC Converter */
      /* Transition: '<S1>:64' */
      arduino_buckboost_X.SFunction_CSTATE[1] = 0.0;
      arduino_buckboost_X.SFunction_CSTATE[2] = 0.64600002765655518;
      stateChanged = true;
      arduino_buckboost_DW.is_c1_arduino_buckboost =
        arduino_buckboost_IN_Charging;
    } else {
      switch (arduino_buckboost_DW.is_c1_arduino_buckboost) {
       case arduino_buckboost_IN_Charging:
        /* During 'Charging': '<S1>:1' */
        if (arduino_buckboost_X.SFunction_CSTATE[2] >= 12.100000381469727) {
          /* Transition: '<S1>:57' */
          stateChanged = true;
          arduino_buckboost_DW.is_c1_arduino_buckboost =
            arduino_buckboos_IN_Discharging;
        }
        break;

       case arduino_buckboos_IN_Discharging:
        /* During 'Discharging': '<S1>:2' */
        if (arduino_buckboost_X.SFunction_CSTATE[1] <= 0.0) {
          /* Transition: '<S1>:48' */
          stateChanged = true;
          arduino_buckboost_DW.is_c1_arduino_buckboost =
            arduino_buckbo_IN_Discontinuous;
        } else {
          if (arduino_buckboost_X.SFunction_CSTATE[2] <= 11.899999618530273) {
            /* Transition: '<S1>:56' */
            stateChanged = true;
            arduino_buckboost_DW.is_c1_arduino_buckboost =
              arduino_buckboost_IN_Charging;
          }
        }
        break;

       default:
        /* During 'Discontinuous': '<S1>:45' */
        if (arduino_buckboost_X.SFunction_CSTATE[2] <= 11.899999618530273) {
          /* Transition: '<S1>:46' */
          stateChanged = true;
          arduino_buckboost_DW.is_c1_arduino_buckboost =
            arduino_buckboost_IN_Charging;
        }
        break;
      }
    }

    if (stateChanged) {
      rtsiSetBlockStateForSolverChangedAtMajorStep
        (&arduino_buckboost_M->solverInfo, true);
    }
  }

  /* End of Chart: '<Root>/DC-to-DC Converter' */
  if (rtmIsMajorTimeStep(arduino_buckboost_M)) {
    rt_ertODEUpdateContinuousStates(&arduino_buckboost_M->solverInfo);

    /* Update absolute time for base rate */
    /* The "clockTick0" counts the number of times the code of this task has
     * been executed. The absolute time is the multiplication of "clockTick0"
     * and "Timing.stepSize0". Size of "clockTick0" ensures timer will not
     * overflow during the application lifespan selected.
     * Timer of this task consists of two 32 bit unsigned integers.
     * The two integers represent the low bits Timing.clockTick0 and the high bits
     * Timing.clockTickH0. When the low bit overflows to 0, the high bits increment.
     */
    if (!(++arduino_buckboost_M->Timing.clockTick0)) {
      ++arduino_buckboost_M->Timing.clockTickH0;
    }

    arduino_buckboost_M->Timing.t[0] = rtsiGetSolverStopTime
      (&arduino_buckboost_M->solverInfo);

    {
      /* Update absolute timer for sample time: [2.0E-6s, 0.0s] */
      /* The "clockTick1" counts the number of times the code of this task has
       * been executed. The resolution of this integer timer is 2.0E-6, which is the step size
       * of the task. Size of "clockTick1" ensures timer will not overflow during the
       * application lifespan selected.
       * Timer of this task consists of two 32 bit unsigned integers.
       * The two integers represent the low bits Timing.clockTick1 and the high bits
       * Timing.clockTickH1. When the low bit overflows to 0, the high bits increment.
       */
      arduino_buckboost_M->Timing.clockTick1++;
      if (!arduino_buckboost_M->Timing.clockTick1) {
        arduino_buckboost_M->Timing.clockTickH1++;
      }
    }
  }                                    /* end MajorTimeStep */
}

/* Derivatives for root system: '<Root>' */
void arduino_buckboost_derivatives(void)
{
  XDot_arduino_buckboost_T *_rtXdot;
  _rtXdot = ((XDot_arduino_buckboost_T *) arduino_buckboost_M->derivs);

  /* Derivatives for Chart: '<Root>/DC-to-DC Converter' incorporates:
   *  Constant: '<Root>/Vsfixed'
   */
  _rtXdot->SFunction_CSTATE[0] = 0.0;
  _rtXdot->SFunction_CSTATE[1] = 0.0;
  _rtXdot->SFunction_CSTATE[2] = 0.0;
  switch (arduino_buckboost_DW.is_c1_arduino_buckboost) {
   case arduino_buckboost_IN_Charging:
    /* During 'Charging': '<S1>:1' */
    _rtXdot->SFunction_CSTATE[1] = (arduino_buckboost_DW.a00c *
      arduino_buckboost_X.SFunction_CSTATE[1] + arduino_buckboost_DW.a01c *
      arduino_buckboost_X.SFunction_CSTATE[2]) + arduino_buckboost_DW.b0c *
      24.0F;
    _rtXdot->SFunction_CSTATE[2] = (arduino_buckboost_DW.a10c *
      arduino_buckboost_X.SFunction_CSTATE[1] + arduino_buckboost_DW.a11c *
      arduino_buckboost_X.SFunction_CSTATE[2]) + arduino_buckboost_DW.b1c *
      24.0F;
    _rtXdot->SFunction_CSTATE[0] = 1.0;
    break;

   case arduino_buckboos_IN_Discharging:
    /* During 'Discharging': '<S1>:2' */
    _rtXdot->SFunction_CSTATE[1] = (arduino_buckboost_DW.a00o *
      arduino_buckboost_X.SFunction_CSTATE[1] + arduino_buckboost_DW.a01o *
      arduino_buckboost_X.SFunction_CSTATE[2]) + arduino_buckboost_DW.b0o *
      24.0F;
    _rtXdot->SFunction_CSTATE[2] = (arduino_buckboost_DW.a10o *
      arduino_buckboost_X.SFunction_CSTATE[1] + arduino_buckboost_DW.a11o *
      arduino_buckboost_X.SFunction_CSTATE[2]) + arduino_buckboost_DW.b1o *
      24.0F;
    _rtXdot->SFunction_CSTATE[0] = 1.0;
    break;

   default:
    /* During 'Discontinuous': '<S1>:45' */
    _rtXdot->SFunction_CSTATE[1] = (arduino_buckboost_DW.a00d *
      arduino_buckboost_X.SFunction_CSTATE[1] + arduino_buckboost_DW.a01d *
      arduino_buckboost_X.SFunction_CSTATE[2]) + arduino_buckboost_DW.b0d *
      24.0F;
    _rtXdot->SFunction_CSTATE[2] = (arduino_buckboost_DW.a10d *
      arduino_buckboost_X.SFunction_CSTATE[1] + arduino_buckboost_DW.a11d *
      arduino_buckboost_X.SFunction_CSTATE[2]) + arduino_buckboost_DW.b1d *
      24.0F;
    _rtXdot->SFunction_CSTATE[0] = 1.0;
    break;
  }

  /* End of Derivatives for Chart: '<Root>/DC-to-DC Converter' */
}

/* Model initialize function */
void arduino_buckboost_initialize(void)
{
  /* Registration code */

  /* initialize real-time model */
  (void) memset((void *)arduino_buckboost_M, 0,
                sizeof(RT_MODEL_arduino_buckboost_T));

  {
    /* Setup solver object */
    rtsiSetSimTimeStepPtr(&arduino_buckboost_M->solverInfo,
                          &arduino_buckboost_M->Timing.simTimeStep);
    rtsiSetTPtr(&arduino_buckboost_M->solverInfo, &rtmGetTPtr
                (arduino_buckboost_M));
    rtsiSetStepSizePtr(&arduino_buckboost_M->solverInfo,
                       &arduino_buckboost_M->Timing.stepSize0);
    rtsiSetdXPtr(&arduino_buckboost_M->solverInfo, &arduino_buckboost_M->derivs);
    rtsiSetContStatesPtr(&arduino_buckboost_M->solverInfo, (real_T **)
                         &arduino_buckboost_M->contStates);
    rtsiSetNumContStatesPtr(&arduino_buckboost_M->solverInfo,
      &arduino_buckboost_M->Sizes.numContStates);
    rtsiSetNumPeriodicContStatesPtr(&arduino_buckboost_M->solverInfo,
      &arduino_buckboost_M->Sizes.numPeriodicContStates);
    rtsiSetPeriodicContStateIndicesPtr(&arduino_buckboost_M->solverInfo,
      &arduino_buckboost_M->periodicContStateIndices);
    rtsiSetPeriodicContStateRangesPtr(&arduino_buckboost_M->solverInfo,
      &arduino_buckboost_M->periodicContStateRanges);
    rtsiSetErrorStatusPtr(&arduino_buckboost_M->solverInfo, (&rtmGetErrorStatus
      (arduino_buckboost_M)));
    rtsiSetRTModelPtr(&arduino_buckboost_M->solverInfo, arduino_buckboost_M);
  }

  rtsiSetSimTimeStep(&arduino_buckboost_M->solverInfo, MAJOR_TIME_STEP);
  arduino_buckboost_M->intgData.y = arduino_buckboost_M->odeY;
  arduino_buckboost_M->intgData.f[0] = arduino_buckboost_M->odeF[0];
  arduino_buckboost_M->intgData.f[1] = arduino_buckboost_M->odeF[1];
  arduino_buckboost_M->intgData.f[2] = arduino_buckboost_M->odeF[2];
  arduino_buckboost_M->contStates = ((X_arduino_buckboost_T *)
    &arduino_buckboost_X);
  rtsiSetSolverData(&arduino_buckboost_M->solverInfo, (void *)
                    &arduino_buckboost_M->intgData);
  rtsiSetSolverName(&arduino_buckboost_M->solverInfo,"ode3");
  rtmSetTPtr(arduino_buckboost_M, &arduino_buckboost_M->Timing.tArray[0]);
  arduino_buckboost_M->Timing.stepSize0 = 2.0E-6;

  /* states (continuous) */
  {
    (void) memset((void *)&arduino_buckboost_X, 0,
                  sizeof(X_arduino_buckboost_T));
  }

  /* states (dwork) */
  (void) memset((void *)&arduino_buckboost_DW, 0,
                sizeof(DW_arduino_buckboost_T));

  /* SystemInitialize for Chart: '<Root>/DC-to-DC Converter' */
  arduino_buckboost_DW.is_active_c1_arduino_buckboost = 0U;
  arduino_buckboost_DW.is_c1_arduino_buckboost = arduino_buck_IN_NO_ACTIVE_CHILD;
  arduino_buckboost_X.SFunction_CSTATE[0] = 0.0;
  arduino_buckboost_X.SFunction_CSTATE[1] = 0.0;
  arduino_buckboost_DW.a00o = -196.22641F;
  arduino_buckboost_DW.a01o = -377.35849F;
  arduino_buckboost_DW.a10o = 454.545471F;
  arduino_buckboost_DW.a11o = -45.4545441F;
  arduino_buckboost_DW.a00c = -271.69812F;
  arduino_buckboost_DW.a01c = -377.35849F;
  arduino_buckboost_DW.a10c = 454.545471F;
  arduino_buckboost_DW.a11c = -45.4545441F;
  arduino_buckboost_DW.b0o = 0.0F;
  arduino_buckboost_DW.b1o = 0.0F;
  arduino_buckboost_DW.b0c = 377.35849F;
  arduino_buckboost_DW.b1c = 0.0F;
  arduino_buckboost_X.SFunction_CSTATE[2] = 0.0;
  arduino_buckboost_DW.a01d = 0.0F;
  arduino_buckboost_DW.a10d = 0.0F;
  arduino_buckboost_DW.a11d = -45.4545441F;
  arduino_buckboost_DW.a00d = 0.0F;
  arduino_buckboost_DW.b0d = 0.0F;
  arduino_buckboost_DW.b1d = 0.0F;
}

/* Model terminate function */
void arduino_buckboost_terminate(void)
{
  /* (no terminate code required) */
}
