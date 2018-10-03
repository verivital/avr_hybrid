  /* 
   *  Project:      LANSCE Experiment 2018
   *  Author:       Chris Shum (University of Waterloo, Real-time Embedded Software Group)
   *  Description:  Cruise control application, plant module (controller/plant implementation)
   *  Updated:      2018-09-20
   */

  #include <SPI.h>
  #include "mcp_can.h"
  
  /* System Configuration
   *  SERIAL_BAUD : Serial connection baud rate (e.g. 115200)
   *  SERIAL_DISPLAY : Serial monitor output enabled/disabled [true|false]
   *  CAN_ENABLE : CAN communications enabled/disabled [true|false]
   */
  #define SERIAL_BAUD 115200        // Define Serial Monitor baud rate [115200]
  #define SERIAL_DISPLAY true       // Enable Serial Monitor output [true|false]
  #define CAN_ENABLE true           // Enable CAN traffic [true|false]
  #define SPI_CS_PIN 9              // Define SPI CS Pin (Default: 9)
  #define TIMEOUT_THRESH 5          // Loops to wait before stepping plant
  #define SIMULATION_DURATION 15    // Simulation stop time [s]

  MCP_CAN CAN(SPI_CS_PIN);          // Set CS pin
  
  unsigned char payload[8];         // CAN frame data payload
  const char payload_zero[8] = {0,0,0,0,0,0,0,0};  // used to clear CAN frame data payload  

  #define MODE_CLOSED 0
  #define MODE_OPEN 1
  #define MODE_DCM 2
  #define MODE_NONE 1000
 
  /* Plant Parameters
   *  Assumptions:
   */

  /* Plant parameters
   */
  static float T = 1.0/50000.0;
  static float Vs = 24.0;
  static float Vref = 12.0;
  static float Vtol = Vref/120.0; // Tolerance level for hysteresis band 
  static float C = 2.2e-3;
  static float L = 2.65e-3;
  static float R = 10.0; // load resistance
  static float rs = 200.0e-3; // switching loss
  static float rL = 520.0e-3; //  inductor loss
  static float Tmax = T*1000.0; // number of periods

  // switch closed
  static float Ac_00 = -1*(rs+rL)/L;
  static float Ac_01 = -(1/L);
  static float Ac_10 = (1/C);
  static float Ac_11 = -(1/(R*C));
  static float Bc_0 = (1/L);
  static float Bc_1 = 0.0;

  // switch open
  static float Ao_00 = -rL/L;
  static float Ao_01 = -(1/L);
  static float Ao_10 = (1/C);
  static float Ao_11 = -(1/(R*C));
  static float Bo_0 = 0.0;
  static float Bo_1 = 0.0;

  // dcm and switch open
  static float Ad_00 = 0.0;
  static float Ad_01 = 0.0;
  static float Ad_10 = 0.0;
  static float Ad_11 = -(1/(R*C));
  static float Bd_0 = 0.0;
  static float Bd_1 = 0.0;

  /* Simulation Global Variables
   */
  static float vc;
  static float il;

  static float vc_new;
  static float il_new;

  static int plant_mode = MODE_CLOSED;
  static int plant_mode_new = MODE_NONE;
  static float il0 = 0.0;
  static float vc0 = 0.646;

  /* Simulation Parameters
   */
   static bool simulation_stop = false;             // Simulation STOP flag - stops loop()
   static float simulation_time;                    // Simulation clock [s]
   static float simulation_timestep = 2.0e-6;        // Simulation timestep [s]
   static float simulation_duration = SIMULATION_DURATION;       // Simulation stop time [s]
//   static float sch_cycle_lead[] = {0.1};           //{0,1,2,3,4,5,6,7,8,9,10,9,8,7,6,5,4,3,2,1,0};   // Lead vehicle drive schedule [m/s]

void setup() {
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
}


bool LOOP_FOREVER = true;
void loop() {
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

    Serial.println("beginning to wait for CAN rx");

    //RECEIVED_PID_PARAM = true;
    
    while (!RECEIVED_PID_PARAM) {
      if (CAN_ENABLE) {
        while (!(CAN_MSGAVAIL == CAN.checkReceive())) {
          ; // wait for CAN Rx
        }
        CAN.readMsgBuf(&len, buf);
        canID = CAN.getCanId();
         
        // CAN Rx 0x230 -
        if (canID==0x200){
          plant_mode_new = (long) ((payload[0] << (8*1)) + (payload[1] << (8*0)));

        Serial.print("Plant mode rx: ");Serial.print(plant_mode);Serial.print("\n");
          
          RECEIVED_PID_PARAM = true;
        }
      }
    }
    
    // Initialize simulation (configuration, state variables)
    simulation_time = 0;
    il = il0;
    vc = vc0;
    il_new = il0;
    vc_new = vc0;

    
    if (SERIAL_DISPLAY) {
      /*
      Serial.println("System parameters");
      Serial.print(Ac_00);
      Serial.print(", ");
      Serial.print(Ac_01);
      Serial.print(", ");
      Serial.print(Ac_10);
      Serial.print(", ");
      Serial.print(Ac_11);
      Serial.print(", ");
      Serial.print(Bc_0);
      Serial.print(", ");
      Serial.print(Bc_1);
      Serial.print(", ");
      */


      
      Serial.println("---");
      Serial.println("!!! Simulation Starting");
      Serial.println("---");
    }
    bool ALLOW_STEP_PLANT = false;
    while ((simulation_time <= simulation_duration) && (!simulation_stop)) {
      // Program Flow Control Flags
        bool TRANSMITTED_VC = false;
        bool RECEIVED_CTRL_COMMAND = false;
        plant_mode_new = MODE_NONE;

      
      while (!ALLOW_STEP_PLANT) {

  
        if (CAN_ENABLE) {
          // CAN Tx 0x0130 - Timestamp
          if (SERIAL_DISPLAY) {
            Serial.println(simulation_time, 6);
          }
          
          memcpy(payload, payload_zero, sizeof(payload)); // initialize data payload with zeroes
          payload[0] = (long) (simulation_time/simulation_timestep) >> (8*3) & 0xFF;
          payload[1] = (long) (simulation_time/simulation_timestep) >> (8*2) & 0xFF;
          payload[2] = (long) (simulation_time/simulation_timestep) >> (8*1) & 0xFF;
          payload[3] = (long) (simulation_time/simulation_timestep) >> (8*0) & 0xFF;
          CAN.sendMsgBuf(0x130, 0, 8, payload);
    
          // CAN Tx 0x132 - 
          memcpy(payload, payload_zero, sizeof(payload)); // initialize data payload with zeroes
          payload[0] = (long) (vc * 100) >> (8*3) & 0xFF;
          payload[1] = (long) (vc * 100) >> (8*2) & 0xFF;
          payload[2] = (long) (vc * 100) >> (8*1) & 0xFF;
          payload[3] = (long) (vc * 100) >> (8*0) & 0xFF;

          Serial.print("VC IN CAN BUFFER BEFORE SEND: ");
          if (SERIAL_DISPLAY) {          
            for (int i = 0; i<len; i++) {    // print the data
                Serial.print(buf[i], HEX);
                Serial.print("\t");
            }
          }
          
          CAN.sendMsgBuf(0x132, 0, 8, payload);
          
          TRANSMITTED_VC = true;

          memcpy(payload, payload_zero, sizeof(payload)); // initialize data payload with zeroes
          payload[0] = (long) (plant_mode) & 0xFF;
          CAN.sendMsgBuf(0x100, 0, 8, payload);
        }
  
        int CAN_TIMEOUT=0;
        while (!RECEIVED_CTRL_COMMAND) {
          if (CAN_ENABLE) {
            while (!(CAN_MSGAVAIL == CAN.checkReceive()) & simulation_time>simulation_timestep & !(CAN_TIMEOUT>TIMEOUT_THRESH) ) {
              if (SERIAL_DISPLAY) {Serial.println("Waiting for CAN Message from CONTROLLER...");}
                CAN_TIMEOUT++;
                if (CAN_TIMEOUT>TIMEOUT_THRESH) {
                  RECEIVED_CTRL_COMMAND = true; // hold last on timeout
                }
              ; // wait for CAN Rx
            }
            CAN.readMsgBuf(&len, buf);
            canID = CAN.getCanId();
            
            // CAN Tx 0x210 - 
            Serial.print("plant received can cmd:");
            Serial.println(canID);
            if (canID==0x210){
              memcpy(payload,buf,sizeof(buf));
              plant_mode_new = (long) (payload[0] << (8*0));

                Serial.print("Received plant_mode: ");Serial.println(plant_mode_new);
             }
              RECEIVED_CTRL_COMMAND = true;
            }
          }
        
        
        /* --- Plant Loop ---
         * Calculate forces on ego vehicle
         * Sum of wheel force (F_w), drag force (F_d)
         */
        if (SERIAL_DISPLAY) {     
          Serial.print("vc, il: ");
          Serial.print(vc);
          Serial.print(", ");
          Serial.println(il);
        }


        
        
        switch (plant_mode) {
          case MODE_CLOSED: {
            il_new = il + (Ac_00*il + Ac_01*vc + Bc_0*Vs)*simulation_timestep;
            vc_new = vc + (Ac_10*il + Ac_11*vc + Bc_1*Vs)*simulation_timestep;

            break;
          }
          case MODE_OPEN: {
            il_new = il + (Ao_00*il + Ao_01*vc + Bo_0*Vs)*simulation_timestep;
            vc_new = vc + (Ao_10*il + Ao_11*vc + Bo_1*Vs)*simulation_timestep;

            if (il <= 0.0) {
              plant_mode_new = MODE_DCM;
              il_new = 0.0; // cannot physically become negative
            }

            break;
          }
          case MODE_DCM: {
            il_new = il + (Ad_00*il + Ad_01*vc + Bd_0*Vs)*simulation_timestep;
            vc_new = vc + (Ad_10*il + Ad_11*vc + Bd_1*Vs)*simulation_timestep;

            break;
          }
          default: {
            // error
          }
        }

        vc = vc_new;
        il = il_new;

        if (plant_mode_new != MODE_NONE) {
          Serial.println("MODE SWITCH");
          plant_mode = plant_mode_new;
          plant_mode_new = MODE_NONE;
        }
        
        if (SERIAL_DISPLAY) {
          Serial.print("vc: ");Serial.println(vc,5);
          Serial.print("il: ");Serial.println(il,5);
          Serial.print("mode: ");Serial.println(plant_mode);
        }
         
        if (RECEIVED_CTRL_COMMAND && TRANSMITTED_VC) {
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
