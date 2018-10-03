  /**
   *  Project:      
   *  Author:       
   *  Description:  Buck converter controller
   *  Updated:      
   */

  #include <SPI.h>
  #include "mcp_can.h"
  
  /** System Configuration
   *  SERIAL_BAUD : Serial connection baud rate (e.g. 115200)
   *  SERIAL_DISPLAY : Serial monitor output enabled/disabled [true|false]
   *  CAN_ENABLE : CAN communications enabled/disabled [true|false]
   */
  #define SERIAL_BAUD 115200
  #define SERIAL_DISPLAY true
  #define CAN_ENABLE true
  #define SPI_CS_PIN 9
  #define SPEED_SETPOINT 50 // Controller speed setpoint, km/h


  #define MODE_CLOSED 0
  #define MODE_OPEN 1
  #define MODE_DCM 2
  #define MODE_NONE 1000
  
  MCP_CAN CAN(SPI_CS_PIN);                                    // Set CS pin
  
  unsigned char payload[8];
  const char payload_zero[8] = {0,0,0,0,0,0,0,0};  // used to clear CAN frame data payload  

  /**  Controller Parameters
   */
  static int plant_mode = MODE_CLOSED;
  static int plant_mode_new = MODE_NONE;

  static float simulation_time;                 // Simulation clock [s]
  static float simulation_timestep = 2.0e-6;     // Simulation timestep [s]

  static float vc = 0.0;
  static float il = 0.0;

  static float Vref = 12.0;
  static float Vtol = Vref/120.0; // Tolerance level for hysteresis band 
 
void setup() {
  //Initialize serial and wait for port to open:
  Serial.begin(SERIAL_BAUD);
  while (!Serial) {
    ; // wait for serial port to connect. Needed for native USB
  }
  // Print simulation parameters
    Serial.println("Serial Connected, Hello World!");    

  // Initialize CAN board
  if (CAN_ENABLE) {
    while (CAN_OK != CAN.begin(CAN_500KBPS))              // init can bus : baudrate = 500k
    {
        Serial.println("CAN BUS Shield init fail");
        Serial.println("Init CAN BUS Shield again");
        delay(100);
    }
    Serial.println("CAN BUS Shield init ok!");
  }   
}


void loop() {
  // CAN Communications
  unsigned char len = 0;
  unsigned char buf[8];
  unsigned int canID;
  
  bool RESET_CONTROLLER = false;
  bool RECEIVED_PLANT_MODE = false;
  
  /* AT START OF EACH SIMULATION RUN  
   *  - Send CAN frame indicating control module startup 0x20:[AA 00 ..]
   *  - Initialize controller variables (e[], e_int, e_div)
   * AT START OF EACH CONTROL LOOP
   *  - Receive Vc
   *  - Determine new plant mode (toggle transitor)
   */
  if (CAN_ENABLE) {
    memcpy(payload, payload_zero, sizeof(payload)); // initialize data payload with zeroes
    payload[0]=0xAA;  // controller starting up
    CAN.sendMsgBuf(0x200, 0, 8, payload);
  }

  Serial.println("Starting to wait for incoming plant mode");

  while (!RECEIVED_PLANT_MODE) {
    if (CAN_ENABLE) {
      while (!(CAN_MSGAVAIL == CAN.checkReceive()) & !RESET_CONTROLLER) {
        ; // wait for CAN Rx
      }
      CAN.readMsgBuf(&len, buf);
      canID = CAN.getCanId();
       

      if (canID==0x100){

        memcpy(payload,buf,sizeof(buf));
        plant_mode = (long) (payload[0] >> 8);
        
        Serial.print("Plant mode rx: ");Serial.print(plant_mode);Serial.print("\n");
        RECEIVED_PLANT_MODE = true;
      }
    }
  }

  while (!RESET_CONTROLLER) {
    bool ALLOW_STEP_CONTROLLER = false;
    while (!ALLOW_STEP_CONTROLLER) {
      bool RECEIVED_VC = false;      
      bool COMPLETED_CTRL_CALC = false;
      bool TRANSMITTED_COMMAND = false;
            
      while (!RECEIVED_VC) {
        //Serial.println("WAITING FOR VC SIGNAL");
        if (CAN_ENABLE) {
          while (!(CAN_MSGAVAIL == CAN.checkReceive()) & !RESET_CONTROLLER) {
            ; // wait for CAN Rx
          }
          CAN.readMsgBuf(&len, buf);
          canID = CAN.getCanId();

           
          // CAN Rx 0x132 - Vc
          if (canID==0x132){
            if (SERIAL_DISPLAY) {          
              for (int i = 0; i<len; i++) {    // print the data
                  Serial.print(buf[i], HEX);
                  Serial.print("\t");
              }
            }
            memcpy(payload,buf,sizeof(buf));
            vc = (float) ((payload[0] << (8*3)) + (payload[1] << (8*2)) + (payload[2] << (8*1)) + (payload[3] << (8*0))) / 100;
            Serial.print("vc: ");
            Serial.println(vc,8);
            RECEIVED_VC = true;
          }
          if (canID==0x100){
            memcpy(payload,buf,sizeof(buf));
            if (payload[0]=0xFF){
              RESET_CONTROLLER = true;
              // CAN Tx 0x220 - Controller Status
              memcpy(payload, payload_zero, sizeof(payload)); // initialize data payload with zeroes
              payload[0]=0xFF;  // controller reset
              CAN.sendMsgBuf(0x220, 0, 8, payload);            
            }
          }          
        }
      }

      while (!COMPLETED_CTRL_CALC) {
        /* --- Controller Loop --- */

        plant_mode_new = plant_mode; // use last
        
        switch (plant_mode) {
          case MODE_CLOSED: {
            if (vc >= Vref+Vtol) {
              plant_mode_new = MODE_OPEN;
            }
            break;
          }
          case MODE_OPEN: {
            if (vc <= Vref-Vtol) {
              plant_mode_new = MODE_CLOSED;
            }
            break;
          }
          case MODE_DCM: {
            if (vc <= Vref-Vtol) {
              plant_mode_new = MODE_CLOSED;
            }
            break;
          }
          default: {
            // error or use last command
            plant_mode_new = plant_mode;
          }
        }
 
        COMPLETED_CTRL_CALC = true;
        if (SERIAL_DISPLAY) {
          Serial.print("Controller Calcuation Complete :: Mode: ");
          Serial.println(plant_mode_new);
        }
      }

      while (!TRANSMITTED_COMMAND) {
        if (CAN_ENABLE){
          // CAN Tx 0x210 - Ctrl command
          memcpy(payload, payload_zero, sizeof(payload)); // initialize data payload with zeroes
          payload[0] = (long) (plant_mode_new) >> (8*0) & 0xFF;
          CAN.sendMsgBuf(0x210, 0, 8, payload);
          delay(10);

          TRANSMITTED_COMMAND = true;
        }
      }
      if (RECEIVED_VC & COMPLETED_CTRL_CALC & TRANSMITTED_COMMAND) {
        ALLOW_STEP_CONTROLLER = true;
      }
    }
  } // end of reset block
}
