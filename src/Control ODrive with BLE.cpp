#include <Arduino.h>
#include <ArduinoBLE.h>
#include <HardwareSerial.h>
#include <ODriveArduino.h>
#include <Ramp.h>

// Printing with stream operator helper functions
template<class T> inline Print& operator <<(Print &obj,     T arg) { obj.print(arg);    return obj; }
template<>        inline Print& operator <<(Print &obj, float arg) { obj.print(arg, 4); return obj; }


//----------------------------------------------------------------------------------------------------------------------
// BLE UUIDs
//----------------------------------------------------------------------------------------------------------------------

#define BLE_UUID_MOTOR_INSTRUCTIONS_SERVICE        "9e661242-e694-4262-b393-61744b474383"
#define BLE_UUID_GET_G                             "73cce820-35ee-47f2-bca7-31105f673e20"
#define BLE_UUID_ROTATION_DIRECTION                "aae6924e-fdd1-40d7-9e71-b026bcec5cc9"
#define BLE_UUID_MEASUREMENT_TIME                  "80077d16-c5c6-48c4-8c10-7da0b6fe3e6f"
#define BLE_UUID_START_FINISH                      "40e0f2d5-2327-4930-83ca-c2ca729ca1bb"
#define BLE_UUID_STATUS                            "5720e451-6f72-49f1-8721-079d7e4bb497" 

//----------------------------------------------------------------------------------------------------------------------
// BLE
//----------------------------------------------------------------------------------------------------------------------

BLEService Motor_Instructions_Service(BLE_UUID_MOTOR_INSTRUCTIONS_SERVICE);
BLEStringCharacteristic Get_g_Characteristic(BLE_UUID_GET_G, BLERead | BLEWrite, 20);
BLEStringCharacteristic Rotation_Direction_Characteristic(BLE_UUID_ROTATION_DIRECTION, BLERead | BLEWrite, 20);
BLEStringCharacteristic Measurment_Time_Characteristic (BLE_UUID_MEASUREMENT_TIME, BLERead | BLEWrite, 20);
BLEStringCharacteristic Start_Finish_Characteristic(BLE_UUID_START_FINISH, BLERead | BLEWrite, 20);
BLEStringCharacteristic Status_Characteristic( BLE_UUID_STATUS, BLERead | BLEWrite, 20 );

////////////////////////////////
// Set up serial pins to the ODrive
////////////////////////////////
HardwareSerial& odrive_serial = Serial1;

//////ODrive object//////
ODriveArduino odrive(odrive_serial);

//////Ramp object//////
rampFloat myRamp;

//////Setup pin names//////
const int CCW_Button = D6;
const int CW_Button = D7;
const int LED_Red = D8;
const int LED_Green = D9;
const int ledPin = LED_BUILTIN;

//////BLE variables//////
String get_g;
String Rotation_Direction;
String Measurment_Time;
String Start_Finish;

//////Motor control variables//////
float get_g_to_float;
float Rotation_Speed = 0.0f;
float Final_Rotation_Speed = 3.5f;
float Rotation_Direction_to_float;
float Last_Rotation_Direction;
long Measurment_Time_to_long;
int Start_Finish_to_int = 0;

//////Ramp variables//////
unsigned long Rate = 1000; //duration of the interpolation in milliseconds

//////Timer variables//////
unsigned long Start_time;
unsigned long Current_time;
unsigned long print_time = 1000;//in milliseconds

//////speed calculation variables//////
float g = 9.81; //9.81 m/s^2
float r = 0.2; //20cm
float g_to_speed = sqrt(g/r)*(5/PI); //from g to turns/sec

//////Motor control functions//////
void Spin_CCW(float Rotation_Speed, float Rotation_Direction){
  digitalWrite(LED_Red, HIGH);
  odrive.SetVelocity(0, Rotation_Direction * Rotation_Speed);
}

void Spin_CW(float Rotation_Speed, float Rotation_Direction){
  digitalWrite(LED_Green, HIGH);
  odrive.SetVelocity(0, Rotation_Direction * Rotation_Speed);
}

void Stop(float Rotation_Speed, float Rotation_Direction){ 
  while(!myRamp.isFinished()){
    Current_time = millis();
    Rotation_Speed = myRamp.update();
    odrive.SetVelocity(0, Rotation_Direction * Rotation_Speed);
  
    //Serial.println(Rotation_Speed);
    //Serial.println("in 4");
   
  }
}

void Go(float Rotation_Speed, float Rotation_Direction, long Start_time, long Measurement_time){
  //////listen for BLE peripherals to connect//////
  BLEDevice central = BLE.central();

  if (central){
    Serial.print("Connected to central: ");
    Serial.println(central.address());

    while (central.connected()){
      digitalWrite(LEDB, LOW);
      Start_Finish = Start_Finish_Characteristic.value();
    // Rotation_Speed = Rotation_Direction = Measurement_time = 0          
    } 
    
    Start_Finish_to_int = Start_Finish.toInt();
    Serial.print("Disconnected from central: ");
    Serial.println(central.address());
    digitalWrite(LEDB, HIGH);
  }
  
  Current_time = millis();
  odrive.SetVelocity(0, Rotation_Direction * Rotation_Speed);
  
  if(Rotation_Direction == 1.0f){
    digitalWrite(LED_Red, HIGH);
  }
  if(Rotation_Direction == -1.0f){
    digitalWrite(LED_Green, HIGH);
  }

  if(Current_time - Start_time >= Measurement_time*1000){
    Start_Finish_to_int = 0;
    digitalWrite(LED_Red, LOW);
    digitalWrite(LED_Green, LOW);
    //Rotation_Speed = odrive.GetVelocity(0);
    myRamp.go(0.0f, Rate); 
    Stop(Rotation_Speed, Rotation_Direction);
    Serial.println("Done with go");
    Status_Characteristic.writeValue("Ready");
  }

   if(!Start_Finish_to_int){
    digitalWrite(LED_Red, LOW);
    digitalWrite(LED_Green, LOW);
    Rotation_Speed = odrive.GetVelocity(0);
    myRamp.go(0.0f, Rate); 
    Stop(Rotation_Speed, Rotation_Direction);
    Serial.println("Stopped");
    Status_Characteristic.writeValue("Ready");
  }
}

//////BLE functions//////
bool setupBleMode(){
  if (!BLE.begin()){
    Serial.println("Starting BLE failed!");
    digitalWrite(LEDR, LOW);
    delay(1000);
    digitalWrite(LEDR, HIGH);
    return false;
  }

  //////Set advertised local name and service UUID://////
  BLE.setDeviceName("Odrive Arduino");
  BLE.setLocalName("Odrive Arduino");
  BLE.setAdvertisedService( Motor_Instructions_Service );

  //////BLE add characteristics//////
  Motor_Instructions_Service.addCharacteristic(Get_g_Characteristic);
  Motor_Instructions_Service.addCharacteristic(Rotation_Direction_Characteristic);
  Motor_Instructions_Service.addCharacteristic(Measurment_Time_Characteristic);
  Motor_Instructions_Service.addCharacteristic(Start_Finish_Characteristic);
  Motor_Instructions_Service.addCharacteristic(Status_Characteristic);
  
  //////Add service//////
  BLE.addService(Motor_Instructions_Service);

  //////Set initial value//////
  Status_Characteristic.writeValue("Ready");

  //////start advertising//////
  BLE.advertise();

  return true;
}

void setup() {
  //////Configure input/output pins//////
  pinMode(CCW_Button, INPUT);
  pinMode(CW_Button, INPUT);
  pinMode(LED_Red, OUTPUT);
  pinMode(LED_Green, OUTPUT);
  pinMode(ledPin, OUTPUT);

  //////Configure the first state of the pins//////
  digitalWrite(ledPin, HIGH);
  digitalWrite(LED_Red, LOW);
  digitalWrite(LED_Green, LOW);
  
  //////ODrive uses 115200 baud//////
  odrive_serial.begin(115200);

  //////Serial to PC//////
  Serial.begin(115200);
  while (!Serial) ; // wait for Arduino Serial Monitor to open

  //////Start BLE//////
  if(setupBleMode())
  {
    digitalWrite(LEDB, LOW);
    delay(1000);
    digitalWrite(LEDB, HIGH); 
  //////print MAC address//////
    Serial.print("My BLE MAC:  ");
    Serial.println(BLE.address());   
    Serial.println("BLE Control ready");
  }

  //////Start Odrive//////
  Serial.println("ODriveArduino");
  Serial.println("Setting parameters...");

  //////Set Motor parameters////// 
  odrive_serial << "w axis" << 0 << ".controller.config.vel_limit " << 2000000.0f << '\n'; //speed limit
  odrive_serial << "w axis" << 0 << ".motor.config.current_lim " << 20.0f << '\n'; // currnet limit
  
  //////Run calibration sequence//////
  int motornum = 0;
  int requested_state;
  
  requested_state = AXIS_STATE_MOTOR_CALIBRATION;
  Serial << "Axis" << motornum << ": Requesting state " << requested_state << '\n';
  if(!odrive.run_state(motornum, requested_state, true)) return;
  
  requested_state = AXIS_STATE_ENCODER_OFFSET_CALIBRATION;
  Serial << "Axis" << motornum << ": Requesting state " << requested_state << '\n';
  if(!odrive.run_state(motornum, requested_state, true, 25.0f)) return;
  
  requested_state = AXIS_STATE_CLOSED_LOOP_CONTROL;
  Serial << "Axis" << motornum << ": Requesting state " << requested_state << '\n';
  if(!odrive.run_state(motornum, requested_state, false /*don't wait*/)) return;
  
  Serial.println("Odrive ready");
  
}

void loop() {
  //////listen for BLE peripherals to connect//////
  BLEDevice central = BLE.central();

  if (central){
    Serial.print("Connected to central: ");
    Serial.println(central.address());

    while (central.connected()){
      digitalWrite(LEDB, LOW);
      get_g = Get_g_Characteristic.value();
      Rotation_Direction = Rotation_Direction_Characteristic.value();
      Measurment_Time = Measurment_Time_Characteristic.value();
      Start_Finish = Start_Finish_Characteristic.value();           
    } 

    Serial.print("Disconnected from central: ");
    Serial.println(central.address());
    digitalWrite(LEDB, HIGH);
    
    ////// Convert strings to valid values //////
    get_g_to_float = get_g.toFloat();
    Rotation_Direction_to_float = Rotation_Direction.toFloat(); 
    Measurment_Time_to_long = Measurment_Time.toInt();
    Start_Finish_to_int = Start_Finish.toInt();

    Rotation_Speed = sqrt(get_g_to_float)*g_to_speed;
        
    Serial.print("get_g = ");
    Serial.print(get_g_to_float);
    Serial.print(" g, ");
    Serial.print("Rotation_Speed = ");
    Serial.print(Rotation_Speed);
    Serial.print(" turns/sec, ");
    Serial.print("Rotation_Direction = ");
    Serial.print(Rotation_Direction_to_float);
    Serial.print(", ");
    Serial.print("Measurment_Time = ");
    Serial.print(Measurment_Time_to_long);
    Serial.print(" sec, ");
    Serial.print("Start_Finish = ");
    Serial.print(Start_Finish_to_int);
    Serial.println(" ");
    
    Start_time = millis();
    myRamp.go(Rotation_Speed, Rate);
  } 
  
    //////Measurement//////
    while(Start_Finish_to_int){ 
      Rotation_Speed = myRamp.update();
           
      //Serial.println(Rotation_Speed);
      //Serial.println("in go");
      
      Serial.print("Velocity: ");
      Serial.println(odrive.GetVelocity(0));
      //Serial.print(", ");
      //Serial.print("Position: ");
      //Serial.print(odrive.GetPosition(0));
      //Serial.print(", ");
      //Serial.print("Counts: ");
      //Serial.println(odrive.GetPositionCounts(0));
  
      Go(Rotation_Speed, Rotation_Direction_to_float, Start_time, Measurment_Time_to_long);
    }
    
  //////Button controls//////
  while(digitalRead(CCW_Button)){
    Rotation_Direction_to_float = 1.0f;
    Rotation_Speed = Final_Rotation_Speed;    
    Spin_CCW(Rotation_Speed, Rotation_Direction_to_float);
    Last_Rotation_Direction = Rotation_Direction_to_float;
    myRamp.go(0.0f, Rate - 850);
    Serial.print("Velocity: ");
    Serial.println(odrive.GetVelocity(0));
    //Serial.print(", ");
    //Serial.print("Position: ");
    //Serial.print(odrive.GetPosition(0));
    //Serial.print(", ");
    //Serial.print("Counts: ");
    //Serial.println(odrive.GetPositionCounts(0));
   }
  
  while(digitalRead(CW_Button)){
    Rotation_Direction_to_float = -1.0f;
    Rotation_Speed = Final_Rotation_Speed;    
    Spin_CW(Rotation_Speed, Rotation_Direction_to_float);
    Last_Rotation_Direction = Rotation_Direction_to_float;
    myRamp.go(0.0f, Rate - 850);
    Serial.print("Velocity: ");
    Serial.println(odrive.GetVelocity(0));
    //Serial.print(", ");
    //Serial.print("Position: ");
    //Serial.print(odrive.GetPosition(0));
    //Serial.print(", ");
    //Serial.print("Counts: ");
    //Serial.println(odrive.GetPositionCounts(0));
  }
  
  digitalWrite(LED_Red, LOW);
  digitalWrite(LED_Green, LOW);
  Rotation_Speed = 0.0f;
  Stop(Rotation_Speed, Last_Rotation_Direction);
}

