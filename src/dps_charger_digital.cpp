/*
required libs

> pio lib install 2 13 2104 1266 6189

*/

#include "Arduino.h"
#ifdef ESP32
#include <WiFi.h>
#endif
#ifdef ESP8266
#include <ESP8266WiFi.h>
#endif


#include <ArduinoOTA.h>
//#include <SPI.h>
#include <Wire.h>
#include <PID_v1.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>
//#include <Fonts/Picopixel.h>
#include <Fonts/Org_01.h>
#include "RemoteDebug.h"  //https://github.com/JoaoLopesF/RemoteDebug
#include "RemoteDebugger.h"    //https://github.com/JoaoLopesF/RemoteDebugger

#ifdef ESP8266
#define OLED_RESET 0
#define PSU_ENABLE_PIN D5
#define PSU_VOLTAGE_PIN D8
#define BUTTON_A_PIN D3
#define BUTTON_B_PIN D4
#define PSU_I2C_SDA D2
#define PSU_I2C_SCL D1
#endif

#ifdef ESP32
#define OLED_RESET -1
#define PSU_ENABLE_PIN 13
#define PSU_VOLTAGE_PIN 26
#define BUTTON_A_PIN 16
#define BUTTON_B_PIN 17
#define PSU_I2C_SDA 21
#define PSU_I2C_SCL 22
#endif

Adafruit_SSD1306 display(OLED_RESET);


#define ANALOG_FREQ_RANGE 1023

#include "wifi-config.h" //#define STASSID and STAPSK for your network

RemoteDebug Debug;

struct Battery_type {
  double max_voltage;
  double charge_voltage;
  double max_current;
  double charge_current;
} typedef Battery_type;

Battery_type LiFePO4_1s = {4.2, 3.65, 50., 25.};
Battery_type LiFePO4_2s = {4.2 * 2 , 3.65 * 2, 120., 80.};

Battery_type battery = LiFePO4_2s;

enum charger_state_type {CS_INIT=0, CS_IDLE, CS_SELF_TEST, CS_CALIBRATION, CS_CC, CS_CV, CS_DONE, CS_ABORT} typedef charger_state_type;

charger_state_type charger_state = CS_INIT;
charger_state_type charger_state_prev = CS_ABORT;

char psu_spn_supported[] = "44";

char psu_spn[11];
char psu_date[9];
char psu_name[27];
char psu_mfu[5];
char psu_ct[15];

uint8_t psu_mcu_addr = 0xFF;
uint8_t psu_mem_addr = 0xFF;


double psu_grid_voltage = 0;
double psu_grid_current = 0;
double psu_grid_power = 0;
double psu_out_voltage = 0;
double psu_out_current = 0;
//double psu_out_current_filtered = 0;
double psu_out_power = 0;
double psu_out_max_current = 0;
double psu_intake_temp = 0;
double psu_internal_temp = 0;
double psu_fan_speed_actual = 0;
double psu_fan_speed_desired = 0;
bool   psu_is_enabled = false;


double vreg_duty;

double desired_voltage = .0;
double desired_current = .0;


void reconnect_wifi();

void display_text(String);
void update_display();

void read_eeprom();
double estimate_duty_from_voltage(double);
void print_charger_state();

String display_error = "";

/****************************************************** U T I L I T Y */

void print_byte(uint8_t b){
  /*rdebugV("0x");
  if (b < 16)
    rdebugV("0");
  rdebugV(b);*/
  debugI("0x%02x", b);
}

    

/****************************************************** i 2 C - f u n c t i o n s */


void scan_for_device(uint8_t from, uint8_t to, uint8_t& ret) {
  byte address;
      Wire.begin(PSU_I2C_SDA, PSU_I2C_SCL);  
  
  for (address = from ; address <= to; address++ )  {
    debugI("Testing: 0x%02x", address);
    Wire.beginTransmission(address);
    uint8_t ack = Wire.endTransmission();
    if (ack == 0){
      debugI("Device found at 0x%02x", address);
      ret = address;
      return;
    } 
  }

  if (ret == 0xFF) {
    debugE("No device found between 0x%02x and 0x%02x!", from, to);
  }
  
}


bool i2c_ping(uint8_t addr) {
  return true; //OBS
  debugI("i2c-pinging 0x%02x", addr);
  Wire.beginTransmission(addr);
  uint8_t ack = Wire.endTransmission();
  return (ack == 0);
}
  
  /*    
void i2c_bus_scan() {
  for (uint8_t address = 0x00 ; address <= 0x7F; address++ )  {
    Wire.beginTransmission(address);
    uint8_t ack = Wire.endTransmission();
    
    if (ack == 0){
        debugI("0x%02x ", address);
    } else {
        //debugI(".... ");
    }
    if (!(address % 8)){
      //debugI("\n");
    }
  }
  debugI("Done scanning");
}
      
*/

/****************************************************** H O U S E K E E P I N G */



//long lastYield = millis();
void my_yield(){
  yield();
  Debug.handle();
  //if (millis() - lastYield < 500) return;
  if (WiFi.status() != WL_CONNECTED) {  
    reconnect_wifi();
  }
  //handleTelnet();
  
  ArduinoOTA.handle();
}

/*
void handleTelnet() {
    if (TelnetServer.hasClient()) {
      if (!Telnet || !Telnet.connected()) {
        if (Telnet) Telnet.stop();
        Telnet = TelnetServer.available();
      } else {
        TelnetServer.available().stop();
      }
    }
}
*/
void reconnect_wifi() {  

    WiFi.mode(WIFI_STA);  
    WiFi.begin(LOCAL_SSID, LOCAL_PSK);  
    Serial.printf("Connecting to: %s\r\n", LOCAL_SSID);
    /*IPAddress ip(10,0,0,167);   
    IPAddress gateway(10,0,0,1);   
    IPAddress subnet(255,255,255,0);   
    WiFi.config(ip, gateway, subnet);*/
    while (WiFi.status() != WL_CONNECTED) {  
      Serial.println("Connecting to wifi...");
      delay(1000);
    }  
    String ip = WiFi.localIP().toString();
    Serial.printf("IP address: %s\n", ip.c_str());
}  


void reconnect_psu(){
  
  if (!i2c_ping(psu_mem_addr)){
    Wire.begin(PSU_I2C_SDA, PSU_I2C_SCL);  
    debugI("Connecting to PSU");

    while (psu_mem_addr == 0xFF){
      debugI("Scanning for EEPROM.");
      scan_for_device(0x50, 0x57, psu_mem_addr);
      my_yield();
    }
    read_eeprom();
  }
  if (!i2c_ping(psu_mem_addr)){
    Wire.begin(PSU_I2C_SDA, PSU_I2C_SCL);  
    debugI("Connecting to PSU");
    while (psu_mcu_addr == 0xFF){
      debugI("Scanning for MCU.");
      scan_for_device(0x58, 0x5F, psu_mcu_addr);
      my_yield();
    }
  }
  delay(500);
}

/****************************************************** P S U  E E P R O M */

uint8_t read_eeprom_byte(long addr)
{
  Wire.beginTransmission(psu_mem_addr);

  Wire.write((uint8_t)(addr));
  Wire.endTransmission();

  Wire.requestFrom((uint8_t)psu_mem_addr, (uint8_t)1);

  uint8_t rdata = 0xFF;
  if (Wire.available()) rdata = Wire.read();
  return rdata;
}

void read_eeprom_string(uint8_t addr, uint8_t len, char* s)
{
  //esp rebootar här. kolla varför
  debugI("Reading eeprom string from 0x%02x, expecting %d chars", addr, len);
  my_yield();
  Wire.beginTransmission(psu_mem_addr);

  Wire.write((uint8_t)(addr));
  Wire.endTransmission();

  Wire.requestFrom((uint8_t)psu_mem_addr, (uint8_t)len);


  uint8_t i=0;
  while (Wire.available() && (i < len)){
    s[i] = Wire.read();
    i++;
    s[i] = '\0';
    debugI("Read EEPROM string: %s", s);
    my_yield();
  }
  s[i] = '\0';

}


void read_eeprom(){
  read_eeprom_string(0x12, 10, psu_spn);
  read_eeprom_string(0x1D, 8, psu_date);
  read_eeprom_string(0x32, 26, psu_name);
  read_eeprom_string(0x5B, 14, psu_ct);
  
  debugI("Found: %s (%s)", psu_name, psu_spn);
  

}


/****************************************************** P S U  M C U */


bool checksum(uint8_t* msg, uint8_t msg_len){

  uint8_t cs = 0;
  for (uint8_t i=0; i<msg_len; ++i){
    cs += msg[i];
  }
  cs = (( 0xFF - cs) + 1) & 0xFF;
  if (cs) { 
    debugE("Wrong checksum: 0x%02x", cs);
  }/* else {
    debugE("Correct checksum");
  }*/
  return (cs==0);

}

uint8_t calculate_checksum(uint8_t reg, uint8_t* msg, uint8_t msg_len){
  uint8_t cs = (psu_mcu_addr<<1) + reg;
  for (uint8_t i=0; i<msg_len; ++i){
    cs += msg[i];
  }
  return ((( 0xFF - cs) + 1) & 0xFF);
}

uint8_t calculate_checksum_u16(uint8_t reg, uint16_t msg){
  uint8_t cs = (psu_mcu_addr<<1) + reg + (msg >> 8) + (msg & 0xFF);
  return ((( 0xFF - cs) + 1) & 0xFF);
}

bool read_psu_mcu(uint8_t reg, uint8_t len, uint8_t* msg){


    Wire.beginTransmission(psu_mcu_addr);
    Wire.write((uint8_t)(reg));
    debugI("> 0x%02x ", reg);
    Wire.write(calculate_checksum_u16(reg, 0));
    debugI("> 0x%02x ", calculate_checksum_u16(reg, 0));
    Wire.endTransmission(false);

    Wire.requestFrom((uint8_t)psu_mcu_addr, (uint8_t)len, (uint8_t)false); //casting to avoid warnings
    
    uint8_t i = 0;
    
    while (Wire.available() && (i < len)){ 
      msg[i] = Wire.read(); 
      debugI("< 0x%02x ", msg[i]);
      i++;
    }
    if (i!=len){
      debugE("Expected to read %d bytes, but got %d bytes.", len, i);
    }
    
    return checksum(msg, len);
}

bool read_psu_mcu_u8(uint8_t reg, uint8_t& ret){
  uint8_t msg[2];
  if (!read_psu_mcu(reg, 2, msg)){
    return false;
  }
  ret = msg[0];
  return true;
}

bool read_psu_mcu_u16(uint8_t reg, uint16_t& ret){
  uint8_t msg[3];
  if (!read_psu_mcu(reg, 3, msg)){
    return false;
  }
  ret = (msg[1] << 8) + msg[0];
  return true;
}


bool read_psu_mcu_f16(uint8_t reg, double scale, double& ret){
  uint16_t u16;
  if (!read_psu_mcu_u16(reg, u16)){
    return false;
  }
  ret = scale * u16;
  return true;
}

bool read_psu_mcu_flags16(uint8_t reg, uint16_t& ret){
  if (!read_psu_mcu_u16(reg, ret)){
    return false;
  }
  return true;
}


bool read_psu_grid_voltage(double& ret){
  return (read_psu_mcu_f16(0x08, 0.032, ret));
}

bool read_psu_grid_current(double& ret){
  return (read_psu_mcu_f16(0x0A, 1/256., ret));
}

bool read_psu_grid_power(double& ret){
  return (read_psu_mcu_f16(0x0C, 2., ret));
}

bool read_psu_out_voltage(double& ret){
  return (read_psu_mcu_f16(0x0E, 1./256, ret));
}

bool read_psu_out_current(double& ret){
  bool b = (read_psu_mcu_f16(0x10, 1./64, ret) );
  /*if (ret < 15.44)
    ret = 0;
  else
    ret -= 15.44; // TODO: Why this offset?*/
  return b;

}

bool read_psu_out_power(double& ret){
  return (read_psu_mcu_f16(0x1A, 2., ret));
}

bool read_psu_intake_temp(double& ret){
  bool r = (read_psu_mcu_f16(0x18, 1./32., ret));
  //ret = (ret - 32.) * 5./9.;
  return r;
}

bool read_psu_internal_temp(double& ret){
  bool r = (read_psu_mcu_f16(0x1C, 1./32., ret));
  //ret = (ret - 32.) * 5./9.;
  return r;
}

bool read_psu_fan_speed_actual(double& ret){
  return (read_psu_mcu_f16(0x1E, 1., ret));
}

bool read_psu_fan_speed_desired(double& ret){
  return (read_psu_mcu_f16(0x40, 1., ret));
}

bool read_psu_out_max_current(double& ret){
  return (read_psu_mcu_f16(0x36, 1./128, ret));
}

bool read_psu_is_enabled(bool& ret){
  uint16_t u16;
  bool r = read_psu_mcu_flags16(0x02, u16);
  if (!r) return false;
  ret = ((u16 & 0x05) == 0x05);
  /*print_byte(u16 >>8);
  rdebugV(" ");
  print_byte(u16 & 0xFF);
  //rdebugVln();*/
  return true;
}

bool read_psu_data(){
  return (
  //(read_psu_grid_voltage(psu_grid_voltage)) &&
  //(read_psu_grid_current(psu_grid_current)) &&
  //(read_psu_grid_power(psu_grid_power)) &&
  (read_psu_out_voltage(psu_out_voltage)) &&
  (read_psu_out_current(psu_out_current)) &&
  //(read_psu_out_power(psu_out_power)) &&
  //(read_psu_intake_temp(psu_intake_temp)) &&
  //(read_psu_internal_temp(psu_internal_temp)) &&
  //(read_psu_fan_speed_actual(psu_fan_speed_actual)) &&
  //(read_psu_out_max_current(psu_out_max_current)) &&
  //(read_psu_fan_speed_desired(psu_fan_speed_desired)

  (read_psu_is_enabled(psu_is_enabled)) 
  
  );
}

void write_psu_mcu_nothing(uint8_t reg){
  Wire.beginTransmission(psu_mcu_addr);
  Wire.write((uint8_t)(reg));
  Wire.write((uint8_t)(calculate_checksum_u16(reg, 0)));
  Wire.endTransmission();
}


void write_psu_mcu_u8(uint8_t reg, uint8_t val){
  
  
  Wire.beginTransmission(psu_mcu_addr);
  Wire.write((uint8_t)(reg));
  Wire.write((uint8_t)(val));
  Wire.write((uint8_t)(calculate_checksum_u16(reg, val)));
  Wire.endTransmission();

}

void write_psu_mcu_u16(uint8_t reg, uint16_t val){

  uint8_t val_lsb = val & 0xff;
  uint8_t val_msb = val >> 8;
  Wire.beginTransmission(psu_mcu_addr);
  Wire.write((uint8_t)(reg));
  Wire.write((uint8_t)(val_lsb));
  Wire.write((uint8_t)(val_msb));
  Wire.write((uint8_t)(calculate_checksum_u16(reg, val_lsb + val_msb)));
  Wire.endTransmission();

}


void write_psu_mcu_f16(uint8_t reg, double val, double scale){
  uint16_t v = (uint16_t)(val / scale);
  write_psu_mcu_u16(reg, v);
}

void force_psu_fan(uint16_t rpm){
  write_psu_mcu_u16(0x40, rpm);
}


/******************************************** P S U   H A R D W A R E */

#ifdef ESP8266
double abs(double d){
  if (d<0) return -d;
  return d;
}
#endif

void psu_enable(){
  rdebugVln("psu_enable");
  pinMode(PSU_VOLTAGE_PIN, OUTPUT);
  digitalWrite(PSU_VOLTAGE_PIN, HIGH);
  delay(1);
  digitalWrite(PSU_ENABLE_PIN, LOW);

  while(
    (!read_psu_is_enabled(psu_is_enabled)) || //not able to read PSU
    (!psu_is_enabled)) // OR psu is not enabled
    {
    
    read_psu_out_voltage(psu_out_voltage);
    debugI("PSU Enabling: %.3fV", psu_out_voltage);
    delay(100);
    my_yield();
  }
}

void psu_settle(){


  double last_voltage = 0;
  while (!read_psu_out_voltage(psu_out_voltage) ||
    abs(psu_out_voltage - last_voltage) > 0.02){
    last_voltage = last_voltage + .25 * (psu_out_voltage - last_voltage); //running average
    delay(5);
    debugI("Waiting for voltage to stabilize. %.3fV", psu_out_voltage);
    my_yield();
    update_display();
  }
  desired_voltage = psu_out_voltage;
  debugI("Stabilized voltage: %.3fV", psu_out_voltage);
  
  vreg_duty = estimate_duty_from_voltage(psu_out_voltage)  /* - margin*/;
  pinMode(PSU_VOLTAGE_PIN, OUTPUT);
  //analogWrite(PSU_VOLTAGE_PIN, vreg_duty);
}


void psu_disable(){
  debugI("psu_disable");
  pinMode(PSU_VOLTAGE_PIN, INPUT);
  digitalWrite(PSU_ENABLE_PIN, HIGH);
}


double estimate_duty_from_voltage(double voltage){
  return ANALOG_FREQ_RANGE;
  /*double duty;
  //Simple linear interpolation. Maybe a lookup would be more exact
  double step_per_volt = ((double)ANALOG_FREQ_RANGE - 0.) / (13.7 - 2.5);
  duty = ANALOG_FREQ_RANGE - step_per_volt * voltage;

  return duty;*/
}

double pid_current_kp = 0.1;
double pid_current_ki = 0.1;
double pid_current_kd = 0.1;

PID current_regulator(&psu_out_current, &vreg_duty, &desired_current, pid_current_kp, pid_current_ki, pid_current_kd, P_ON_E, REVERSE);

unsigned long last_current_regulation = millis();
void regulate_current(){
  read_psu_data();
  if (current_regulator.Compute()){
    //analogWrite(PSU_VOLTAGE_PIN, vreg_duty);
  } else {
    debugI("Skipped current PID calculation.");
  }
  debugI("current_regulator freq: %.1f/s (%dms)",(double) (1000. / (millis() - last_current_regulation)), (uint16_t)(millis() - last_current_regulation));
  if ((/*current_regulator.GetSampleTime()*/ 50 * 2) < (millis() - last_current_regulation)){
    debugI("current_regulator not called often enough.");
  }

  
  last_current_regulation = millis();

}

double pid_voltage_kp = 1.5;
double pid_voltage_ki = .2;
double pid_voltage_kd = .2;

PID voltage_regulator(&psu_out_voltage, &vreg_duty, &desired_voltage, pid_voltage_kp, pid_voltage_ki, pid_voltage_kd, P_ON_E, REVERSE);

unsigned long last_voltage_regulation = millis();
void regulate_voltage(){
  if (voltage_regulator.Compute()){
    //analogWrite(PSU_VOLTAGE_PIN, vreg_duty);
  } else {
    rdebugVln("Skipped voltage PID calculation.");
  }
  debugI("voltage_regulator freq: %.1f/s (%dms)",(double) (1000. / (millis() - last_voltage_regulation)), (uint16_t)(millis() - last_voltage_regulation));

  if ((/*voltage_regulator.GetSampleTime()*/ 50 * 2) < (millis() - last_voltage_regulation)){
    debugI("voltage_regulator not called often enough.");
  }

  
  last_voltage_regulation = millis();

}
/****************************************************** D I S P L A Y */


bool button_A_pressed(){
    return (digitalRead(BUTTON_A_PIN) == LOW);
}

bool button_B_pressed(){
    return (digitalRead(BUTTON_B_PIN) == LOW);
}

void display_text(String text){
  display.setTextSize(1);
  display.setCursor(0, 24);
  display.setTextColor(WHITE);
  display.clearDisplay();
  display.print(text);
  display.display();

}

long lastDisplayUpdate = millis();
void update_display() { 
  if (millis() - lastDisplayUpdate < 100) return;
  lastDisplayUpdate = millis();
  
  display.clearDisplay();

  display.setTextSize(2);
  display.setTextColor(WHITE);
  

  display.setCursor(0, 46 - 14);
  display.print(psu_out_voltage);
  display.setCursor(64 - 12, 46 - 14);
  display.print("V");
  
  display.setCursor(0, 46);
  display.print(psu_out_current);
  display.setCursor(64 - 12, 46);
  display.print("A");
  
  if (WiFi.status() == WL_CONNECTED) {
    display.setTextSize(1);
    display.setCursor(64-5, 8);
    display.print("W");
  } else {
    display.setTextSize(1);
    display.setCursor(64-5, 8);
    display.print("D");
  }

  display.setTextSize(1);
  display.setCursor(0, 8);



  switch (charger_state) {
    case CS_INIT:
      display.print("INIT");
      break;
    case CS_SELF_TEST:
      display.print("SELF_TEST");
      break;
    case CS_CALIBRATION:
      display.print("CALIBRATION");
      break;
    case CS_IDLE:
      display.print("IDLE");
      display.setTextSize(1);
      display.setCursor(0, 16);
      display.print(psu_spn);
      break;
    case CS_CC:
      display.print("CC");
      display.setTextSize(1);
      display.setCursor(0, 14);
      display.print(vreg_duty);
      display.setCursor(0, 22);
      display.print(psu_out_current);
      display.print("A");

      break;
    case CS_CV:
      display.print("CV");
      display.setTextSize(1);
      display.setCursor(0, 14);
      display.print(vreg_duty);
      break;
    case CS_DONE:
      display.print("DONE");
      break;
    case CS_ABORT:
      display.print("ABORT");
      display.setTextSize(1);
      display.setCursor(0, 14);
      display.print(display_error);
      break;
  }




  display.display();
  


}

/****************************************************** S E T U P */

void setup() {



  
  Serial.begin(115200);
  Serial.println("Start");


  pinMode(PSU_ENABLE_PIN, OUTPUT);
  digitalWrite(PSU_ENABLE_PIN, HIGH);
  //pinMode(PSU_VOLTAGE_PIN, INPUT);
  


  pinMode(BUTTON_A_PIN, INPUT);
  pinMode(BUTTON_B_PIN, INPUT);



  display_text("Connecting\nto WiFi...");
  reconnect_wifi();
 
  Debug.begin("dps1200FB");

  Debug.initDebugger(debugGetDebuggerEnabled, debugHandleDebugger, debugGetHelpDebugger, debugProcessCmdDebugger); // Set the callbacks

  debugInitDebugger(&Debug);


  ArduinoOTA.begin(); 
 
  
  String ip = WiFi.localIP().toString();
  debugI("IP address: %s", ip.c_str());
  

  
  reconnect_psu();
  force_psu_fan(3200);
  my_yield();
  display.begin(SSD1306_SWITCHCAPVCC, 0x3C);  // initialize with the I2C addr 0x3C (for the 64x48)
  display.clearDisplay();
  display.setFont(&Org_01);
  display.display();
  display_text(ip);

  

  current_regulator.SetSampleTime(50);
  current_regulator.SetOutputLimits(0, ANALOG_FREQ_RANGE);

  //analogWriteFreq(1024); //Slower the better? Less oscillations
  ////analogWriteRange(ANALOG_FREQ_RANGE);
  pinMode(PSU_VOLTAGE_PIN, INPUT);

}




/****************************************************** M A I N  L O O P */

void set_charger_state(charger_state_type cs){
  print_charger_state();
  debugI(" -> ");
  charger_state = cs;
  print_charger_state();
}



void charger_fsm(){
  //print_charger_state();

  debugI("%.3fV %.3fV %.3fA %.3fA %.2f", psu_out_voltage, desired_voltage, psu_out_current, desired_current, vreg_duty);

  charger_state_type cs_tmp = charger_state;
  switch (charger_state) {
    
    case CS_INIT:
      if (i2c_ping(psu_mem_addr) && i2c_ping(psu_mcu_addr)){
        read_eeprom();
        if (String(psu_spn).startsWith(psu_spn_supported)){
          //All good - init OK.
          set_charger_state(CS_SELF_TEST);
          break;
        } else {
          display_error=String(psu_spn) + String(" PSU not supported: ");
          set_charger_state(CS_ABORT);
        }

      } else {
        display_error = "PSU not found";
        set_charger_state(CS_ABORT);
      }
      break;

    case CS_SELF_TEST:
        set_charger_state(CS_IDLE);
        break;
      break;

    case CS_CALIBRATION:
        set_charger_state(CS_SELF_TEST);
        break;
      break;

    case CS_IDLE:
        if (charger_state_prev != charger_state){
          psu_disable();
        }
        if (button_B_pressed()){
          set_charger_state(CS_CC);
          break;
        }
     break;
    case CS_CC:
        if (charger_state_prev != charger_state){
          psu_enable();
          psu_settle();

          
          desired_current = battery.charge_current;
          while (!read_psu_data()) delay(10);
          current_regulator.SetMode(AUTOMATIC);
          
        } else {
            if (psu_out_voltage >= battery.max_voltage){
              current_regulator.SetMode(MANUAL);
              set_charger_state(CS_INIT); //CS_CV
              break;
            }

            regulate_current();
          

        }
        


      break;


    case CS_CV:
        if (charger_state_prev != charger_state){
          psu_enable();
          psu_settle();
          desired_voltage = battery.charge_voltage;
        } else {
          if (psu_out_current <= 0.1 /*battery.charge_voltage*/){
              set_charger_state(CS_DONE);
              break;
          }
          regulate_voltage();
          
        }


        
      break;
    case CS_DONE:
    case CS_ABORT:
        if (charger_state_prev != charger_state){
          psu_disable();
        }
        if (button_A_pressed()){
          set_charger_state(CS_INIT);
          break;
        }
      break;
  }
  charger_state_prev = cs_tmp;

}


void print_charger_state(){
  switch (charger_state) {
    case CS_INIT:
      debugI("CS_INIT");
      break;
    case CS_SELF_TEST:
      debugI("CS_SELF_TEST");
      break;
    case CS_CALIBRATION:
      debugI("CS_CALIBRATION");
      break;
    case CS_IDLE:
      debugI("CS_IDLE");
      break;
    case CS_CC:
      debugI("CS_CC");
      break;
    case CS_CV:
      debugI("CS_CV");
      break;
    case CS_DONE:
      debugI("CS_DONE");
      break;
    case CS_ABORT:
      debugI("CS_ABORT");
      break;
  }

}


void loop() {
  //i2c_bus_scan();
  my_yield();

  /*debugI("i2c SCA: %d SCL: %d", PSU_I2C_SDA, PSU_I2C_SCL);
  uint8_t u8;
  scan_for_device(0x00, 0x7F, u8);*/
  read_eeprom_string(0x12, 10, psu_spn);
  
  reconnect_psu();
  //read_eeprom();
  read_psu_data();
  charger_fsm();

  

  update_display();
}
  
