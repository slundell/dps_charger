/*
required libs

>pio lib install 2
PID @ 1.2.0 has been successfully installed!

>pio lib install 13
Adafruit GFX Library @ 1.5.6 has been successfully installed!

>pio lib install 2104
Adafruit SSD1306 Wemos Mini OLED @ 1.1.2 has been successfully installed!
*/


#include <ESP8266WiFi.h>
#include <ArduinoOTA.h>
#include <SPI.h>
#include <Wire.h>
#include <PID_v1.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>
//#include <Fonts/Picopixel.h>
#include <Fonts/Org_01.h>
#define OLED_RESET 0  // GPIO0
Adafruit_SSD1306 display(OLED_RESET);

#define ANALOG_FREQ_RANGE 1023

#include "./stassid.h" //#define STASSID and STAPSK for your network


struct Battery_type {
  double max_voltage;
  double charge_voltage;
  double max_current;
  double charge_current;
} typedef Battery_type;

Battery_type LiFePO4_1s = {4.2, 3.65, 50., 25.};

Battery_type battery = LiFePO4_1s;

enum charger_state_type {CS_INIT=0, CS_IDLE, CS_SELF_TEST, CS_CALIBRATION, CS_CC, CS_CV, CS_DONE, CS_ABORT} typedef charger_state_type;

charger_state_type charger_state = CS_INIT;
charger_state_type charger_state_prev = CS_ABORT;

char psu_spn_supported[] = "441830-001";

char psu_spn[11];
char psu_date[9];
char psu_name[27];
char psu_mfu[5];
char psu_ct[15];

WiFiServer TelnetServer(23);
WiFiClient Telnet;

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

/*double voltage_controller_Kp = 40;
double voltage_controller_Ki = 2.;
double voltage_controller_Kd = .01;

PID voltage_controller(&psu_out_voltage, &vreg_duty, &desired_voltage, voltage_controller_Kp, voltage_controller_Ki, voltage_controller_Kd, P_ON_E, REVERSE);
*/



#define PSU_ENABLE_PIN D0
#define PSU_VOLTAGE_PIN D5
#define BUTTON_A_PIN D3
#define BUTTON_B_PIN D4

/****************************************************** U T I L I T Y */

void print_byte(uint8_t b){
  Telnet.print("0x");
  if (b < 16)
    Telnet.print("0");
  Telnet.print(b, HEX);
}

    

/****************************************************** i 2 C - f u n c t i o n s */


void scan_for_device(uint8_t from, uint8_t to, uint8_t& ret) {
  byte address;
  
  
  for (address = from ; address <= to; address++ )  {
    Wire.beginTransmission(address);
    uint8_t ack = Wire.endTransmission();
    if (ack == 0){
      Telnet.print("Device found at ");
      print_byte(address);
      Telnet.println();
      ret = address;
      return;
    } 
  }

  if (ret == 0xFF) {
    Telnet.print("No device found between ");
    print_byte(from);
    Telnet.print(" and ");
    print_byte(to);
    Telnet.println(".");
  }
  
}


bool i2c_ping(uint8_t addr) {
  Wire.beginTransmission(addr);
  uint8_t ack = Wire.endTransmission();
  return (ack == 0);
}
      


/****************************************************** H O U S E K E E P I N G */



long lastYield = millis();
void my_yield(){
  yield();
  if (millis() - lastYield < 500) return;
  if (WiFi.status() != WL_CONNECTED) {  
    reconnect_wifi();
  }
  handleTelnet();
  ArduinoOTA.handle();
}


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

void reconnect_wifi() {  
    WiFi.mode(WIFI_STA);  
    WiFi.begin(STASSID, STAPSK);  
    while (WiFi.status() != WL_CONNECTED) {  
      Serial.println("Connecting to wifi...");
        delay(500);  
    }  
}  


void reconnect_psu(){
  
  Wire.begin(D2 /*SDA*/, D1 /*SCL*/);  
  display_text("Connecting to PSU");

  while (psu_mem_addr == 0xFF){
    Telnet.println("Scanning for EEPROM.");
    scan_for_device(0x50, 0x57, psu_mem_addr);
    my_yield();
  }
  read_eeprom();

  while (psu_mcu_addr == 0xFF){
    Telnet.println("Scanning for MCU.");
    scan_for_device(0x58, 0x5F, psu_mcu_addr);
    my_yield();
  }
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
  Wire.beginTransmission(psu_mem_addr);

  Wire.write((uint8_t)(addr));
  Wire.endTransmission();

  Wire.requestFrom((uint8_t)psu_mem_addr, (uint8_t)len);


  uint8_t i=0;
  while (Wire.available() && (i <= len)){
    s[i] = Wire.read();
    i++;
  }
  s[i] = '\0';
}

void read_eeprom(){

  read_eeprom_string(0x12, 10, psu_spn);
  read_eeprom_string(0x1D, 8, psu_date);
  read_eeprom_string(0x32, 26, psu_name);
  read_eeprom_string(0x5B, 14, psu_ct);
  
  Telnet.print("Found: ");
  Telnet.println(psu_name);

}


/****************************************************** P S U  M C U */


bool checksum(uint8_t* msg, uint8_t msg_len){

  uint8_t cs = 0;
  for (uint8_t i=0; i<msg_len; ++i){
    cs += msg[i];
  }
  cs = (( 0xFF - cs) + 1) & 0xFF;
  if (cs) { 
    Telnet.print("Wrong checksum: ");
    Telnet.println(cs);
  }
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
    Wire.write(calculate_checksum_u16(reg, 0));
    Wire.endTransmission(true);

    Wire.requestFrom((uint8_t)psu_mcu_addr, (size_t)len, false); //casting to avoid warnings
    
    uint8_t i = 0;
    
    while (Wire.available() && (i < len)){ 
      msg[i] = Wire.read(); 
      /*print_byte(msg[i]);
      Telnet.println();*/
      i++;
    }
    if (i!=len){
      Telnet.print("Expected to read ");
      Telnet.print(len);
      Telnet.print(" bytes, but got ");
      Telnet.print(i);
      Telnet.println(" bytes.");
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
  bool b = (read_psu_mcu_f16(0x10, 1./128, ret) );
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
  Telnet.print(" ");
  print_byte(u16 & 0xFF);
  Telnet.println();*/
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

double abs(double d){
  if (d<0) return -d;
  return d;
}

void psu_enable(){
  Telnet.println("psu_enable");
  pinMode(PSU_VOLTAGE_PIN, OUTPUT);
  digitalWrite(PSU_VOLTAGE_PIN, HIGH);
  delay(1);
  digitalWrite(PSU_ENABLE_PIN, LOW);

  while(
    (!read_psu_is_enabled(psu_is_enabled)) || //not able to read PSU
    (!psu_is_enabled)) // OR psu is not enabled
    {
    Telnet.print("PSU Enabling: ");
    read_psu_out_voltage(psu_out_voltage);
    Telnet.print(psu_out_voltage);
    Telnet.println("V");
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
    Telnet.println("Waiting for voltage to stabilize");
    Telnet.print(psu_out_voltage);
    Telnet.println("V");
    my_yield();
    update_display();
  }
  desired_voltage = psu_out_voltage;
  Telnet.println("Stabilized voltage:");
  Telnet.print(psu_out_voltage);
  Telnet.println("V");
  vreg_duty = estimate_duty_from_voltage(psu_out_voltage)  /* - margin*/;
  pinMode(PSU_VOLTAGE_PIN, OUTPUT);
  analogWrite(PSU_VOLTAGE_PIN, vreg_duty);
}


void psu_disable(){
  Telnet.println("psu_disable");
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

double pid_current_kp = 1.5;
double pid_current_ki = .2;
double pid_current_kd = .2;

PID current_regulator(&psu_out_voltage, &vreg_duty, &desired_current, pid_current_kp, pid_current_ki, pid_current_kd, P_ON_E, REVERSE);

unsigned long last_current_regulation = millis();
void regulate_current(){
  if (current_regulator.Compute()){
    analogWrite(PSU_VOLTAGE_PIN, vreg_duty);
  } else {
    Telnet.println("Skipped current PID calculation.");
  }
  Telnet.print("current_regulator freq: ");
  Telnet.print((double) (1000. / (millis() - last_current_regulation)));  
  Telnet.print("/s ");
  Telnet.print((millis() - last_current_regulation));  
  Telnet.println("ms");

  if ((/*current_regulator.GetSampleTime()*/ 50 * 2) < (millis() - last_current_regulation)){
    Telnet.println("current_regulator not called often enough.");
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
    analogWrite(PSU_VOLTAGE_PIN, vreg_duty);
  } else {
    Telnet.println("Skipped voltage PID calculation.");
  }
  Telnet.print("voltage_regulator freq: ");
  Telnet.print((double) (1000. / (millis() - last_voltage_regulation)));  
  Telnet.print("/s ");
  Telnet.print((millis() - last_voltage_regulation));  
  Telnet.println("ms");

  if ((/*voltage_regulator.GetSampleTime()*/ 50 * 2) < (millis() - last_voltage_regulation)){
    Telnet.println("voltage_regulator not called often enough.");
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
  if (millis() - lastDisplayUpdate < 250) return;
  lastDisplayUpdate = millis();
  display.setTextSize(2);
  display.setTextColor(WHITE);
  display.clearDisplay();

  display.setCursor(0, 46 - 14);
  display.print(psu_out_voltage);
  display.setCursor(64 - 12, 46 - 14);
  display.print("V");
  
  display.setCursor(0, 46);
  display.print(psu_out_current);
  display.setCursor(64 - 12, 46);
  display.print("A");

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
      break;
  }




  display.display();
  

  /*display.startscrollright(0x00, 0x01);
  delay(200);
  display.stopscroll();*/
}

/****************************************************** S E T U P */

void setup() {

  display.begin(SSD1306_SWITCHCAPVCC, 0x3C);  // initialize with the I2C addr 0x3C (for the 64x48)
  display.clearDisplay();
  display.setFont(&Org_01);
  display.display();
  
  Serial.begin(115200);
  Serial.println("Start");


  pinMode(PSU_ENABLE_PIN, OUTPUT);
  digitalWrite(PSU_ENABLE_PIN, HIGH);
  //pinMode(PSU_VOLTAGE_PIN, INPUT);
  


  pinMode(BUTTON_A_PIN, INPUT);
  pinMode(BUTTON_B_PIN, INPUT);


  display_text("Connecting\nto WiFi...");
  reconnect_wifi();
 
  ArduinoOTA.begin(); 
 
  TelnetServer.begin();
  TelnetServer.setNoDelay(true);
  Telnet.println("Ready");
  Telnet.println("IP address: ");
  String ip = WiFi.localIP().toString();
  Telnet.println(ip);
  Serial.println(ip);
  display_text(ip);

  reconnect_psu();
  my_yield();
  
  force_psu_fan(3200);

  current_regulator.SetSampleTime(50);
  current_regulator.SetOutputLimits(0, ANALOG_FREQ_RANGE);

  analogWriteFreq(1); //Slower the better? Less oscillations
  //analogWriteRange(ANALOG_FREQ_RANGE);
  pinMode(PSU_VOLTAGE_PIN, INPUT);

}




/****************************************************** M A I N  L O O P */

void set_charger_state(charger_state_type cs){
  print_charger_state();
  Telnet.print(" -> ");
  charger_state = cs;
  print_charger_state();
}



void charger_fsm(){
  //print_charger_state();

  Telnet.print(desired_voltage);
  Telnet.print("V ");
  Telnet.print(psu_out_voltage);
  Telnet.print("V ");
  Telnet.print(desired_current);
  Telnet.print("A ");
  Telnet.print(psu_out_current);
  Telnet.print("A ");
  Telnet.print(vreg_duty);
  Telnet.println("");

  charger_state_type cs_tmp = charger_state;
  switch (charger_state) {
    
    case CS_INIT:
      if (i2c_ping(psu_mem_addr) && i2c_ping(psu_mcu_addr)){
        read_eeprom();
        if (strcmp(psu_spn, psu_spn_supported) == 0){
          //All good - init OK.
          set_charger_state(CS_SELF_TEST);
          break;
        }

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
          current_regulator.SetMode(AUTOMATIC);
          
        } else {
            if (psu_out_voltage >= battery.max_voltage){
              current_regulator.SetMode(MANUAL);
              set_charger_state(CS_CV);
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
      Telnet.println("CS_INIT");
      break;
    case CS_SELF_TEST:
      Telnet.println("CS_SELF_TEST");
      break;
    case CS_CALIBRATION:
      Telnet.println("CS_CALIBRATION");
      break;
    case CS_IDLE:
      Telnet.println("CS_IDLE");
      break;
    case CS_CC:
      Telnet.println("CS_CC");
      break;
    case CS_CV:
      Telnet.println("CS_CV");
      break;
    case CS_DONE:
      Telnet.println("CS_DONE");
      break;
    case CS_ABORT:
      Telnet.println("CS_ABORT");
      break;
  }

}


void loop() {
  my_yield();
  read_psu_data();
  charger_fsm();
  update_display();
}
  
