/*
required libs

> pio lib install 2 13 542 571 651

*/

#include "Arduino.h"
#include <WiFi.h>
#include <ArduinoOTA.h>

#include <PID_v1.h>

#include <SPI.h>
#include <Adafruit_GFX.h>
#include <Adafruit_ILI9341.h>
#include <XPT2046_Touchscreen.h>

/* text output */

WiFiServer TelnetServer(23);
WiFiClient Telnet;



#include <PrintEx.h>
StreamEx debugI = Telnet;



#include "wifi-config.h" //#define LOCAL_SSID and LOCAL_PSK for your network

#ifndef LOCAL_SSID
#error "LOCAL_SSID not defined. "
#endif

#ifndef LOCAL_PSK
#error "LOCAL_PSK not defined. "
#endif



/****************************************************** P I N S */

/*
1: +12V Power out
14: GND Power out
27, 28, 29: I2C slave address selection pins. 
30: GND (for I2C and I2C slave address selection, I guess)
31: I2C SCL
32: I2C SDA
33: ENABLE#
34: LOAD SHARE
35: STATUS
36: PRESENT
37: +12V stand-by
38: PSALARM
*/

#define PSU_PS_ALARM_PIN 34 
#define PSU_STATUS_PIN 15 
#define PSU_LOAD_SHARE 25
#define PSU_ENABLE_PIN 13
#define PSU_VOLTAGE_CONTROL_PIN 26
#define PSU_VOLTAGE_CONTROL_MAX 255

//ADC
#define PSU_OUT_VOLTAGE_PIN 36 //VP
#define BATTERY_VOLTAGE_PIN 39 //VN

//Touch screen
#define TFT_CS_PIN 14
#define TFT_DC_PIN 27
#define TFT_RST_PIN 33
#define TOUCH_CS_PIN 12


 //tune to your own voltage divider
#define PSU_OUT_VOLTAGE_CORRECTION_FACTOR 10.84368000
#define BATTERY_VOLTAGE_CORRECTION_FACTOR 10.82052848

//tune to voltage drop of charge leads
#define CHARGE_CURRENT_CORRECTION_FACTOR 85.90


 
 
Adafruit_ILI9341 display = Adafruit_ILI9341(TFT_CS, TFT_DC, TFT_RST);
XPT2046_Touchscreen touch(TS_CS);

/****************************************************** B A T T E R I E S */

struct Battery_type {
  double max_voltage;
  double charge_voltage;
  double max_current;
  double charge_current;
} typedef Battery_type;

Battery_type LiFePO4_1p1s = {4.2 * 1, 3.65 * 1, 15. * 1, 10. * 1};
Battery_type LiFePO4_1p2s = {4.2 * 2, 3.65 * 2, 15. * 1, 10. * 1};
Battery_type LiFePO4_8p2s = {4.2 * 2, 3.65 * 2, 15. * 8, 10. * 8};
Battery_type LiFePO4_1p4s = {4.2 * 4, 3.65 * 4, 15. * 1, 10. * 1};

Battery_type battery = LiFePO4_8p2s;



enum charger_state_type {CS_INIT=0, CS_IDLE, CS_SELF_TEST, CS_CALIBRATION, CS_CC, CS_CV, CS_DONE, CS_ABORT} typedef charger_state_type;

charger_state_type charger_state = CS_INIT;
charger_state_type charger_state_prev = CS_ABORT;

double vreg_dac;
double creg_dac;

double desired_voltage = .0;
double desired_current = .0;

double battery_voltage = 0.;
double psu_out_voltage = .0;
double psu_charge_current = .0;


void reconnect_wifi();
void display_text(String);
void update_display();
void handleTelnet();
double estimate_duty_from_voltage(double);
void print_charger_state();

String display_error = "";

/****************************************************** U T I L I T Y */



/****************************************************** H O U S E K E E P I N G */



long lastYield = millis();
void my_yield(){
  yield();
 
  //if (millis() - lastYield < 500) return;
  if (WiFi.status() != WL_CONNECTED) {  
    reconnect_wifi();
  }
  handleTelnet();
  ArduinoOTA.handle();
}


void reconnect_wifi() {  
    WiFi.mode(WIFI_STA);  
    WiFi.begin(LOCAL_SSID, LOCAL_PSK);  
    //debugI.printf("Connecting to: %s\r\n", LOCAL_SSID);
    /*IPAddress ip(10,0,0,167);   
    IPAddress gateway(10,0,0,1);   
    IPAddress subnet(255,255,255,0);   
    WiFi.config(ip, gateway, subnet);*/
    while (WiFi.status() != WL_CONNECTED) {  
      //debugI.println("Connecting to wifi...");
      delay(1000);
    }  
    String ip = WiFi.localIP().toString();
    //display_text("8");
    //debugI.printf("IP address: %s\n", ip.c_str());
    //display_text(ip.c_str());
    TelnetServer.begin();
    TelnetServer.setNoDelay(true);

    handleTelnet();
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

/******************************************** P S U   H A R D W A R E */


bool get_psu_is_enabled(){
  return (digitalRead(PSU_STATUS_PIN) == HIGH); //TODO: Check PS_ALARM as well
}


double get_pin_voltage_from_adc(uint16_t reading){
 //From github.com/G6EJD
 if (reading < 1 || reading > 4094) return 0;
 return -0.000000000000016 * pow(reading,4) + 0.000000000118171 * pow(reading,3)- 0.000000301211691 * pow(reading,2)+ 0.001109019271794 * reading + 0.034143524634089;
}



/*
double get_psu_out_voltage(){
  if (adcBusy(PSU_OUT_VOLTAGE_PIN) || adcBusy(BATTERY_VOLTAGE_PIN)){
    return psu_out_voltage;
  }
  
  uint16_t adc = adcEnd(PSU_OUT_VOLTAGE_PIN);
  adcStart(PSU_OUT_VOLTAGE_PIN);
  
  double r = get_pin_voltage_from_adc(adc) * PSU_OUT_VOLTAGE_CORRECTION_FACTOR;
  psu_out_voltage = r;
  return r;
}

double get_battery_voltage(){
  static double battery_voltage;
  if (adcBusy(PSU_OUT_VOLTAGE_PIN) || adcBusy(BATTERY_VOLTAGE_PIN)){
    return battery_voltage;
  }
  
  uint16_t adc = adcEnd(BATTERY_VOLTAGE_PIN);
  adcStart(BATTERY_VOLTAGE_PIN);
  
  double r = get_pin_voltage_from_adc(adc) * BATTERY_VOLTAGE_CORRECTION_FACTOR;
  battery_voltage = r;
  return r;
}
*/


double get_psu_out_voltage(){
  static double psu_out_voltage;
  static double counter = 1.;
  static const double factor = 1000.;
  static uint32_t last_read = millis();

  if (millis() - last_read < 10) return psu_out_voltage; //to cache and to make sure readings are somewhat regular.

  double f;

  uint16_t adc = analogRead(PSU_OUT_VOLTAGE_PIN);
  double r = get_pin_voltage_from_adc(adc) * PSU_OUT_VOLTAGE_CORRECTION_FACTOR;
  
  if (counter < factor) {
    f = counter;
    counter++;
  } else {
    f = factor;
  }

  if (abs(r - psu_out_voltage) > 2.){ //reset averaging on big diffs. This will really only happen in state changes.
    counter = 1.;
    f = counter;
  }

  psu_out_voltage = psu_out_voltage + (r - psu_out_voltage) / f;
  last_read = millis();
  return psu_out_voltage;
}

double get_battery_voltage(){
  static double counter = 1.;
  static const double factor = 25.;
  static uint32_t last_read = millis();

  if (millis() - last_read < 10) return battery_voltage; //to cache and to make sure readings are somewhat regular.

  double f;

  uint16_t adc = analogRead(BATTERY_VOLTAGE_PIN);
  if (adc < 1 || adc > 4094) return battery_voltage;
  double r = get_pin_voltage_from_adc(adc) * BATTERY_VOLTAGE_CORRECTION_FACTOR;
  /*
  if (counter < factor) {
    f = counter;
    counter++;
  } else {
    f = factor;
  }
  
  if ((abs(r - battery_voltage) > 1.) && (counter >= factor)){ //reset averaging on big diffs. This will really only happen in state changes.
    debugI.printf("Resetting battery voltage averaging. %.2fV %.2fV\n", r, battery_voltage);
    counter = 1.;
    f = counter;
    battery_voltage = r;
  }
  
  battery_voltage += (r - battery_voltage) / f;
  */
  battery_voltage = r;
  last_read = millis();
  return battery_voltage;
}

double get_charge_current(){
  double vp = get_psu_out_voltage();
  double vb = get_battery_voltage();
  if (vb > vp){
    return 0.;
  }

  double c = (vp - vb) * CHARGE_CURRENT_CORRECTION_FACTOR;

  psu_charge_current = c;
  return c;

}

void psu_enable(){
  debugI.print("psu_enable\n");
  
  vreg_dac = 0xFF;
  while(!get_psu_is_enabled()){

    vreg_dac -= 0x0F;
    digitalWrite(PSU_ENABLE_PIN, HIGH);
    for (uint16_t i = 0; i<0x2F; i++){
      my_yield();
      update_display();      
    }
    
    pinMode(PSU_VOLTAGE_CONTROL_PIN, INPUT);
    digitalWrite(PSU_ENABLE_PIN, LOW);
    for (uint16_t i = 0; i<0x2F; i++){
      my_yield();
      update_display();      
    }

    pinMode(PSU_VOLTAGE_CONTROL_PIN, OUTPUT);
    dacWrite(PSU_VOLTAGE_CONTROL_PIN, vreg_dac);
        
    for (uint16_t i = 0; i<0x2F; i++){
      my_yield();
      update_display();      
    }


  }
}

void psu_settle(){


  double last_voltage = get_psu_out_voltage();
  my_yield();
  delay(250);
  double voltage = get_psu_out_voltage();
  while (abs(voltage - last_voltage) > 0.02){
    
    voltage = get_psu_out_voltage();
    last_voltage = last_voltage + .25 * (voltage - last_voltage); //running average
    delay(5);
    debugI.printf("Waiting for voltage to stabilize. %.3fV\n", voltage);
    my_yield();
    update_display();
  }

  desired_voltage = voltage;
  debugI.printf("Stabilized voltage: %.3fV\n", voltage);
  
  vreg_dac = estimate_duty_from_voltage(voltage)  /* - margin*/;
  
  dacWrite(PSU_VOLTAGE_CONTROL_PIN, vreg_dac);
}


void psu_disable(){
  debugI.printf("psu_disable\n");
  pinMode(PSU_VOLTAGE_CONTROL_PIN, INPUT);
  digitalWrite(PSU_ENABLE_PIN, HIGH);
}


double estimate_duty_from_voltage(double voltage){
  return PSU_VOLTAGE_CONTROL_MAX * 2.5 / 3.3 * 12 / voltage;
  /*double duty;
  //Simple linear interpolation. Maybe a lookup would be more exact
  double step_per_volt = ((double)PSU_VOLTAGE_CONTROL_MAX - 0.) / (13.7 - 2.5);
  duty = PSU_VOLTAGE_CONTROL_MAX - step_per_volt * voltage;

  return duty;*/
}

double pid_current_kp = 0.1;
double pid_current_ki = 0.1;
double pid_current_kd = 0.1;

PID current_regulator(&psu_charge_current, &creg_dac, &desired_current, pid_current_kp, pid_current_ki, pid_current_kd, P_ON_E, REVERSE);

unsigned long last_current_regulation = millis();
void regulate_current(){
  if (current_regulator.Compute()){
    dacWrite(PSU_VOLTAGE_CONTROL_PIN, creg_dac);
  } else {
    debugI.printf("Skipped current PID calculation.\n");
  }
  debugI.printf("current_regulator freq: %.1f/s (%dms)\n",(double) (1000. / (millis() - last_current_regulation)), (uint16_t)(millis() - last_current_regulation));
  if ((/*current_regulator.GetSampleTime()*/ 50 * 2) < (millis() - last_current_regulation)){
    debugI.printf("current_regulator not called often enough.\n");
  }

  
  last_current_regulation = millis();

}

double pid_voltage_kp = 10;
double pid_voltage_ki = 1;
double pid_voltage_kd = 1;

PID voltage_regulator(&battery_voltage, &vreg_dac, &desired_voltage, pid_voltage_kp, pid_voltage_ki, pid_voltage_kd, P_ON_E, DIRECT);


void regulate_voltage(){
  if (voltage_regulator.Compute()){
    dacWrite(PSU_VOLTAGE_CONTROL_PIN, vreg_dac);
  } 
}
/****************************************************** D I S P L A Y */


uint8_t text_height = 8 * 2;
uint8_t text_width = 6;
uint16_t display_height = 240;
uint16_t display_width = 320;
uint8_t display_text_row = 0;



void display_print(String string){
  /*int16_t  x1, y1;
  uint16_t w, h;
  uint16_t x = display.getCursorX();
  uint16_t y = display.getCursorY();
  display.getTextBounds(string + "_", x, y, &x1, &y1, &w, &h);
  display.fillRect(x1,y1,w,h, ILI9341_BLACK);*/
  display.setTextColor(ILI9341_WHITE, ILI9341_BLACK);
  display.print(string);
}

void display_print(double d){
  display_print(String(d, 3));
}

void display_print(int i){
  display_print(String(i));
}

void display_print(long unsigned int i){
  display_print(String(i));
}

void display_text(String text){
  if (display_text_row == 0) {
    display.fillScreen(ILI9341_BLACK);
  }
  display.setTextSize(1);
  display.setCursor(0, text_height + display_text_row * text_height);
  display.setTextColor(ILI9341_WHITE);
  display_print(text);
  display_text_row++;
  if (display_text_row * text_height > display_height) {
    display_text_row = 0;
  }
}

long lastDisplayUpdate = millis();
void update_display() { 

  //if (millis() - lastDisplayUpdate < 100) return;
  lastDisplayUpdate = millis();


  //display.fillScreen(ILI9341_BLACK);

  display.setTextSize(2);
  

  display.setCursor(0, 2 * text_height);
  display.setTextColor(ILI9341_WHITE);
  display_print("PSU out: ");
  display_print(get_psu_out_voltage());
  display_print("V ");
  
  display.setCursor(0, 3 * text_height);
  display.setTextColor(ILI9341_WHITE);
  display_print("Battery: ");
  display_print(get_battery_voltage());
  display_print("V ");
  
  display.setCursor(0, 4 * text_height);
  display.setTextColor(ILI9341_WHITE);
  display_print("Set V: ");
  display_print(desired_voltage);
  display_print("V ");
  

  display.setCursor(0, 5 * text_height);
  display_print("Charge: ");
  display_print(get_charge_current());
  display_print("A ");

  display.setCursor(0, 12 * text_height);
  display_print(millis());
  display_print("ms ");

  display.setCursor(0, 8 * text_height);
  display_print("creg: ");
  display_print(creg_dac);
  display_print(" ");
  
  display.setCursor(0, 9 * text_height);
  display_print("vreg: ");
  display_print(vreg_dac);
  display_print(" ");

  display.setCursor(display_width - 2 * text_width, text_height);
  if (WiFi.status() == WL_CONNECTED) {
    display_print("W");
  } else {
    display_print("D");
  }



  display.setCursor(0, text_height);



  switch (charger_state) {
    case CS_INIT:
      display_print("INIT");
      break;
    case CS_SELF_TEST:
      display_print("SELF_TEST");
      break;
    case CS_CALIBRATION:
      display_print("CALIBRATION");
      break;
    case CS_IDLE:
      display_print("IDLE");
      break;
    case CS_CC:
      display_print("CC");
      /*display.setTextSize(1);
      display.setCursor(0, text_height);
      display_print(vreg_dac);
      display.setCursor(0, 22);
      display_print(psu_charge_current);
      display_print("A");
*/
      break;
    case CS_CV:
      display_print("CV");
/*      display.setTextSize(1);
      display.setCursor(0, text_height);
      display_print(vreg_dac);*/
      break;
    case CS_DONE:
      display_print("DONE");
      break;
    case CS_ABORT:
      display_print("ABORT");
      /*display.setTextSize(1);
      display.setCursor(0, text_height);
      display_print(display_error);*/
      break;
  }

  String ip = WiFi.localIP().toString();
  ip = String(" - ") + ip;
  display_print(ip);  
}


/****************************************************** S E T U P */

void setup() {



  
  Serial.begin(115200);
  Serial.println("Start");


  pinMode(PSU_ENABLE_PIN, OUTPUT);
  digitalWrite(PSU_ENABLE_PIN, HIGH);
  //pinMode(PSU_VOLTAGE_CONTROL_PIN, INPUT);
  pinMode(PSU_STATUS_PIN, INPUT);

  analogSetClockDiv(255);
  pinMode(BATTERY_VOLTAGE_PIN, INPUT);
  pinMode(PSU_OUT_VOLTAGE_PIN, INPUT);
  adcAttachPin(PSU_OUT_VOLTAGE_PIN);
  adcAttachPin(BATTERY_VOLTAGE_PIN);
  
  touch.begin();
  touch.setRotation(3);
 
  display.begin();
  display.setRotation(3);
  display.fillScreen(ILI9341_BLACK);
  display.setTextSize(1);
  //display.setFont(&FreeMono12pt7b);
  display.setTextColor(ILI9341_WHITE);



  //display_text("Connecting to WiFi...");
  reconnect_wifi();
  String ip = WiFi.localIP().toString();
  debugI.printf("IP address: %s\n", ip.c_str());

  my_yield();

  ArduinoOTA.begin(); 
  current_regulator.SetSampleTime(50);
  current_regulator.SetOutputLimits(0, PSU_VOLTAGE_CONTROL_MAX);

  //analogWriteFreq(1024); //Slower the better? Less oscillations
  ////analogWriteRange(PSU_VOLTAGE_CONTROL_MAX);
  pinMode(PSU_VOLTAGE_CONTROL_PIN, INPUT);

}




/****************************************************** M A I N  L O O P */

void set_charger_state(charger_state_type cs){
  print_charger_state();
  debugI.printf(" -> ");
  charger_state = cs;
  print_charger_state();
  display.fillScreen(ILI9341_BLACK);
}



void charger_fsm(){
  //print_charger_state();

  //debugI.printf("PSU: %.3fV Battery: %.3fV Setpoint: %.3fV  Charge: %.3fA Setpoint: %.3fA vreg: %.2f creg: %.2f\n", get_psu_out_voltage(), get_battery_voltage(), desired_voltage, get_charge_current(), desired_current, vreg_dac, creg_dac);

  charger_state_type cs_tmp = charger_state;
  switch (charger_state) {
    
    case CS_INIT:
        set_charger_state(CS_SELF_TEST);
      break;

    case CS_SELF_TEST:
        set_charger_state(CS_IDLE);
      break;

    case CS_CALIBRATION:
        set_charger_state(CS_SELF_TEST);
      break;

    case CS_IDLE:
        if (charger_state_prev != charger_state){
          psu_disable();
        }
        if (touch.touched()){
          set_charger_state(CS_CV);
          break;
        }
      break;

/*
    case CS_CC:
        set_charger_state(CS_CV);
        

        if (charger_state_prev != charger_state){
          psu_enable();
          psu_settle();

          
          desired_current = battery.charge_current;
          current_regulator.SetMode(AUTOMATIC);
          
        } else {
            if (get_psu_out_voltage() >= battery.max_voltage){
              current_regulator.SetMode(MANUAL);
              set_charger_state(CS_CV);
              break;
            }
            
            if (touch.touched()){
              current_regulator.SetMode(MANUAL);
              set_charger_state(CS_INIT);
            }

            if (!get_psu_is_enabled()){
              current_regulator.SetMode(MANUAL);
              set_charger_state(CS_ABORT);
            }

            regulate_current();
 
        }
        


      break;
  */

    case CS_CV:


        if (charger_state_prev != charger_state){
          psu_enable();
          psu_settle();

          
          desired_voltage = battery.charge_voltage;
          voltage_regulator.SetMode(AUTOMATIC);
          
        } else {
            if (psu_charge_current <= 0.1 /*battery.charge_voltage*/){
              voltage_regulator.SetMode(MANUAL);
              set_charger_state(CS_DONE);
              break;
            }
            
            if (touch.touched()){
              voltage_regulator.SetMode(MANUAL);
              set_charger_state(CS_INIT);
            }

            if (!get_psu_is_enabled()){
              voltage_regulator.SetMode(MANUAL);
              set_charger_state(CS_ABORT);
            }
 
        }
 

       
        regulate_voltage();
          
        
      break;
    case CS_DONE:
    case CS_ABORT:
        if (charger_state_prev != charger_state){
          psu_disable();
        }
        if (touch.touched()){
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
      debugI.printf("CS_INIT\n");
      break;
    case CS_SELF_TEST:
      debugI.printf("CS_SELF_TEST\n");
      break;
    case CS_CALIBRATION:
      debugI.printf("CS_CALIBRATION\n");
      break;
    case CS_IDLE:
      debugI.printf("CS_IDLE\n");
      break;
    case CS_CC:
      debugI.printf("CS_CC\n");
      break;
    case CS_CV:
      debugI.printf("CS_CV\n");
      break;
    case CS_DONE:
      debugI.printf("CS_DONE\n");
      break;
    case CS_ABORT:
      debugI.printf("CS_ABORT\n");
      break;
  }

}


void loop() {
  my_yield();
  charger_fsm();
  my_yield();
  update_display();
}
  
