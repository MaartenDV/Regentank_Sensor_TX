//COM 64
//arduino MEGA


//************ INCLUDES ************
#include <RH_ASK.h>
#ifdef RH_HAVE_HARDWARE_SPI
  #include <SPI.h> // Not actually used but needed to compile
#endif
#include <Ultrasonic.h>
#include <OneWire.h>
#include <DallasTemperature.h>



//************ DEFINES ************
#define PIN_ANALOG_BAT_V  A0
#define ONE_WIRE_BUS      2
#define PIN_READ_BAT_V    3
#define PIN_RX            3   //unused
#define PIN_PS_EN         4
#define PIN_FULL_LED      5
#define PIN_75_LED        6
#define PIN_50_LED        7  
#define PIN_TX            8   
#define PIN_TRIG          9
#define PIN_ECHO          10
#define PIN_25_LED        11
#define PIN_EMPTY_LED     12

#define VOLUME_MEASUREMENT_MILI_DELAY           5000//36000000    //=1h
#define TEMPERATURE_MEASUREMENT_MILI_DELAY      60000       //1min
#define LED_ANIMATION_MILI_DELAY                200         //0.200s

#define PACKET_BYTE_LENGTH    5
#define PACKET_REV            1
#define TASK_DELAY_MS         3000

//calibration params
#define A   1.1938
#define B   4.058

#define OUTFLOW_HEIGHT_MM           1500    //max water height
#define MUD_THICKNESS_MM            100     //mud in well height
#define SENSOR_HEIGHT_MM            1700    //height sensor above empty well
#define ULTRASONIC_DEPTH_SAMPLES    32

#define LEVEL_10_PERCENT            10
#define LEVEL_25_PERCENT            25  
#define LEVEL_50_PERCENT            50
#define LEVEL_75_PERCENT            75
#define LEVEL_90_PERCENT            90
#define LEVEL_100_PERCENT           100



//************ VARIABLES ************
RH_ASK driver(2000, PIN_RX, PIN_TX, 0); // ATTiny, RX on D3 (pin 2 on attiny85) TX on D4 (pin 3 on attiny85), 
String unit = "mm";
Ultrasonic ultrasonicSensor(PIN_TRIG, PIN_ECHO, unit);
OneWire oneWire(ONE_WIRE_BUS);
DallasTemperature sensors(&oneWire);

unsigned long volume_task_millis = 0;
unsigned long temperature_task_millis = 0;
unsigned long led_task_millis = 0;

word ultrasonic_depth; //unsigned 16bit
word rain_water_volume_percent;
int ds18b20_temperature; //signed 16 bit
float battery_voltage;
byte tx_packet[PACKET_BYTE_LENGTH];
bool bool_led_blink;

//******* FUNCTION DISCIPTION *******
void do_volume_measurement_task(void);
void do_temperature_measurement_task(void);
void do_battery_voltage_measurement_task(void);
void do_led_animation_task(void);
void ps_enable(void);
void ps_disable(void);
void ps_enabled_after_mili_delay(unsigned long);
void vbat_measure_enable(void);
void vbat_measure_disable(void);
word calculate_volume_percent(word, word, word, word);



// ***************************************
//                 SETUP
//****************************************
void setup() {
  
  //KEEPOUT START
  #ifdef RH_HAVE_SERIAL
      Serial.begin(9600);    // Debugging only
  #endif
      if (!driver.init())
  #ifdef RH_HAVE_SERIAL
           Serial.println("init failed");
  #else
    ;
  #endif

  sensors.begin();
  //KEEPOUT END

  pinMode(PIN_PS_EN, OUTPUT);
  pinMode(PIN_EMPTY_LED, OUTPUT);
  pinMode(PIN_25_LED, OUTPUT);
  pinMode(PIN_50_LED, OUTPUT);
  pinMode(PIN_75_LED, OUTPUT);
  pinMode(PIN_FULL_LED, OUTPUT);
  pinMode(PIN_READ_BAT_V, OUTPUT);

  volume_task_millis = millis();
  temperature_task_millis = millis();
  led_task_millis = millis();
  rain_water_volume_percent = 0;
}



// ***************************************
//                  MAIN
//****************************************
void loop() {

  // volume meter task
  if ((millis() - volume_task_millis) >=  VOLUME_MEASUREMENT_MILI_DELAY) {
    do_volume_measurement_task();
    volume_task_millis = millis();
  }
  
  // temperature task
  if ((millis() - temperature_task_millis) >=  TEMPERATURE_MEASUREMENT_MILI_DELAY) {
    do_temperature_measurement_task();
    do_battery_voltage_measurement_task();
    temperature_task_millis = millis();
  }
  
  //led animation task
  if ((millis() - led_task_millis) >=  LED_ANIMATION_MILI_DELAY) {
    do_led_animation_task();
    led_task_millis = millis();
  }
}
  

// ***************************************
//             TASKS volume
//****************************************
void do_volume_measurement_task() {
  ps_enabled_after_mili_delay(25);
  Serial.print("The Distance is : ");
  ultrasonic_depth = ultrasonicSensor.getDistance();
  ultrasonic_depth = (A*ultrasonic_depth + B); 
  Serial.print(ultrasonic_depth);
  Serial.println(" mm");
  Serial.print("The Volume is : ");
  rain_water_volume_percent = calculate_volume_percent(SENSOR_HEIGHT_MM, ultrasonic_depth, MUD_THICKNESS_MM, OUTFLOW_HEIGHT_MM);
  Serial.print(rain_water_volume_percent);
  Serial.println(" %");
  ps_disable();
}

// ***************************************
//           TASKS temperature
//****************************************
void do_temperature_measurement_task() {
  ps_enabled_after_mili_delay(25);
  unsigned long temp_measure_current_millis = millis();
  Serial.print("The Temperature is : ");
  sensors.requestTemperatures(); // Send the command to get temperatures
  ds18b20_temperature = (int)(sensors.getTempCByIndex(0) * 100);
  // Check if reading was successful
  //if(tempC != DEVICE_DISCONNECTED_C) {
    //nothing
  //} else {
    //ds18b20_temperature = 0xFFFF;
    //fault!
  //}
  Serial.print((ds18b20_temperature / 100));
  Serial.println(" °C");
  Serial.print("Time to receive one wire temp :");
  Serial.print(millis() - temp_measure_current_millis);
  Serial.println(" ms");
  ps_disable();
}

// ***************************************
//             TASKS led-a
//****************************************
void do_led_animation_task() {
  bool_led_blink = !bool_led_blink;     //used for the blining animation
  digitalWrite(PIN_EMPTY_LED, (rain_water_volume_percent < LEVEL_10_PERCENT));
  digitalWrite(PIN_EMPTY_LED, (((rain_water_volume_percent >= LEVEL_10_PERCENT) && (rain_water_volume_percent < LEVEL_25_PERCENT) && (bool_led_blink))));  
  digitalWrite(PIN_25_LED, (rain_water_volume_percent >= LEVEL_25_PERCENT));
  digitalWrite(PIN_50_LED, (rain_water_volume_percent >= LEVEL_50_PERCENT));
  digitalWrite(PIN_75_LED, (rain_water_volume_percent >= LEVEL_75_PERCENT));
  digitalWrite(PIN_FULL_LED, (rain_water_volume_percent >= LEVEL_100_PERCENT));
}


// ***************************************
//             TASKS bat-v
//****************************************
void do_battery_voltage_measurement_task() {
  vbat_measure_enable();  //put before next function
  ps_enabled_after_mili_delay(25);
  Serial.print("The battery voltage is : ");
  battery_voltage = 0.024 * (analogRead(PIN_ANALOG_BAT_V));
  Serial.print(battery_voltage, 2);
  Serial.println(" V");
  vbat_measure_disable();
  ps_disable();
}


// ***************************************
//            basic FUNCTIONS
//****************************************
// basic function to enable the power 5V switched power supply
void ps_enable() {
  digitalWrite(PIN_PS_EN, LOW);
}

// basic function to disable the power 5V switched power supply
void ps_disable() {
  digitalWrite(PIN_PS_EN, HIGH);
}

// basic function to enable the power 5V switched power supply after a given dalay
void ps_enabled_after_mili_delay(unsigned long mili_delay) {
    unsigned long temp_current_Millis = millis();
    while ((temp_current_Millis + mili_delay) >= millis()) {
      //just wait
    }
  digitalWrite(PIN_PS_EN, LOW);
}

//basic function to enable the battery voltage readout
void vbat_measure_enable() {
  digitalWrite(PIN_READ_BAT_V, HIGH);
}

//basic function to disable the battery voltage readout
void vbat_measure_disable() {
  digitalWrite(PIN_READ_BAT_V, LOW);
}

//function to calculate the percent volume in the rain water tank
word calculate_volume_percent(word sensor_h_mm, word distance_h_mm, word mud_h_mm, word outflow_h_mm) {
  //todo checks
  return ((sensor_h_mm - distance_h_mm) - mud_h_mm) / (outflow_h_mm / 100);
}


//todo - transport data using the RF link
/*
    //prepare for transport
    tx_packet[0] = PACKET_REV;
    tx_packet[1] = ((ultrasonic_depth >> 8) & 0xFF);
    tx_packet[2] = (ultrasonic_depth & 0xFF);
    tx_packet[3] = ((ds18b20_temperature >> 8) & 0xFF);
    tx_packet[4] = (ds18b20_temperature & 0xFF);
    for (byte i = 0; i < PACKET_BYTE_LENGTH; i++) {
      Serial.print(tx_packet[i], DEC);
      Serial.print(" ");
    }
    Serial.println();
    Serial.println();

    //tranport
    driver.send((uint8_t *)tx_packet, PACKET_BYTE_LENGTH);
    driver.waitPacketSent();
*/
