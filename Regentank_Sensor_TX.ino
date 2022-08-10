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
#define FAULT_LED         13  //LED_BUILTIN

#define VOLUME_MEASUREMENT_MILI_DELAY           1000//36000000    //=1h
#define TEMPERATURE_MEASUREMENT_MILI_DELAY      10000       //1min
#define LED_ANIMATION_MILI_DELAY                200         //0.200s

#define PACKET_BYTE_LENGTH    5
#define PACKET_REV            1
#define TASK_DELAY_MS         3000
#define NUM_BLINK_ERROR_LED_DISTANCE_FAULT    5      

//calibration params
#define A   1.1938
#define B   4.058

#define OUTFLOW_HEIGHT_MM           1500    //max water height
#define MUD_THICKNESS_MM            100     //mud in well height
#define SENSOR_HEIGHT_MM            1700    //height sensor above empty well
#define ULTRASONIC_DEPTH_SAMPLES    32

#define ULTRASONIC_MINIMUM_ACCEPTABLE   10
#define ULTRASONIC_MAXIMUM_ACCEPTABLE   SENSOR_HEIGHT_MM

#define LEVEL_10_PERCENT            10
#define LEVEL_25_PERCENT            25  
#define LEVEL_50_PERCENT            50
#define LEVEL_75_PERCENT            75
#define LEVEL_90_PERCENT            90
#define LEVEL_100_PERCENT           100

#define SUCCESSFUL  1
#define FAILED      0



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
word ultrasonic_depth_samples[ULTRASONIC_DEPTH_SAMPLES];
word average_ultrasonic_depth;
word rain_water_volume_percent;
int ds18b20_temperature; //signed 16 bit
float battery_voltage;
byte tx_packet[PACKET_BYTE_LENGTH];
bool bool_led_blink;
uint8_t error_flashes;

//******* FUNCTION DISCIPTION *******
bool do_volume_measurement_task(void);
void do_temperature_measurement_task(void);
void do_battery_voltage_measurement_task(void);
void do_led_animation_task(void);
void ps_enable(void);
void ps_disable(void);
void ps_enabled_after_mili_delay(unsigned long);
void vbat_measure_enable(void);
void vbat_measure_disable(void);
void fault_led_on(void);
void fault_led_off(void);
word calculate_volume_percent(word, word, word, word);
word calculate_average_ultrasonic_depth(word);



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
  pinMode(FAULT_LED, OUTPUT);

  volume_task_millis = millis();
  temperature_task_millis = millis();
  led_task_millis = millis();
  rain_water_volume_percent = 0;
  error_flashes = 0;

  ps_enable();    //enable the 5V power supply. Keep always on. It drives otherwise the 5V sensors insane.
  fault_led_off();

  delay(50);

  for(uint8_t measurement_repeats = ULTRASONIC_DEPTH_SAMPLES; measurement_repeats > 0; measurement_repeats--){
    while(!do_volume_measurement_task());
    Serial.print("sample taken: ");
    Serial.print((ULTRASONIC_DEPTH_SAMPLES + 1) - measurement_repeats); 
    Serial.print("/");
    Serial.println(ULTRASONIC_DEPTH_SAMPLES); 
    delay(20);
  }
  Serial.println("INIT DONE!");
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
bool do_volume_measurement_task() {
  Serial.print("The Distance is : ");
  ultrasonic_depth = ultrasonicSensor.getDistance();
  ultrasonic_depth = (A*ultrasonic_depth + B);    //correct for measuring faults
  // Check if reading was successful
  if ((ultrasonic_depth < ULTRASONIC_MAXIMUM_ACCEPTABLE) && (ultrasonic_depth > ULTRASONIC_MINIMUM_ACCEPTABLE)) {
    Serial.print(ultrasonic_depth);
    Serial.println(" mm");  
    average_ultrasonic_depth = calculate_average_ultrasonic_depth(ultrasonic_depth);
    Serial.print("The Average is : ");
    Serial.print(average_ultrasonic_depth);
    Serial.println(" mm");
    Serial.print("The Volume is : ");   
    rain_water_volume_percent = calculate_volume_percent(SENSOR_HEIGHT_MM, average_ultrasonic_depth, MUD_THICKNESS_MM, OUTFLOW_HEIGHT_MM);
    Serial.print(rain_water_volume_percent);
    Serial.println(" %");
    return(SUCCESSFUL);
  } else {
    error_flashes += NUM_BLINK_ERROR_LED_DISTANCE_FAULT;
    Serial.print("measuring distance fault!");
    Serial.println();
    return(FAILED);  
  }
}


// ***************************************
//           TASKS temperature
//****************************************
void do_temperature_measurement_task() {
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
}

// ***************************************
//             TASKS led-a
//****************************************
void do_led_animation_task() {
  bool_led_blink = !bool_led_blink;     //used for the blinking animation
  if (rain_water_volume_percent < LEVEL_10_PERCENT) {
    digitalWrite(PIN_EMPTY_LED, HIGH);
  } else {
    digitalWrite(PIN_EMPTY_LED, (((rain_water_volume_percent >= LEVEL_10_PERCENT) && (rain_water_volume_percent < LEVEL_25_PERCENT) && (bool_led_blink))));   
  }
  digitalWrite(PIN_25_LED, (rain_water_volume_percent >= LEVEL_25_PERCENT));
  digitalWrite(PIN_50_LED, (rain_water_volume_percent >= LEVEL_50_PERCENT));
  digitalWrite(PIN_75_LED, (rain_water_volume_percent >= LEVEL_75_PERCENT));
  digitalWrite(PIN_FULL_LED, (rain_water_volume_percent >= LEVEL_100_PERCENT));

  //display if needed error codes here
  if (error_flashes) {
    if (bool_led_blink) {
      fault_led_on();
    } else {
      fault_led_off();
      error_flashes--;
    }
  }
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

//basic function to light up the fault LED
void fault_led_on() {
  digitalWrite(FAULT_LED, HIGH);
}

//basic function to light out the fault LED
void fault_led_off() {
  digitalWrite(FAULT_LED, LOW);
}

//function to calculate the percent volume in the rain water tank
word calculate_volume_percent(word sensor_h_mm, word distance_h_mm, word mud_h_mm, word outflow_h_mm) {
  //todo checks
  if ((distance_h_mm + mud_h_mm) > sensor_h_mm) { //if waterlevel is below mudlevel = 0% (limit, otherwise negative value)
    return 0; //%
  }
  return ((sensor_h_mm - distance_h_mm) - mud_h_mm) / (outflow_h_mm / 100);
}

//function to calculate the average measured depth
word calculate_average_ultrasonic_depth(word last_sample) {
  word average = 0;  
  for (uint8_t i = (ULTRASONIC_DEPTH_SAMPLES - 1) ; i > 0; i--) {
    ultrasonic_depth_samples[i] = ultrasonic_depth_samples[i - 1];  //shift values in array one place
    average += ultrasonic_depth_samples[i];  //meanwhile, add all values together
  }
  ultrasonic_depth_samples[0] = (word) last_sample;
  average += ultrasonic_depth_samples[0];  //add last value together
  average /= ULTRASONIC_DEPTH_SAMPLES; //devide by number of values in array -> average! :-
  return average;
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
