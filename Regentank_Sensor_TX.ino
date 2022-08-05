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

#define VOLUME_MEASUREMENT_MILI_DELAY           36000000    //=1h
#define TEMPERATURE_MEASUREMENT_MILI_DELAY      60000       //1m
#define LED_ANIMATION_MILI_DELAY                500         //0.5s

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

//************ VARIABLES ************
RH_ASK driver(2000, PIN_RX, PIN_TX, 0); // ATTiny, RX on D3 (pin 2 on attiny85) TX on D4 (pin 3 on attiny85), 
String unit = "mm";

Ultrasonic ultrasonicSensor(PIN_TRIG, PIN_ECHO, unit);

OneWire oneWire(ONE_WIRE_BUS);
DallasTemperature sensors(&oneWire);

unsigned long previousMillis = 0;
unsigned long currentMillis = 0;
word ultrasonic_depth; //unsigned 16bit
//word ultrasonic_depth_samples_array[ULTRASONIC_DEPTH_SAMPLES];
//word average_ultrasonic_depth = 0;
int ds18b20_temperature; //signed 16 bit
byte tx_packet[PACKET_BYTE_LENGTH];
byte display_volume_leds = 0;


//************ SETUP ************
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
  
}
void loop() {
  //const char *msg = "hello";

  currentMillis = millis();
  if (currentMillis - previousMillis >= TASK_DELAY_MS) {

    //enable PS
    digitalWrite(PIN_PS_EN, LOW);

    //enable Battery voltage readout
    digitalWrite(PIN_READ_BAT_V, HIGH);

    //small dalay without using the Delay() function.
    unsigned long temp_current_Millis = millis();
    while ((temp_current_Millis + 50) > millis()) {
      //just wait
    }

    //readout battery voltage
    Serial.print("The battery voltage is : ");
    float battery_voltage = 0.024 * (analogRead(PIN_ANALOG_BAT_V));
    Serial.print(battery_voltage, 2);
    Serial.println(" V");

    //readout untrasonic distance
    Serial.print("The Distance is : ");
    ultrasonic_depth = ultrasonicSensor.getDistance();
    ultrasonic_depth = (A*ultrasonic_depth + B); 
    Serial.print(ultrasonic_depth);
    Serial.println(" mm");
  
    //readout temperature
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
    Serial.print((ds18b20_temperature / 100), 2);
    Serial.println(" °C");
    Serial.print("Time to receive one wire temp :");
    Serial.print(millis() - temp_measure_current_millis);
    Serial.println(" ms");

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

    //update LEDs
    display_volume_leds++;
    if (display_volume_leds >= 6) {
      display_volume_leds = 0;
    }
    digitalWrite(PIN_EMPTY_LED, (display_volume_leds >= 1));  
    digitalWrite(PIN_25_LED, (display_volume_leds >= 2));
    digitalWrite(PIN_50_LED, (display_volume_leds >= 3));
    digitalWrite(PIN_75_LED, (display_volume_leds >= 4));
    digitalWrite(PIN_FULL_LED, (display_volume_leds >= 5));
    
    //show error
      //todo

    //end
    previousMillis = currentMillis;

    //disable ADC readout
    digitalWrite(PIN_READ_BAT_V, LOW);

    //disable PS
    digitalWrite(PIN_PS_EN, HIGH);
  }
}
