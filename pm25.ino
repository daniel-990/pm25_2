// On Leonardo/Micro or others with hardware serial, use those!
// uncomment this line:
// #define pmsSerial Serial1

// For UNO and others without hardware serial, we must use software serial...
// pin #2 is IN from sensor (TX pin on sensor), leave pin #3 disconnected
// comment these two lines if using hardware serial
#include <SoftwareSerial.h>
#include <FastLED.h>
SoftwareSerial pmsSerial(D3, 3);
SoftwareSerial BT(D8, D7); // RX, TX
int BluetoothData; // the data given from Computer

#define NUM_LEDS 3
#define DATA_PIN D1 //12 = d6
#define CLOCK_PIN D2 //4 = d2

// Array leds
CRGB leds[NUM_LEDS];

void setup() {
  // our debugging output
  Serial.begin(115200);

  // put your setup code here, to run once:
  BT.begin(9600);
  BT.println("Bluetooth On please press 1 or 0 blink LED ..");

  // sensor baud rate is 9600
  pmsSerial.begin(9600);
  FastLED.addLeds<DOTSTAR, DATA_PIN,CLOCK_PIN,BGR >(leds, NUM_LEDS);
}

struct pms5003data {
  uint16_t framelen;
  uint16_t pm10_standard, pm25_standard, pm100_standard;
  uint16_t pm10_env, pm25_env, pm100_env;
  uint16_t particles_03um, particles_05um, particles_10um, particles_25um, particles_50um, particles_100um;
  uint16_t unused;
  uint16_t checksum;
};

struct pms5003data data;
    
void loop() {
  if (readPMSdata(&pmsSerial)) {
    // reading data was successful!
    Serial.println();
    Serial.println("---------------------------------------");
    Serial.println("Concentration Units (standard)");
    Serial.print("PM 1.0: "); Serial.print(data.pm10_standard);
    Serial.print("\t\tPM 2.5: "); Serial.print(data.pm25_standard);
    Serial.print("\t\tPM 10: "); Serial.println(data.pm100_standard);
    Serial.println("---------------------------------------");
    Serial.println("Concentration Units (environmental)");
    Serial.print("PM 1.0: "); Serial.print(data.pm10_env);
    Serial.print("\t\tPM 2.5: "); Serial.print(data.pm25_env);
    Serial.print("\t\tPM 10: "); Serial.println(data.pm100_env);
    Serial.println("---------------------------------------");
    Serial.print("Particles > 0.3um / 0.1L air:"); Serial.println(data.particles_03um);
    Serial.print("Particles > 0.5um / 0.1L air:"); Serial.println(data.particles_05um);
    Serial.print("Particles > 1.0um / 0.1L air:"); Serial.println(data.particles_10um);
    Serial.print("Particles > 2.5um / 0.1L air:"); Serial.println(data.particles_25um);
    Serial.print("Particles > 5.0um / 0.1L air:"); Serial.println(data.particles_50um);
    Serial.print("Particles > 10.0 um / 0.1L air:"); Serial.println(data.particles_100um);
    Serial.println("---------------------------------------");

      // put your main code here, to run repeatedly:
      if (BT.available())
      {
        BluetoothData=BT.read();
        if(BluetoothData=='2')
        {
          BT.println("PM 2.5: ");
          BT.println(data.pm25_standard);
          BT.println("--------");
              for (int i = 0; i < NUM_LEDS; i++){
                leds[i] = CRGB::Purple;
                FastLED.show();
                delay(100);
              }
              for (int i = 0; i < NUM_LEDS; i++){
                leds[i] = CRGB::Black;
                FastLED.show();
                delay(100);
              }
        }
        if (BluetoothData=='1')
        {
          BT.println("PM 1.0: ");
          BT.println(data.pm10_standard);
          BT.println("--------");
        }
        if (BluetoothData=='0')
        {
          BT.println("PM 10: ");
          BT.println(data.pm100_standard);
          BT.println("-------");
        }
      }
      delay(100);// prepare for next data ...

          if(data.pm25_standard <= 20){
              for (int i = 0; i < NUM_LEDS; i++){
                leds[i] = CRGB::Green;
                FastLED.show();
                delay(100);
              }
              for (int i = 0; i < NUM_LEDS; i++){
                leds[i] = CRGB::Black;
                FastLED.show();
                delay(100);
              }
          }
          
          if(data.pm25_standard >= 21){
              for (int i = 0; i < NUM_LEDS; i++){
                leds[i] = CRGB::Yellow;
                FastLED.show();
                delay(100);
              }
              for (int i = 0; i < NUM_LEDS; i++){
                leds[i] = CRGB::Black;
                FastLED.show();
                delay(100);
              }   
          }
          
          if(data.pm25_standard >= 50){
              for (int i = 0; i < NUM_LEDS; i++){
                leds[i] = CRGB::Orange;
                FastLED.show();
                delay(100);
              }
              for (int i = 0; i < NUM_LEDS; i++){
                leds[i] = CRGB::Black;
                FastLED.show();
                delay(100);
              }
          }
        
          if(data.pm25_standard >= 100){
              for (int i = 0; i < NUM_LEDS; i++){
                leds[i] = CRGB::Brown;
                FastLED.show();
                delay(100);
              }
              for (int i = 0; i < NUM_LEDS; i++){
                leds[i] = CRGB::Black;
                FastLED.show();
                delay(100);
              }   
          }
        
          if(data.pm25_standard >= 200){
              for (int i = 0; i < NUM_LEDS; i++){
                leds[i] = CRGB::Red;
                FastLED.show();
                delay(100);
              }
              for (int i = 0; i < NUM_LEDS; i++){
                leds[i] = CRGB::Black;
                FastLED.show();
                delay(100);
              }   
          }
  }
  
}

boolean readPMSdata(Stream *s) {
  if (! s->available()) {
    return false;
  }
  
  // Read a byte at a time until we get to the special '0x42' start-byte
  if (s->peek() != 0x42) {
    s->read();
    return false;
  }

  // Now read all 32 bytes
  if (s->available() < 32) {
    return false;
  }
    
  uint8_t buffer[32];    
  uint16_t sum = 0;
  s->readBytes(buffer, 32);

  // get checksum ready
  for (uint8_t i=0; i<30; i++) {
    sum += buffer[i];
  }

  /* debugging
  for (uint8_t i=2; i<32; i++) {
    Serial.print("0x"); Serial.print(buffer[i], HEX); Serial.print(", ");
  }
  Serial.println();
  */
  
  // The data comes in endian'd, this solves it so it works on all platforms
  uint16_t buffer_u16[15];
  for (uint8_t i=0; i<15; i++) {
    buffer_u16[i] = buffer[2 + i*2 + 1];
    buffer_u16[i] += (buffer[2 + i*2] << 8);
  }

  // put it into a nice struct :)
  memcpy((void *)&data, (void *)buffer_u16, 30);

  if (sum != data.checksum) {
    Serial.println("Checksum failure");
    return false;
  }
  // success!
  return true;
}
