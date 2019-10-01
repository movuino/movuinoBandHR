#include "FS.h"
#include "SPIFFS.h"
#include <Wire.h>
#include <SPI.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BME280.h>
#include <Adafruit_NeoPixel.h>
#include "max30102.h"
#include "algorithm_by_RF.h"



/*Neopixels global values*/
#define PIN           15
#define NUMPIXELS     1
Adafruit_NeoPixel pixels = Adafruit_NeoPixel(NUMPIXELS, PIN, NEO_GRB + NEO_KHZ800);


/* You only need to format SPIFFS the first time you run a
   test or else use the SPIFFS plugin to create a partition
   https://github.com/me-no-dev/arduino-esp32fs-plugin */
#define FORMAT_SPIFFS_IF_FAILED true

/*Pins*/
int buttonPin=13;
long lastButtonTime = 0;
int debounceDelay = 400;
int buttonState = 1;
int opMode=0;
long int sampleNb=0;
long int startTimer;
long int dataTimer;
long int dataTimer0;
long int timer1;
int samplingFrequency = 1; //in HZ
int samplingPeriod=1000/samplingFrequency;
bool pixelState=false;
File fw;
const byte oxiInt = 14;
/*needed for HR sensor*/
uint32_t elapsedTime,timeStart;
uint32_t aun_ir_buffer[BUFFER_SIZE]; //infrared LED sensor data
uint32_t aun_red_buffer[BUFFER_SIZE];  //red LED sensor data
float aun_ir_bufferf1[BUFFER_SIZE]; //infrared LED sensor data filtered : dc removal
float aun_red_bufferf1[BUFFER_SIZE];  //red LED sensor data filtered : dc removal
float aun_ir_bufferf2[BUFFER_SIZE];
float aun_red_bufferf2[BUFFER_SIZE];

float old_n_spo2;  // Previous SPO2 value
uint8_t uch_dummy,k;

//tell if we use raw data or filetered data
int rawData=0;
int dataMode=0; //0 : raw only ; 1 : filtered only ; 2 raw & filtered & spo2v & bpm
int timerArray[BUFFER_SIZE];

void read_memory() {
  // nothing to do for now, this is just a simple test
  File f = SPIFFS.open("/data/1.txt", "r");
  // we could open the file
  while (f.available()) {
    //Lets read line by line from the file
    String line = f.readStringUntil('\n');
    Serial.println(line);
    delay(5);
   // yield();
  }
  f.close();
}


void setup(){
    Serial.begin(115200);
    pinMode(oxiInt, INPUT); 
    maxim_max30102_reset(); //resets the MAX30102
    delay(1000);
    maxim_max30102_read_reg(REG_INTR_STATUS_1,&uch_dummy);  //Reads/clears the interrupt status register
    maxim_max30102_init();  //initialize the MAX30102
    old_n_spo2=0.0;

    pinMode(13,INPUT);
    if(!SPIFFS.begin(FORMAT_SPIFFS_IF_FAILED)){
        Serial.println("SPIFFS Mount Failed");
        return;
    }
    /*Pixel setup*/
    pixels.begin(); // This initializes the NeoPixel library.
    pixels.setPixelColor(0, pixels.Color(125,125,125));
    pixels.show();
    delay(100);
    pixels.setPixelColor(0, pixels.Color(0,0,0));
    pixels.show();
    delay(100);
}
void loop(){
  /*for HR sensor*/
  float n_spo2,ratio,correl;  //SPO2 value
  int8_t ch_spo2_valid;  //indicator to show if the SPO2 calculation is valid
  int32_t n_heart_rate; //heart rate value
  int8_t  ch_hr_valid;  //indicator to show if the heart rate calculation is valid
  int32_t i;
  char hr_str[10];
  
  buttonState = digitalRead(buttonPin);
  if (!buttonState) {
    if (millis() - lastButtonTime > debounceDelay) {
      lastButtonTime = millis();
      Serial.println("Recording...");
      //if we are recording stop recording
      if (opMode == 2) {
        //TODO fonction start_recording & stop_recording
        fw.close();
        Serial.println("Stop recording");
        pixels.setPixelColor(0, pixels.Color(0,0,0));
        pixels.show();
        opMode = 0;
      }
      else {
        Serial.println("Recording");
        fw = SPIFFS.open("/data/1.txt", "w+");
        opMode = 2;
        }
    }
  }
 /*Serial Manager*/   
  if (Serial.available() > 0) {
    char inByte = Serial.read();
    //Ping
    if (inByte == '?') {
      Serial.println("?");
    }
    else if (inByte == 'l') {
      Serial.println("Start live");
      startTimer=millis(); 
      rawData=1;
      opMode = 1;
    }
    else if (inByte == 'f') {
      Serial.println("filtered data live");
      rawData=0;
      opMode = 1;
    }
    //quit live mode
    else if (inByte == 'L') {
      opMode = 0;
    }
    else if (inByte == 'r') {
      Serial.println("recording!");
      fw = SPIFFS.open("/data/1.txt", "w+");
      startTimer=millis(); 
      opMode = 2;
    }
    else if (inByte == 'R') {
      Serial.println("Stop recording!");
      fw.close();
      pixels.setPixelColor(0, pixels.Color(0,0,0));
      pixels.show();
      //reset sample nb
      sampleNb=0;
      opMode = 0;
    }
    else if (inByte == 'p') {
      //Serial.println("Read");
      read_memory();
      Serial.println("Done!");
    }
      //delete file
    else if (inByte == 'P') {
      Serial.println("Deleting file");
      SPIFFS.remove("/data/1.txt");
      Serial.println("deletion");
      opMode = 0;
      //read_memory();
    }
  }  
  /*opMode Manager*/
  if (opMode == 1) {
      //buffer length of BUFFER_SIZE stores ST seconds of samples running at FS sps
      //read BUFFER_SIZE samples, and determine the signal range
      for(i=0;i<BUFFER_SIZE;i++)
        {
        while(digitalRead(oxiInt)==1);  //wait until the interrupt pin asserts
        maxim_max30102_read_fifo((aun_red_buffer+i), (aun_ir_buffer+i));  //read from MAX30102 FIFO
        if(dataMode==0 || dataMode==2)  {
            //Serial.print(F("r"));
            //Serial.print(F(" "));
            if(i==0) {
              dataTimer=millis()-startTimer;
              dataTimer0= dataTimer;
              }
            else dataTimer=dataTimer+40;
            //Serial.print(dataTimer, DEC);
            //Serial.print(F(" "));
            //Serial.print(i, DEC);
            //Serial.print(F(" "));
            Serial.print(aun_red_buffer[i], DEC);
            //Serial.print(F(" "));
            //Serial.print(aun_ir_buffer[i], DEC);    
            Serial.println("");
            }
        }
    if(dataMode==1 || dataMode==2) {
      //calculate heart rate and SpO2 after BUFFER_SIZE samples (ST seconds of samples) using Robert's method
      rf_heart_rate_and_oxygen_saturation2(aun_ir_buffer, BUFFER_SIZE, aun_red_buffer, &n_spo2, &ch_spo2_valid, &n_heart_rate, &ch_hr_valid, &ratio, &correl,1,dataTimer0,aun_ir_bufferf1,aun_red_bufferf1,aun_ir_bufferf2,aun_red_bufferf2); 
      for(i=1;i<BUFFER_SIZE;i++){
        Serial.print("f ");
        Serial.print(aun_ir_bufferf1[i]);
        Serial.print(" ");
        Serial.print(aun_red_bufferf1[i]);
        Serial.print(" ");
        Serial.print(aun_ir_bufferf2[i]);
        Serial.print(" ");
        Serial.println(aun_red_bufferf2[i]);
        }
      if(dataMode==2){
         Serial.print("spo2 ");
         Serial.print(n_spo2);
         Serial.print(" ");
         Serial.println(ch_spo2_valid);
         Serial.print("bpm ");
         Serial.print(n_heart_rate);
         Serial.print(" ");
         Serial.println(ch_hr_valid);
         Serial.print("correl ");
         Serial.print(ratio);
         Serial.print(" ");
         Serial.println(correl);
        }
      
      }
     else if(dataMode==3){
       //calculate heart rate and SpO2 after BUFFER_SIZE samples (ST seconds of samples) using Robert's method
       rf_heart_rate_and_oxygen_saturation(aun_ir_buffer, BUFFER_SIZE, aun_red_buffer, &n_spo2, &ch_spo2_valid, &n_heart_rate, &ch_hr_valid, &ratio, &correl);
       Serial.println("--RF--");
       Serial.print(n_spo2, DEC);
       Serial.print("\t");
       Serial.print(ch_spo2_valid);
       Serial.print("\t");
       Serial.print(n_heart_rate, DEC);
       Serial.print("\t");
       Serial.println(hr_str);
       Serial.print("\t");
       Serial.println(ch_hr_valid);
       Serial.print("\t");
       Serial.println(correl);
       Serial.println("------");
     }
    }
  else if (opMode == 2) {
      //buffer length of BUFFER_SIZE stores ST seconds of samples running at FS sps
      //read BUFFER_SIZE samples, and determine the signal range
       for(i=0;i<BUFFER_SIZE;i++)
        {
        while(digitalRead(oxiInt)==1);  //wait until the interrupt pin asserts
        maxim_max30102_read_fifo((aun_red_buffer+i), (aun_ir_buffer+i));  //read from MAX30102 FIFO
        
      if(dataMode==0 || dataMode==2)  {
            //fw.print(F("r"));
            //fw.print(F(" "));
           /* if(i==0) {
              dataTimer=millis()-startTimer;
              timerArray[i]=dataTimer;
              }
            else timerArray[i]=dataTimer+(i*40);
            fw.print(timerArray[i], DEC);
            fw.print(F(" "));
            fw.print(i, DEC);
            fw.print(F(" "));*/
            fw.println(aun_red_buffer[i], DEC);
           // fw.print(F(" "));
            //fw.println(aun_ir_buffer[i], DEC);    
            }
        }
      if(dataMode==1 || dataMode==2) {
      //calculate heart rate and SpO2 after BUFFER_SIZE samples (ST seconds of samples) using Robert's method
      rf_heart_rate_and_oxygen_saturation2(aun_ir_buffer, BUFFER_SIZE, aun_red_buffer, &n_spo2, &ch_spo2_valid, &n_heart_rate, &ch_hr_valid, &ratio, &correl,1,dataTimer0,aun_ir_bufferf1,aun_red_bufferf1,aun_ir_bufferf2,aun_red_bufferf2); 
      //aun_ir_bufferf = with dc removal
      //aun_red_bufferf = with dc removal
      for(i=1;i<BUFFER_SIZE;i++){
            fw.print(" ");
            fw.print(aun_ir_bufferf1[i]);
            fw.print(" ");
            fw.print(aun_red_bufferf1[i]);
            fw.print(" ");
            fw.print(aun_ir_bufferf2[i]);
            fw.print(" ");
            fw.println(aun_red_bufferf2[i]);
          }
         fw.print("spo2");
         fw.print(n_spo2);
         fw.print(" ");
         fw.print(ch_spo2_valid);
         fw.print(" ");
         fw.print(n_heart_rate);
         fw.print(" ");
         fw.print(ch_hr_valid);
         fw.print(" ");
         fw.print(ratio);
         fw.print(" ");
         fw.println(correl);
      }
        }
}
