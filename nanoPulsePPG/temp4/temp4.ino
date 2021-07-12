#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>
//#include "ssd1306h.h"
#include "MAX30102.h"
#include "Pulse.h"
#include <avr/pgmspace.h>
#include <EEPROM.h>
#include <avr/sleep.h>

// Routines to clear and set bits 
#ifndef cbi
#define cbi(sfr, bit) (_SFR_BYTE(sfr) &= ~_BV(bit))
#endif
#ifndef sbi
#define sbi(sfr, bit) (_SFR_BYTE(sfr) |= _BV(bit))
#endif


//#define LED 13
#define BUTTON 3
#define OPTIONS 7

#define SCREEN_WIDTH 128 
#define SCREEN_HEIGHT 64 

#define SCREEN_ADDRESS 0x3C 
Adafruit_SSD1306 display(SCREEN_WIDTH, SCREEN_HEIGHT, &Wire, -1);


//SSD1306 oled; 
MAX30102 sensor;
Pulse pulseIR;
Pulse pulseRed;
MAFilter bpm;


//spo2_table is approximated as  -45.060*ratioAverage* ratioAverage + 30.354 *ratioAverage + 94.845 ;
const uint8_t spo2_table[184] PROGMEM =
        { 95, 95, 95, 96, 96, 96, 97, 97, 97, 97, 97, 98, 98, 98, 98, 98, 99, 99, 99, 99, 
          99, 99, 99, 99, 100, 100, 100, 100, 100, 100, 100, 100, 100, 100, 100, 100, 100, 100, 100, 100, 
          100, 100, 100, 100, 99, 99, 99, 99, 99, 99, 99, 99, 98, 98, 98, 98, 98, 98, 97, 97, 
          97, 97, 96, 96, 96, 96, 95, 95, 95, 94, 94, 94, 93, 93, 93, 92, 92, 92, 91, 91, 
          90, 90, 89, 89, 89, 88, 88, 87, 87, 86, 86, 85, 85, 84, 84, 83, 82, 82, 81, 81, 
          80, 80, 79, 78, 78, 77, 76, 76, 75, 74, 74, 73, 72, 72, 71, 70, 69, 69, 68, 67, 
          66, 66, 65, 64, 63, 62, 62, 61, 60, 59, 58, 57, 56, 56, 55, 54, 53, 52, 51, 50, 
          49, 48, 47, 46, 45, 44, 43, 42, 41, 40, 39, 38, 37, 36, 35, 34, 33, 31, 30, 29, 
          28, 27, 26, 25, 23, 22, 21, 20, 19, 17, 16, 15, 14, 12, 11, 10, 9, 7, 6, 5, 
          3, 2, 1 } ;


static const unsigned char PROGMEM noPulseIcon[] = {0x01, 0xf0, 0x0f, 0x80, 0x03, 0x18, 0x18, 0xc0, 0x06, 0x0c, 0x30, 0x60, 0x0c, 0x06, 0x60, 0x30, 0x18, 0x02, 0x40, 0x18, 0x30, 0x03, 0xc0, 0x0c, 0x20, 0x01, 0x80, 0x04, 0x60, 0x01, 0x80, 0x06, 0x40, 0x00, 0x00, 0x02, 0x40, 0x00, 0x00, 0x02, 0xc0, 0x00, 0x00, 0x03, 0x80, 0x00, 0x00, 0x01, 0x80, 0x00, 0x00, 0x01, 0x80, 0x00, 0x00, 0x01, 0x80, 0x00, 0x00, 0x01, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0x80, 0x00, 0x00, 0x01, 0xc0, 0x00, 0x00, 0x03, 0x60, 0x00, 0x00, 0x06, 0x30, 0x00, 0x00, 0x0c, 0x18, 0x00, 0x00, 0x18, 0x0c, 0x00, 0x00, 0x30, 0x06, 0x00, 0x00, 0x60, 0x03, 0x00, 0x00, 0xc0, 0x01, 0x80, 0x01, 0x80, 0x00, 0xc0, 0x03, 0x00, 0x00, 0x60, 0x06, 0x00, 0x00, 0x30, 0x0c, 0x00, 0x00, 0x1c, 0x38, 0x00, 0x00, 0x06, 0x60, 0x00, 0x00, 0x01, 0x80, 0x00};
static const unsigned char PROGMEM pulseIcon[] = {0x01, 0xf0, 0x0f, 0x80, 0x03, 0x18, 0x18, 0xc0, 0x06, 0x0c, 0x30, 0x60, 0x0c, 0x06, 0x60, 0x30, 0x18, 0x02, 0x40, 0x18, 0x30, 0x03, 0xc0, 0x0c, 0x20, 0x01, 0x80, 0x04, 0x60, 0x01, 0x80, 0x06, 0x40, 0x00, 0x00, 0x02, 0x40, 0x08, 0x00, 0x02, 0xc0, 0x14, 0x00, 0x03, 0x80, 0x14, 0x00, 0x01, 0x80, 0x24, 0x00, 0x01, 0x80, 0x22, 0x00, 0x01, 0x80, 0x42, 0x00, 0x01, 0xff, 0xc1, 0x03, 0xff, 0xff, 0x81, 0x07, 0xff, 0x80, 0x00, 0x84, 0x01, 0xc0, 0x00, 0x88, 0x03, 0x60, 0x00, 0x48, 0x06, 0x30, 0x00, 0x50, 0x0c, 0x18, 0x00, 0x50, 0x18, 0x0c, 0x00, 0x20, 0x30, 0x06, 0x00, 0x00, 0x60, 0x03, 0x00, 0x00, 0xc0, 0x01, 0x80, 0x01, 0x80, 0x00, 0xc0, 0x03, 0x00, 0x00, 0x60, 0x06, 0x00, 0x00, 0x30, 0x0c, 0x00, 0x00, 0x1c, 0x38, 0x00, 0x00, 0x06, 0x60, 0x00, 0x00, 0x01, 0x80, 0x00};
static const unsigned char PROGMEM tmpIcon[] = {0x00, 0x07, 0xc0, 0x00, 0x00, 0x08, 0x20, 0x00, 0x00, 0x08, 0x20, 0x00, 0x00, 0x0e, 0x20, 0x00, 0x00, 0x08, 0x20, 0x00, 0x00, 0x08, 0x20, 0x00, 0x00, 0x0e, 0x20, 0x00, 0x00, 0x08, 0x20, 0x00, 0x00, 0x08, 0x20, 0x00, 0x00, 0x0e, 0x20, 0x00, 0x00, 0x08, 0x20, 0x00, 0x00, 0x08, 0x20, 0x00, 0x00, 0x0e, 0x20, 0x00, 0x00, 0x08, 0x20, 0x00, 0x00, 0x08, 0x20, 0x00, 0x00, 0x0e, 0x20, 0x00, 0x00, 0x08, 0x20, 0x00, 0x00, 0x08, 0x20, 0x00, 0x00, 0x1f, 0xf0, 0x00, 0x00, 0x7f, 0xfc, 0x00, 0x00, 0xff, 0xfe, 0x00, 0x00, 0xff, 0xfe, 0x00, 0x01, 0xff, 0xff, 0x00, 0x01, 0xff, 0xff, 0x00, 0x01, 0xff, 0xff, 0x00, 0x01, 0xff, 0xff, 0x00, 0x01, 0xff, 0xff, 0x00, 0x00, 0xff, 0xfe, 0x00, 0x00, 0xff, 0xfe, 0x00, 0x00, 0x7f, 0xfc, 0x00, 0x00, 0x1f, 0xf0, 0x00, 0x00, 0x07, 0xc0, 0x00};
static const unsigned char PROGMEM oxyIcon[] = {0x89, 0x00, 0x00, 0x00, 0x00, 0x00, 0x20, 0x7f, 0x00, 0x00, 0x00, 0x02, 0xff, 0xe0, 0x00, 0x46, 0x2f, 0xff, 0xf8, 0x00, 0x00, 0x09, 0xff, 0xfc, 0x00, 0x10, 0x3f, 0x7f, 0xfe, 0x00, 0x41, 0x55, 0xff, 0xff, 0x00, 0x04, 0xff, 0x80, 0xff, 0x80, 0x89, 0x5e, 0x00, 0x3f, 0xc0, 0x01, 0xf4, 0x00, 0x1f, 0xc0, 0x4b, 0xb8, 0x00, 0x0f, 0xe0, 0x12, 0xf0, 0x00, 0x07, 0xe0, 0x03, 0xd0, 0x00, 0x07, 0xe0, 0x06, 0xe0, 0x00, 0x03, 0xf0, 0x03, 0xa0, 0x00, 0x03, 0xf0, 0x07, 0xe0, 0x00, 0x03, 0xf0, 0x07, 0xe0, 0x00, 0x03, 0xf0, 0x07, 0xe0, 0x00, 0x03, 0xf0, 0x07, 0xe0, 0x00, 0x03, 0xf0, 0x07, 0xe0, 0x00, 0x03, 0xf0, 0x03, 0xf0, 0x00, 0x07, 0xe0, 0x03, 0xf0, 0x00, 0x07, 0xe0, 0x03, 0xf8, 0x00, 0x0f, 0xe0, 0x01, 0xfc, 0x00, 0x1f, 0xc0, 0x01, 0xfe, 0x00, 0x3f, 0xc0, 0x00, 0xff, 0x80, 0xff, 0x80, 0x00, 0x7f, 0xff, 0xff, 0x30, 0x00, 0x3f, 0xff, 0xfe, 0x48, 0x00, 0x1f, 0xff, 0xfc, 0x08, 0x00, 0x0f, 0xff, 0xf8, 0x10, 0x00, 0x03, 0xff, 0xe0, 0x20, 0x00, 0x00, 0x7f, 0x00, 0x78};

//
//void print_digit(int x, int y, long val, char c=' ', uint8_t field = 3,const int BIG = 2)
//    {  
//    uint8_t ff = field;
//    do { 
//        char ch = (val!=0) ? val%10+'0': c;
//        oled.drawChar( x+BIG*(ff-1)*6, y, ch, BIG);
//        val = val/10; 
//        --ff;
//    } while (ff>0);
//}



int  beatAvg;
int  SPO2;
float temp;
bool filter_for_graph = false;
bool draw_Red = false;
uint8_t pcflag =0;
uint8_t istate = 0;
uint8_t sleep_counter = 0;



void draw_oled(int msg);

void button(void){
    pcflag = 1;
}

void checkbutton(){
    if (pcflag && !digitalRead(BUTTON)) {
      istate = (istate +1) % 4;
      filter_for_graph = istate & 0x01;
      draw_Red = istate & 0x02;
      EEPROM.write(OPTIONS, filter_for_graph);
      EEPROM.write(OPTIONS+1, draw_Red);
    }
    pcflag = 0;
}



void Display4(){
   if(pcflag && !digitalRead(BUTTON)){
     draw_oled(3);
     delay(1100);
   }
   pcflag = 0;
 

}

void go_sleep() {
//    oled.fill(0);
//    oled.off();
    display.clearDisplay();
    display.display();
    delay(10);
    sensor.off();
    delay(10);
    cbi(ADCSRA, ADEN);  // disable adc
    delay(10);
//    pinMode(0,INPUT);
//    pinMode(2,INPUT);
    set_sleep_mode(SLEEP_MODE_PWR_DOWN);     
    sleep_mode();  // sleep until button press 
    // cause reset
    setup();
}



void setup(void) {
//  Serial.begin(9600);
//  pinMode(LED, OUTPUT);
  pinMode(BUTTON, INPUT_PULLUP);
  filter_for_graph = EEPROM.read(OPTIONS);
  draw_Red = EEPROM.read(OPTIONS+1);

  if(!display.begin(SSD1306_SWITCHCAPVCC, SCREEN_ADDRESS))
    while(true);
  display.clearDisplay();
  display.display();
  
  if (!sensor.begin()){
    draw_oled(0);
    while (1);
  }
  sensor.setup(); 
  sensor.enableDIETEMPRDY();
  attachInterrupt(digitalPinToInterrupt(BUTTON),button, CHANGE);
}

long lastBeat = 0;    //Time of the last beat 
long displaytime = 0; //Time of the last display update
bool beat = false;
long lastTemp = 0;

int i = 0;
void loop()  {
//    draw_oled(i++ % 5);
//    draw_oled(4);
//    delay(500);
//    return;
    
    long now = millis();   //start time of this cycle
    if((now - lastTemp)>1000){
      lastTemp = now;
      temp = sensor.readTemperature();  
    }
    sensor.check();
    if (!sensor.available()) return;
    uint32_t irValue = sensor.getIR(); 
    uint32_t redValue = sensor.getRed();
    sensor.nextSample();
    if (irValue<5000) {
        checkbutton();
        draw_oled(sleep_counter<=50 ? 1 : 4); // finger not down message
        delay(200);
        ++sleep_counter;
        if (sleep_counter>100) {
          go_sleep(); 
          sleep_counter = 0;
        }
    } else {
        sleep_counter = 0;
        // remove DC element
        int16_t IR_signal, Red_signal;
        bool beatRed, beatIR;
        if (!filter_for_graph) {
           IR_signal =  pulseIR.dc_filter(irValue) ;
           Red_signal = pulseRed.dc_filter(redValue);
           beatRed = pulseRed.isBeat(pulseRed.ma_filter(Red_signal));
           beatIR =  pulseIR.isBeat(pulseIR.ma_filter(IR_signal));        
        } else {
           IR_signal =  pulseIR.ma_filter(pulseIR.dc_filter(irValue)) ;
           Red_signal = pulseRed.ma_filter(pulseRed.dc_filter(redValue));
           beatRed = pulseRed.isBeat(Red_signal);
           beatIR =  pulseIR.isBeat(IR_signal);
        }
        // check IR or Red for heartbeat     
        if (beatRed){//(draw_Red ? beatRed : beatIR){
            long btpm = 60000/(now - lastBeat);
            if (btpm > 0 && btpm < 200) beatAvg = bpm.filter((int16_t)btpm);
            lastBeat = now; 
//            digitalWrite(LED, HIGH); 
            beat = true;
            // compute SpO2 ratio
            long numerator   = (pulseRed.avgAC() * pulseIR.avgDC())/256;
            long denominator = (pulseRed.avgDC() * pulseIR.avgAC())/256;
            int RX100 = (denominator>0) ? (numerator * 100)/denominator : 999; 
            // from spo2_table
            if ((RX100>=0) && (RX100<184))
              SPO2 = pgm_read_byte_near(&spo2_table[RX100]);
        }
        // update display every 50 ms if fingerdown
        if (now-displaytime>50) {
            displaytime = now;
            draw_oled(2);
            
        }
        Display4();

 
    }
    // flash led for 25 ms
    if (beat && (now - lastBeat)>100){
//        digitalWrite(LED, LOW);
        beat = false;
     }
}


void draw_oled(int msg) {
  
    display.clearDisplay();
    display.setTextColor(SSD1306_WHITE);
    
    switch(msg){
        case 0:  display.setTextSize(2);
                 display.setCursor(20,25);
                 display.print("ERROR..."); 
                 break;
        case 1:
                 display.setTextSize(2);
                 display.setCursor(20,5);
                 display.print("PLACE"); 
                 
                 display.setCursor(20,25);
                 display.print("YOUR"); 
                 
                 display.setCursor(20,45);
                 display.print("FINGER"); 
                 break;

        
        case 3:  display.setTextSize(2);
                 display.setCursor(0,0);
                 display.print("BPM: "); 
                 display.print(beatAvg);
                 display.setCursor(0,20);
                 display.print("SPO2: "); 
                 display.print(beatAvg);
                 display.setCursor(0,40);
                 display.print("TEMP: "); 
                 display.print(temp);
                 break;
         
        case 4:  display.setTextSize(2);
                 display.setCursor(5,25);
                 display.print("OFF IN "); 
                 display.print(10-sleep_counter/10); 
                 display.print("s"); 
                 break;
                 
        case 2:    
                display.drawBitmap(0,0,beat ? pulseIcon : noPulseIcon,32,32,1);
                display.drawBitmap(47,0,tmpIcon,32,32,1);
                display.drawBitmap(90,0,oxyIcon,37,32,1);

                display.setTextColor(SSD1306_WHITE);

                display.setTextSize(2);
                display.setCursor(95,44);
                if(SPO2 >= 100){
                  display.print(SPO2/10);
                  display.setTextSize(1);
                  display.setCursor(120,51);
                  display.print(SPO2%10);
                }
                else {
                  display.print(SPO2);
                }
              
                display.setTextSize(1);
                display.setCursor(122, 42);
                display.print("%");



                display.setTextSize(2);
                uint8_t t1 = temp/1;
                uint8_t t2 = (temp - t1)*100;
                display.setCursor(47, 34);
                display.print(t1);
              
                display.setCursor(47, 50);
                if(t2<10) display.print("0");
                display.print(t2);
                
              
                display.setCursor(72, 42);
                display.setTextSize(1);
                display.cp437(true);
                display.write(7);
              
                display.setTextSize(1);
                display.setCursor(75, 50);
                display.print("C");

                display.setTextSize(2);
                display.setCursor(0, 44);
                display.print(beatAvg);
                
                display.setTextSize(1);
                display.setCursor(25, 55);
                display.print("M");
                display.setCursor(25, 47);
                display.print("P");
                display.setCursor(25, 39);
                display.print("B");
                break;
         
                 
                 
                
//                oled.drawStr(28,12,F("OFF IN"),1);
//                 oled.drawChar(76,12,10-sleep_counter/10+'0');
//                 oled.drawChar(82,12,'s');
        
                
                 
        }

    display.display();

}
