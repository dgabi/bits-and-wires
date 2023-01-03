#include <RTClib.h>
#include <TM1637Display.h>
#include <SPI.h>

const int clk = 8;
const int dio = 9;

const int modeButton = 7;
const int minuteButton = 6;
const int hourButton = 5;

const int buzzer = 10;

int loopcount = 0;
int mode_button_state = 0;
int minute_button_state = 0;
int hour_button_state = 0;

RTC_DS3231 rtc;
TM1637Display display = TM1637Display(clk, dio);

void minuteISR(){
   DateTime now = rtc.now();
   int minute = now.minute();
   if (minute >= 59) {
      minute = 0;
   } else {
      minute++;
   }
   rtc.adjust(DateTime
              (now.year(), now.month(), now.day(), now.hour(), minute,
               now.second()));
   Serial.println("minute isr");
}

void hourISR(){
   DateTime now = rtc.now();
   int hour = now.hour();
   if (hour >= 23) {
      hour = 0;
   } else {
      hour++;
   }
   rtc.adjust(DateTime
              (now.year(), now.month(), now.day(), hour, now.minute(),
               now.second()));
   Serial.println("hour isr");
}

void setup(){
   Serial.begin(9600);
   delay(3000);
   if (!rtc.begin()) {
      Serial.println("RTC not plugged in!");
   }

   if (rtc.lostPower()) {
      rtc.adjust(DateTime(2023, 1, 3, 12, 10, 0));
   }
   // 0 - 7
   display.setBrightness(4);
   display.clear();

   pinMode(minuteButton, INPUT);
   pinMode(hourButton, INPUT);
   pinMode(modeButton, INPUT);
   pinMode(buzzer, OUTPUT);
   //attachInterrupt(digitalPinToInterrupt(minuteButton), minuteISR, RISING);
   //attachInterrupt(digitalPinToInterrupt(hourButton), hourISR, RISING);
}

void loop(){
   DateTime now = rtc.now();
   int displaytime = (now.hour() * 100) + now.minute();
   display.showNumberDecEx(displaytime, 0xb11100000, true);
   delay(300);

   mode_button_state = digitalRead(modeButton);
   minute_button_state = digitalRead(minuteButton);
   hour_button_state = digitalRead(hourButton);
   if (hour_button_state == HIGH) {
      hourISR();
   }
   if (minute_button_state == HIGH) {
      minuteISR();
   }

   if (mode_button_state == HIGH) {
      tone(buzzer, 10000, 10);
   }
}
