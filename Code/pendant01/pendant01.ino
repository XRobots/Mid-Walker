int sw1;
int sw2;
int sw3;
int sw4;

int but1;
int but2;
int but3;
int but4;
int but5;
int but6;
int but7;
int but8;
int but9;
int but10;
int but11;
int but12;
int but13;
int but14;

int total1;
int total2;

int joy1;
int joy2;
int joy1Th;
int joy2Th;

unsigned long currentMillis;
unsigned long previousMillis = 0;        // set up timers
long interval = 20;             // time constant for timer

void setup() {

  Serial.begin(115200);
  Serial1.begin(115200);
  
  pinMode(2, INPUT_PULLUP);
  pinMode(3, INPUT_PULLUP);
  pinMode(4, INPUT_PULLUP);
  pinMode(5, INPUT_PULLUP);
  pinMode(6, INPUT_PULLUP);
  pinMode(7, INPUT_PULLUP);
  pinMode(8, INPUT_PULLUP);
  pinMode(9, INPUT_PULLUP);
  pinMode(10, INPUT_PULLUP);
  pinMode(11, INPUT_PULLUP);
  pinMode(12, INPUT_PULLUP);
  pinMode(17, INPUT_PULLUP);
  pinMode(18, INPUT_PULLUP);
  pinMode(19, INPUT_PULLUP);
  pinMode(20, INPUT_PULLUP);
  pinMode(21, INPUT_PULLUP);  
  pinMode(22, INPUT_PULLUP); 
  pinMode(23, INPUT_PULLUP);
}

void loop() {

  currentMillis = millis();
  if (currentMillis - previousMillis >= interval) {   // start of timed loop
      previousMillis = currentMillis;
          
      joy1 = analogRead(A1);      // FB
      joy2 = analogRead(A0);      // Twist
      joy1Th = thresholdStick(joy1);
      joy2Th = thresholdStick(joy2);
    
      sw1 = digitalRead(3);
      sw2 = digitalRead(4);
      sw3 = digitalRead(2);
      sw4 = digitalRead(23);
    
      but1 = digitalRead(7);
      but2 = digitalRead(8);
      but3 = digitalRead(6);
      but4 = digitalRead(5);
    
      but5 = digitalRead(10);
      but6 = digitalRead(12);
      but7 = digitalRead(11);
      but8 = digitalRead(9);
    
      but9 = digitalRead(18);
      but10 = digitalRead(17);
    
      but11 = digitalRead(22);
      but12 = digitalRead(19);
      but13 = digitalRead(20);
      but14 = digitalRead(21);
    
      // pack up data to smaller variables
    
      if (sw1 == 0) {
        total1 = total1 + 1;
      }
      if (sw2 == 0) {
        total1 = total1 + 2;
      }
      if (sw3 == 0) {
        total1 = total1 + 4;
      }
      if (sw4 == 0) {
        total1 = total1 + 8;
      }
      if (but1 == 0) {
        total1 = total1 + 16;
      }
      if (but2 == 0) {
        total1 = total1 + 32;
      }
      if (but3 == 0) {
        total1 = total1 + 64;
      }
      if (but4 == 0) {
        total1 = total1 + 128;
      }
      if (but5 == 0) {
        total1 = total1 + 256;
      }
      if (but6 == 0) {
        total1 = total1 + 512;
      }
      if (but7 == 0) {
        total1 = total1 + 1024;
      }
      if (but8 == 0) {
        total1 = total1 + 2048;
      }
      if (but9 == 0) {
        total2 = total2 + 1;
      }
      if (but10 == 0) {
        total2 = total2 + 2;
      }
      if (but11 == 0) {
        total2 = total2 + 4;
      }
      if (but12 == 0) {
        total2 = total2 + 8;
      }
      if (but13 == 0) {
        total2 = total2 + 16;
      }
      if (but14 == 0) {
        total2 = total2 + 32;
      }
    
      // send to serial 1
    
      Serial1.print(9000);    // start of data
      Serial1.print(" , ");    // start of data
      Serial1.print(total1);
      Serial1.print(" , ");
      Serial1.print(total2);
      Serial1.print(" , ");
      Serial1.print(joy1Th);
      Serial1.print(" , ");
      Serial1.print(joy2Th);
      Serial1.print('\n');
  
    
      //Serial.print(total1);
      //Serial.print(" , ");
      //Serial.print(total2);  
      //Serial.println();
    
      total1 = 0;     // set totals back to zero on each loop
      total2 = 0;
      
      
     /*
      Serial.print(joy1Th);
      Serial.print(" , ");
      Serial.print(joy2Th);
      Serial.print(" , ");
      Serial.print(sw1);
      Serial.print(" , ");
      Serial.print(sw2);
      Serial.print(" , ");
      Serial.print(sw3);
      Serial.print(" , ");
      Serial.print(sw4);
      Serial.print(" , ");
      
      Serial.print(but1);
      Serial.print(" , ");
      Serial.print(but2);
      Serial.print(" , ");
      Serial.print(but3);
      Serial.print(" , ");
      Serial.print(but4);
      Serial.print(" , ");
    
      Serial.print(but5);
      Serial.print(" , ");
      Serial.print(but6);
      Serial.print(" , ");
      Serial.print(but7);
      Serial.print(" , ");
      Serial.print(but8);
      Serial.print(" , ");
    
      Serial.print(but9);
      Serial.print(" , ");
      Serial.print(but10);
      Serial.print(" , ");
    
      Serial.print(but11);
      Serial.print(" , ");
      Serial.print(but12);
      Serial.print(" , ");
      Serial.print(but13);
      Serial.print(" , ");
      Serial.print(but14);
      Serial.print(" , ");
      */

  } // end of timed loop

} // end of main loop
