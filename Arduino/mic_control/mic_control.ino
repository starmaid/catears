#include <arduinoFFT.h>
#include <Servo.h>
 
#define SAMPLES 64             //Must be a power of 2
#define SAMPLING_FREQUENCY 5000 //Hz, must be less than 10000 due to ADC

#define LED 13

Servo SR1;
Servo SR2;
Servo SL1;
Servo SL2;

arduinoFFT FFT = arduinoFFT();
 
unsigned int sampling_period_us;
unsigned long microseconds;
 
double vReal[SAMPLES];
double vImag[SAMPLES];

// each bin is 40 Hz

int beat = 0;
int vol = 0;
int s = 2;
int n = 2;
 
void setup() {
    pinMode(LED, OUTPUT);
    Serial.begin(115200);
    SR1.attach(5);
    SR1.write(180);
    SR2.attach(6);
    SR2.write(0);
    SL1.attach(9);
    SL1.write(0);
    SL2.attach(10);
    SL2.write(180);
    sampling_period_us = round(1000000*(1.0/SAMPLING_FREQUENCY));
}
 
void loop() {
   
    /*SAMPLING*/
    for(int i=0; i<SAMPLES; i++)
    {
        microseconds = micros();    //Overflows after around 70 minutes!
     
        vReal[i] = analogRead(0);
        vImag[i] = 0;
     
        while(micros() < (microseconds + sampling_period_us)){
        }
    }
 
    /*FFT*/
    FFT.DCRemoval(vReal, SAMPLES);
    FFT.Windowing(vReal, SAMPLES, FFT_WIN_TYP_HAMMING, FFT_FORWARD);
    FFT.Compute(vReal, vImag, SAMPLES, FFT_FORWARD);
    FFT.ComplexToMagnitude(vReal, vImag, SAMPLES);
    //double peak = FFT.MajorPeak(vReal, SAMPLES, SAMPLING_FREQUENCY);
    
    beat = 0;
    for (int i=s; i<=s+n; i++) {
      beat += vReal[i];
    }
    beat = beat / n;

    vol = 0;
    for (int i=0; i<SAMPLES; i++) {
      vol += vReal[i];
    }
    vol = vol / SAMPLES;

    //Serial.print(beat);
    //Serial.print(" ");
    //Serial.print(vol);
    //Serial.print("\n");
    
    /*
    if (beat > 350) {
      digitalWrite(LED,HIGH);
      SR2.write(70);
      SL2.write(100);
      delay(90);
    } else {
      digitalWrite(LED,LOW);
      SR2.write(0);
      SL2.write(180);
      delay(90);
    }
    */

    
    for(int i=0; i<(SAMPLES/2); i++)
    {
        //View all these three lines in serial terminal to see which frequencies has which amplitudes
         
        //Serial.print((i * 1.0 * SAMPLING_FREQUENCY) / SAMPLES, 1);
        //Serial.print(" ");
        Serial.println(vReal[i], 1);    //View only this line in serial plotter to visualize the bins
    }
    

    //delay(5);
}
