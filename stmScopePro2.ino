/*. THIS IS RIGHT VERSION WITH SCROLL
(c) Andrew Hull - 2015

STM32-O-Scope - aka "The Pig Scope" or pigScope released under the GNU GENERAL PUBLIC LICENSE Version 2, June 1991

https://github.com/pingumacpenguin/STM32-O-Scope

Adafruit Libraries released under their specific licenses Copyright (c) 2013 Adafruit Industries.  All rights reserved.



THIS IS STMDUINO CORE


*/

#include "Adafruit_ILI9341_STM.h"
#include "Adafruit_GFX.h"

//     core notes
// timer structures 
//  user\AppData\Local\Arduino15\packages\STMicroelectronics\hardware\stm32\2.7.1\system\Drivers\CMSIS\Device\ST\STM32F1xx\Include\stm32f103x6.h
//
//
//
//
//
//
//
//
//
//
//


// Be sure to use the latest version of the SPI libraries see stm32duino.com - http://stm32duino.com/viewtopic.php?f=13&t=127
#include <SPI.h>

#define PORTRAIT 2
#define LANDSCAPE 3


//#define BKP_REG_BASE   (uint32_t *)(0x40006C00 +0x04)

// Defined for power and sleep functions pwr.h and scb.h
#include <libmaple/pwr.h>
#include <libmaple/scb.h>


// SeralCommand -> https://github.com/kroimon/Arduino-SerialCommand.git
//#include <SerialCommand.h>

/* For reference on STM32F103CXXX

variants/generic_stm32f103c/board/board.h:#define BOARD_NR_SPI              2
variants/generic_stm32f103c/board/board.h:#define BOARD_SPI1_NSS_PIN        PA4
variants/generic_stm32f103c/board/board.h:#define BOARD_SPI1_MOSI_PIN       PA7
variants/generic_stm32f103c/board/board.h:#define BOARD_SPI1_MISO_PIN       PA6
variants/generic_stm32f103c/board/board.h:#define BOARD_SPI1_SCK_PIN        PA5

variants/generic_stm32f103c/board/board.h:#define BOARD_SPI2_NSS_PIN        PB12
variants/generic_stm32f103c/board/board.h:#define BOARD_SPI2_MOSI_PIN       PB15
variants/generic_stm32f103c/board/board.h:#define BOARD_SPI2_MISO_PIN       PB14
variants/generic_stm32f103c/board/board.h:#define BOARD_SPI2_SCK_PIN        PB13

*/

// Additional  display specific signals (i.e. non SPI) for STM32F103C8T6 (Wire colour)
// For the Adafruit shield, these are the default.
#define TFT_DC      PB8      //   (Green) 
#define TFT_CS      PB6      //   (Orange) 
#define TFT_RST     PB7      //   (Yellow)


// Hardware SPI1 on the STM32F103C8T6 *ALSO* needs to be connected and pins are as follows.
//
// SPI1_NSS  (PA4) (LQFP48 pin 14)    (n.c.)
// SPI1_SCK  (PA5) (LQFP48 pin 15)    (Brown)
// SPI1_MOSO (PA6) (LQFP48 pin 16)    (White)
// SPI1_MOSI (PA7) (LQFP48 pin 17)    (Grey)
//

//#define TFT_LED        PA3     // Backlight 
#define TEST_WAVE_PIN       PC13     //PB1 PWM 500 Hz 

// Create the lcd object
Adafruit_ILI9341_STM TFT = Adafruit_ILI9341_STM(TFT_CS, TFT_DC); // Using hardware SPI
//Adafruit_ILI9341 TFT = Adafruit_ILI9341(TFT_CS, TFT_DC, TFT_MOSI, TFT_CLK, TFT_RST, TFT_MISO);


// LED - blinks on trigger events - leave this undefined if your board has no controllable LED
// define as PC13 on the "Red/Blue Pill" boards and PD2 on the "Yellow Pill R"
#define BOARD_LED PC13 //PB0

// Display colours
#define BEAM1_COLOUR ILI9341_GREEN
#define BEAM2_COLOUR ILI9341_RED
#define GRATICULE_COLOUR 0x07FF
#define BEAM_OFF_COLOUR ILI9341_BLACK
#define CURSOR_COLOUR ILI9341_GREEN

// rotary encoder variables
#define HOLD_BUTTON PA0
#define ENCODER_OUT_A PA1
#define ENCODER_OUT_B PA2

#define SINGLE_SHOT_BUTTON PB5

int counter = 0; 
int encoderAState;
int encoderALastState;  


// Analog input
#define ANALOG_MAX_VALUE 4096
const int8_t analogInPin = PB1;   // Analog input pin: any of LQFP44 pins (PORT_PIN), 10 (PA0), 11 (PA1), 12 (PA2), 13 (PA3), 14 (PA4), 15 (PA5), 16 (PA6), 17 (PA7), 18 (PB0), 19  (PB1)
float samplingTime = 0;
float displayTime = 0;


// Variables for the beam position
uint16_t signalX ;
uint16_t signalY ;
uint16_t signalY1;
int16_t xZoomFactor = 1;
// yZoomFactor (percentage)
int16_t yZoomFactor = 100; //Adjusted to get 3.3V wave to fit on screen
int16_t Y_SCALE = 70;
int16_t yPosition = 30 ;

// Startup with sweep hold off or on
boolean triggerHeld = 0 ;

unsigned long sweepDelayFactor = 1;
unsigned long timeBase = 200;  //Timebase in microseconds

// Screen dimensions
int16_t myWidth ;
int16_t myHeight ;

//Trigger stuff
boolean triggered ;

// Sensitivity is the necessary change in AD value which will cause the scope to trigger.
// If VAD=3.3 volts, then 1 unit of sensitivity is around 0.8mV but this assumes no external attenuator. Calibration is needed to match this with the magnitude of the input signal.

// Trigger is setup in one of 32 positions
#define TRIGGER_POSITION_STEP ANALOG_MAX_VALUE/32
// Trigger default position (half of full scale)
int32_t TRIGGER_VALUE = 3048; 

int16_t retriggerDelay = 0;
int8_t triggerType = 2; //0-both 1-negative 2-positive

//Array for trigger points
uint16_t triggerPoints[2];

// Serial output of samples - off by default. Toggled from UI/Serial commands.
boolean serialOutput = false;
// Create Serial Command Object.
//SerialCommand sCmd;
// Create USB serial port
USBSerial serial_debug;

// Samples - depends on available RAM 6K is about the limit on an STM32F103C8T6
// Bear in mind that the ILI9341 display is only able to display 240x320 pixels, at any time but we can output far more to the serial port, we effectively only show a window on our samples on the TFT.
# define maxSamples 1024*5 //1024*6
uint32_t startSample = 0; //10
uint32_t endSample = maxSamples ;

// Array for the ADC data
//uint16_t dataPoints[maxSamples];
uint32_t dataPoints32[maxSamples / 2];
uint16_t *dataPoints = (uint16_t *)&dataPoints32;

//array for computed data (speedup)
uint16_t dataPlot[320]; //max(width,height) for this display


// End of DMA indication
volatile static bool dma1_ch1_Active;
#define ADC_CR1_FASTINT 0x70000 // Fast interleave mode DUAL MODE bits 19-16

bool _singleShot = false;

void setup() {

  // BOARD_LED blinks on triggering assuming you have an LED on your board. If not simply dont't define it at the start of the sketch.
#if defined BOARD_LED
  pinMode(BOARD_LED, OUTPUT);
  digitalWrite(BOARD_LED, HIGH);
  delay(1000);
  digitalWrite(BOARD_LED, LOW);
  delay(1000);
#endif

  Serial.begin(1000000);
  delay(40);
  Serial.println("setup");


  adc_calibrate(ADC1);
  Serial.println("adc1");
  adc_calibrate(ADC2);
  Serial.println("adc2");
  setADCs(); //Setup ADC peripherals for interleaved continuous mode.
  Serial.println("setAdcs");

  // The test pulse is a square wave of approx 3.3V (i.e. the STM32 supply voltage) at approx 1 kHz
  // "The Arduino has a fixed PWM frequency of 490Hz" - and it appears that this is also true of the STM32F103 using the current STM32F03 libraries as per
  // STM32, Maple and Maple mini port to IDE 1.5.x - http://forum.arduino.cc/index.php?topic=265904.2520
  // therefore if we want a precise test frequency we can't just use the default uncooked 50% duty cycle PWM output.
//  timer_set_period(Timer3, 1000);

//  toggleTestPulseOn();

  // Set up our sensor pin(s)
  pinMode(analogInPin, INPUT_ANALOG);

  Serial.println("tft setup");
  TFT.begin();
  delay(100);
  Serial.println("tft stp cplt");
  // initialize the display
  clearTFT();

  myHeight   = TFT.width() ;
  myWidth  = TFT.height();
  TFT.setTextColor(CURSOR_COLOUR, BEAM_OFF_COLOUR) ;

  TFT.setRotation(LANDSCAPE);


  delay(1000) ; //5000
  clearTFT();
  triggered = false;

  showLabels();

  Serial.println("START");

  TIMER1->regs.adv->PSC = 72 - 1;  // Set prescaler to 72-1 to get 1 MHz timer frequency (72 MHz system clock / 72)
  TIMER1->regs.adv->ARR = 20 - 1; // Set auto-reload to 100-1 us for 10 kHz frequency (1 MHz / 1000)
  
  pinMode(PA11, PWM);
  pwmWrite(PA11, 10);

//  timer2PWMSetup();

  pinMode(HOLD_BUTTON, INPUT);
  pinMode (ENCODER_OUT_A,INPUT);
  pinMode (ENCODER_OUT_B,INPUT);
  encoderALastState = digitalRead(ENCODER_OUT_A);   

  pinMode(SINGLE_SHOT_BUTTON ,INPUT_PULLUP);

}

void loop()  {
  
  if ( !triggerHeld  )  {
      // Wait for trigger
      trigger();
  
      if (triggered)   {
          // Take our samples
          
          takeSamples();
          blinkLED();
          
          //Blank  out previous plot
          TFTSamplesClear(BEAM_OFF_COLOUR);
    
          // Show the showGraticule
          showGraticule();
    
          //Display the samples
          TFTSamples(BEAM1_COLOUR);
          displayTime = (micros() - displayTime);
          
          // Display the Labels ( uS/Div, Volts/Div etc).
          showLabels();
          displayTime = micros();
  
      } else {
            showGraticule();
      }
  } else {
      readEncoder();
  }
  
  readHoldButton();
  readSingleShotButton();
  // Wait before allowing a re-trigger
  delay(retriggerDelay);
}

void showGraticule(){
  TFT.drawRect(0, 0, myHeight, myWidth, GRATICULE_COLOUR);
  // Dot grid - ten distinct divisions (9 dots) in both X and Y axis.
  for (uint16_t TicksX = 1; TicksX < 10; TicksX++){
    for (uint16_t TicksY = 1; TicksY < 10; TicksY++)
    {
      TFT.drawPixel(  TicksX * (myHeight / 10), TicksY * (myWidth / 10), GRATICULE_COLOUR);
    }
  }
  
  /*
  // Horizontal and Vertical centre lines 5 ticks per grid square with a longer tick in line with our dots
  for (uint16_t TicksX = 0; TicksX < myWidth; TicksX += (myHeight / 50))
  {
    if (TicksX % (myWidth / 10) > 0 )
    {
      TFT.drawFastHLine(  (myHeight / 2) - 1 , TicksX, 3, GRATICULE_COLOUR);
    }
    else
    {
      TFT.drawFastHLine(  (myHeight / 2) - 3 , TicksX, 7, GRATICULE_COLOUR);
    }

  }
  for (uint16_t TicksY = 0; TicksY < myHeight; TicksY += (myHeight / 50) )
  {
    if (TicksY % (myHeight / 10) > 0 )
    {
      TFT.drawFastVLine( TicksY,  (myWidth / 2) - 1 , 3, GRATICULE_COLOUR);
    }
    else
    {
      TFT.drawFastVLine( TicksY,  (myWidth / 2) - 3 , 7, GRATICULE_COLOUR);
    }
  }
  */
}

void setADCs () {
  //  const adc_dev *dev = PIN_MAP[analogInPin].adc_device;
  int pinMapADCin = PIN_MAP[analogInPin].adc_channel;
  adc_set_sample_rate(ADC1, ADC_SMPR_1_5); //=0,58uS/sample.  ADC_SMPR_13_5 = 1.08uS - use this one if Rin>10Kohm,
  adc_set_sample_rate(ADC2, ADC_SMPR_1_5);    // if not may get some sporadic noise. see datasheet.

  //  adc_reg_map *regs = dev->regs;
  adc_set_reg_seqlen(ADC1, 1);
  ADC1->regs->SQR3 = pinMapADCin;
  ADC1->regs->CR2 |= ADC_CR2_CONT; // | ADC_CR2_DMA; // Set continuous mode and DMA
  ADC1->regs->CR1 |= ADC_CR1_FASTINT; // Interleaved mode
  ADC1->regs->CR2 |= ADC_CR2_SWSTART;

  ADC2->regs->CR2 |= ADC_CR2_CONT; // ADC 2 continuos
  ADC2->regs->SQR3 = pinMapADCin;
}


// Crude triggering on positive or negative or either change from previous to current sample.
void trigger()  {
  triggered = false;
  switch (triggerType) {
    case 1:
      triggerNegative() ;
      break;
    case 2:
      triggerPositive() ;
      break;
    case 3:
      singleShot() ;
      break;
    case 4:
      triggerBoth() ;
      break;
    default:
      triggered = true;
      break;
  }
}

void triggerBoth()  {
  triggerPoints[0] = analogRead(analogInPin);
  while(triggered == false){
    triggerPoints[1] = analogRead(analogInPin);
    if ( ((triggerPoints[1] < TRIGGER_VALUE) && (triggerPoints[0] > TRIGGER_VALUE)) ||
         ((triggerPoints[1] > TRIGGER_VALUE) && (triggerPoints[0] < TRIGGER_VALUE)) ){
      triggered = true;
    }
    triggerPoints[0] = triggerPoints[1]; //analogRead(analogInPin);
  }
}

void triggerPositive() {
  triggerPoints[0] = analogRead(analogInPin);

  uint32_t time = micros();
  bool printOnce = false;
  
  while(triggered == false ){
    
    if(printOnce == false){
        Serial.println("trg +");
        printOnce = true;  
    }
    
    triggerPoints[1] = analogRead(analogInPin);
    if ((triggerPoints[1] > TRIGGER_VALUE) && (triggerPoints[0] < TRIGGER_VALUE) ){
      triggered = true;
    }
    triggerPoints[0] = triggerPoints[1]; //analogRead(analogInPin);
  }
  Serial.println("free");
  
  if( _singleShot){
      Serial.print("SS: "); Serial.println(_singleShot);
     _singleShot = false;
      triggerHeld = true;
      triggerType = 5;
  }

}

void triggerNegative() {
  triggerPoints[0] = analogRead(analogInPin);
  while(triggered == false){
    triggerPoints[1] = analogRead(analogInPin);
    if ((triggerPoints[1] < TRIGGER_VALUE) && (triggerPoints[0] > TRIGGER_VALUE) ){
      triggered = true;
    }
    triggerPoints[0] = triggerPoints[1]; //analogRead(analogInPin);
  }
}

void singleShot() {
    triggerType = 2;
    _singleShot = true;
    
}

void incEdgeType() {
  triggerType += 1;
  if (triggerType > 2)
  {
    triggerType = 0;
  }
}

void clearTFT() {
  TFT.fillScreen(BEAM_OFF_COLOUR);                // Blank the display
}

void blinkLED() {
#if defined BOARD_LED
  digitalWrite(BOARD_LED, LOW);
  delay(10);
  digitalWrite(BOARD_LED, HIGH);
#endif

}

// Grab the samples from the ADC
// Theoretically the ADC can not go any faster than this.
//
// According to specs, when using 72Mhz on the MCU main clock,the fastest ADC capture time is 1.17 uS. As we use 2 ADCs we get double the captures, so .58 uS, which is the times we get with ADC_SMPR_1_5.
// I think we have reached the speed limit of the chip, now all we can do is improve accuracy.
// See; http://stm32duino.com/viewtopic.php?f=19&t=107&p=1202#p1194

void takeSamples () {
  // This loop uses dual interleaved mode to get the best performance out of the ADCs
  //

  dma_init(DMA1);
  dma_attach_interrupt(DMA1, DMA_CH1, DMA1_CH1_Event);

  adc_dma_enable(ADC1);
  dma_setup_transfer(DMA1, DMA_CH1, &ADC1->regs->DR, DMA_SIZE_32BITS,
                     dataPoints32, DMA_SIZE_32BITS, (DMA_MINC_MODE | DMA_TRNS_CMPLT));// Receive buffer DMA
  dma_set_num_transfers(DMA1, DMA_CH1, maxSamples / 2);
  dma1_ch1_Active = 1;
  //  regs->CR2 |= ADC_CR2_SWSTART; //moved to setADC
  dma_enable(DMA1, DMA_CH1); // Enable the channel and start the transfer.
  //adc_calibrate(ADC1);
  //adc_calibrate(ADC2);
  samplingTime = micros();
  while (dma1_ch1_Active);
  samplingTime = (micros() - samplingTime);

  dma_disable(DMA1, DMA_CH1); //End of trasfer, disable DMA and Continuous mode.
  // regs->CR2 &= ~ADC_CR2_CONT;

}

void TFTSamplesClear (uint16_t beamColour)  {
  for (signalX=1 ; signalX < myWidth - 2; signalX++)
  {
    //use saved data to improve speed
    TFT.drawLine (  dataPlot[signalX-1], signalX, dataPlot[signalX] , signalX + 1, beamColour) ;
  }
}

void TFTSamples (uint16_t beamColour) {
  //calculate first sample
  signalY =  ((myHeight * dataPoints[0 * ((endSample - startSample) / (myWidth * timeBase / 100)) + 1]) / ANALOG_MAX_VALUE) * (yZoomFactor / 100) + yPosition;
  dataPlot[0]=signalY * 90 / 100 + 1;
  
  for (signalX=1 ; signalX < myWidth - 2; signalX++)  {
    // Scale our samples to fit our screen. Most scopes increase this in steps of 5,10,25,50,100 250,500,1000 etc
    // Pick the nearest suitable samples for each of our myWidth screen resolution points
//    signalY1 = ((myHeight * dataPoints[(signalX + 1) * ((endSample - startSample) / (myWidth * timeBase / 100)) + 1]) / ANALOG_MAX_VALUE) * (yZoomFactor / 100) + yPosition ;
    signalY1 = (myHeight * dataPoints[startSample + round(signalX * timeBase/0.58/myWidth) + 1] / ANALOG_MAX_VALUE) * (yZoomFactor / 100) + yPosition ;
    dataPlot[signalX] = signalY1 * Y_SCALE / 100 + 1;
    TFT.drawLine (  dataPlot[signalX-1], signalX, dataPlot[signalX] , signalX + 1, beamColour) ;
    signalY = signalY1;
  }
}

/*
// Run a bunch of NOOPs to trim the inter ADC conversion gap
void sweepDelay(unsigned long sweepDelayFactor) {
  volatile unsigned long i = 0;
  for (i = 0; i < sweepDelayFactor; i++) {
    __asm__ __volatile__ ("nop");
  }
}
*/

void showLabels() {
  TFT.setRotation(LANDSCAPE);
  TFT.setTextSize(1);
  TFT.setCursor(10, 190);
  // TFT.print("Y=");
  //TFT.print((samplingTime * xZoomFactor) / maxSamples);
  TFT.print(float (float(samplingTime) / float(maxSamples)));

  TFT.setTextSize(1);
  TFT.print(" uS/Sample  ");
  TFT.print(maxSamples);
  TFT.print(" samples ");
//  TFT.setCursor(10, 190);
//  TFT.print(displayTime);
  TFT.print(float (1000000 / float(displayTime)));
  TFT.print(" fps    ");
  TFT.setTextSize(2);
  TFT.setCursor(10, 210);
  TFT.print("0.3");
  TFT.setTextSize(1);
  TFT.print(" V/Div ");
  TFT.setTextSize(1);

  TFT.print("timeBase=");
  TFT.print(timeBase);
  TFT.print(" yzoom=");
  TFT.print(yZoomFactor);
  TFT.print(" ypos=");
  TFT.print(yPosition);
  //showTime();
  TFT.setRotation(PORTRAIT);//
}

void readSingleShotButton(){
    if( digitalRead(SINGLE_SHOT_BUTTON) == LOW) {
        triggerType = 3;
        Serial.println("single shot set");
    }
}

void readEncoder(){
    encoderAState = digitalRead(ENCODER_OUT_A);
    showGraticule();
    while( digitalRead(ENCODER_OUT_A) == encoderALastState);

    if (encoderAState != encoderALastState){     
       // If the outputB state is different to the outputA state, that means the encoder is rotating clockwise
       if (digitalRead(ENCODER_OUT_B) != encoderAState) { 
         scrollLeft();
       } else {
         scrollRight();
       }
    } 
    
    encoderALastState = encoderAState; // Updates the previous state of the outputA with the current state
}

void readHoldButton() {
  
  int buttonState = digitalRead(HOLD_BUTTON);
//  while(digitalRead(HOLD_BUTTON) == LOW);

  if(buttonState == LOW){
    toggleHold();
    Serial.print("trigger held: ");
    Serial.print(triggerHeld);
    
    delay(50);
  }
  
}

void toggleHold() {
  triggerHeld ^= 1 ;
}

void decreaseTimebase() {
  clearTrace();
  /*
  sweepDelayFactor =  sweepDelayFactor / 2 ;
  if (sweepDelayFactor < 1 ) {

    serial_debug.print("Timebase=");
    sweepDelayFactor = 1;
  }
  */
  if (timeBase > 100)
  {
    timeBase -= 100;
  }
  showTrace();
//  serial_debug.print("# Timebase=");
//  serial_debug.println(timeBase);

}

void increaseTimebase() {
  clearTrace();
  serial_debug.print("# Timebase=");
  if (timeBase < 10000)
  {
    timeBase += 100;
  }
  //sweepDelayFactor = 2 * sweepDelayFactor ;
  showTrace();
//  serial_debug.print("# Timebase=");
//  serial_debug.println(timeBase);
}

void increaseZoomFactor() {
  clearTrace();
  if ( xZoomFactor < 21) {
    xZoomFactor += 1;
  }
  showTrace();
//  serial_debug.print("# Zoom=");
//  serial_debug.println(xZoomFactor);

}

void decreaseZoomFactor() {
  clearTrace();
  if (xZoomFactor > 1) {
    xZoomFactor -= 1;
  }
  showTrace();
  Serial.print("# Zoom=");
  Serial.println(xZoomFactor);
  //clearTFT();
}

void clearTrace() {
  TFTSamples(BEAM_OFF_COLOUR);
  showGraticule();
}

void showTrace() {
  showLabels();
  TFTSamples(BEAM1_COLOUR);
}

void scrollRight() {
  
  if (startSample < (endSample - 320)) {
    clearTrace();
    startSample += 100;
    showTrace();
  }
  
  Serial.print("# startSample=");
  Serial.println(startSample);
}

void scrollLeft() {
  if (startSample > (120)) {
    clearTrace();
    startSample -= 100;
    showTrace();
  }
  Serial.print("# startSample=");
  Serial.println(startSample);

}

void increaseYposition() {

  if (yPosition < myHeight ) {
    clearTrace();
    yPosition ++;
    showTrace();
  }
  Serial.print("# yPosition=");
  Serial.println(yPosition);
}

void decreaseYposition() {

  if (yPosition > -myHeight ) {
    clearTrace();
    yPosition --;
    showTrace();
  }
  Serial.print("# yPosition=");
  Serial.println(yPosition);
}


void increaseTriggerPosition() {

  if (TRIGGER_VALUE < ANALOG_MAX_VALUE ) {
    TRIGGER_VALUE += TRIGGER_POSITION_STEP;  //trigger position step
  }
  Serial.print("# TriggerPosition=");
  Serial.println(TRIGGER_VALUE);
}

void decreaseTriggerPosition() {

  if (TRIGGER_VALUE > 0 ) {
    TRIGGER_VALUE -= TRIGGER_POSITION_STEP;  //trigger position step
  }
  Serial.print("# TriggerPosition=");
  Serial.println(TRIGGER_VALUE);
}



void toggleTestPulseOn () {
  pinMode(TEST_WAVE_PIN, OUTPUT);
  analogWrite(TEST_WAVE_PIN, 75);
//  serial_debug.println("# Test Pulse On.");
}

void toggleTestPulseOff () {
  pinMode(TEST_WAVE_PIN, INPUT);
//  serial_debug.println("# Test Pulse Off.");
}

uint16 timer_set_period(HardwareTimer timer, uint32 microseconds) {
  if (!microseconds) {
    timer.setPrescaleFactor(1);
    timer.setOverflow(1);
    return timer.getOverflow();
  }

  uint32 cycles = microseconds * (72000000 / 1000000); // 72 cycles per microsecond

  uint16 ps = (uint16)((cycles >> 16) + 1);
  timer.setPrescaleFactor(ps);
  timer.setOverflow((cycles / ps) - 1 );
  return timer.getOverflow();
}

/**
* @brief Enable DMA requests
* @param dev ADC device on which to enable DMA requests
*/

void adc_dma_enable(const adc_dev * dev) {
  bb_peri_set_bit(&dev->regs->CR2, ADC_CR2_DMA_BIT, 1);
}


//not working
void timer2PWMSetup(){
    pinMode(PA1, PWM);
  
  // Timer2 PWM Configuration for 10 kHz
  
  // Timer frequency = (System Clock / (Prescaler + 1)) / (Auto-Reload Value + 1)
  uint32_t systemClock = 72000000;  // STM32F103 runs at 72 MHz by default
  uint32_t pwmFreq = 10000;         // Desired PWM frequency (10 kHz)
  
  // Calculate prescaler and auto-reload register values
  uint32_t prescaler = 72;  // Timer prescaler calculation
  uint32_t autoReload = 100 - 1;
  
  // Set the prescaler and auto-reload register values for Timer 2
  TIMER2->regs.adv->PSC = prescaler;  // Set the prescaler
  TIMER2->regs.adv->ARR = autoReload; // Set the auto-reload value for period
  
  // Enable PWM mode on the selected Timer2 channel (TIMER2_CH1 for PA0 or TIMER2_CH2 for PA1)
  // Enable PWM on TIMER2_CH2 (PA1)
  TIMER2->regs.adv->CCMR1 |= (0x0060 << 12);  // Set to PWM mode 1 (Active high)
  TIMER2->regs.adv->CCER |= (0x0010 << 4);       // Enable output on Channel 2


  // Enable Timer 2
TIMER2->regs.adv->CR1 |= (0x0001 << 0);   // Start the timer
  
  // Set the PWM duty cycle to 50% (you can change this value as needed)
  analogWrite(PA1, 75);  // 50% duty cycle (range: 0-255)
}

/**
* @brief Disable DMA requests
* @param dev ADC device on which to disable DMA requests
*/

void adc_dma_disable(const adc_dev * dev) {
  bb_peri_set_bit(&dev->regs->CR2, ADC_CR2_DMA_BIT, 0);
}

static void DMA1_CH1_Event() {
  dma1_ch1_Active = 0;
}


void sleepMode()  {
  serial_debug.println("# Nighty night!");
  // Set PDDS and LPDS bits for standby mode, and set Clear WUF flag (required per datasheet):
  PWR_BASE->CR |= PWR_CR_CWUF;
  PWR_BASE->CR |= PWR_CR_PDDS;

  // set sleepdeep in the system control register
  SCB_BASE->SCR |= SCB_SCR_SLEEPDEEP;

  // Now go into stop mode, wake up on interrupt
  // disableClocks();
  asm("wfi");
}
