#include <Arduino_FreeRTOS.h>
#include <freeRTOSVariant.h>
#include <avr/power.h>
#include <avr/sleep.h> 

typedef union {
  uint16_t number;
  uint8_t bytes[2];
} UINT16_t;

#define  pwPin1  5
#define  VOLTAGE_THRESHOLD  3.0 // 3v is when Li-Ion batteries run out of charge 
#define  ADC     A7
#define  R1     47000
#define  R2     47000

volatile bool  lowVoltageFlag = 0;
TaskHandle_t TaskSendHandler;
//TaskHandle_t taskMeasureHandler;


void TaskSend( void *pvParameters );
void TaskMeasure( void *pvParameters );

void setup() {
 
    // Digital Input Disable on Analogue Pins
  // When this bit is written logic one, the digital input buffer on the corresponding ADC pin is disabled.
  // The corresponding PIN Register bit will always read as zero when this bit is set. When an
  // analogue signal is applied to the ADC7..0 pin and the digital input from this pin is not needed, this
  // bit should be written logic one to reduce power consumption in the digital input buffer.
   
  #if defined(__AVR_ATmega640__) || defined(__AVR_ATmega1280__) || defined(__AVR_ATmega1281__) || defined(__AVR_ATmega2560__) || defined(__AVR_ATmega2561__) // Mega with 2560
  DIDR0 = 0xFF;
  DIDR2 = 0xFF;
  #elif defined(__AVR_ATmega644P__) || defined(__AVR_ATmega644PA__) || defined(__AVR_ATmega1284P__) || defined(__AVR_ATmega1284PA__) // Goldilocks with 1284p
  DIDR0 = 0xFF;
  #elif defined(__AVR_ATmega328P__) || defined(__AVR_ATmega168__) || defined(__AVR_ATmega8__) // assume we're using an Arduino with 328p
  DIDR0 = 0x3F;
  #elif defined(__AVR_ATmega32U4__) || defined(__AVR_ATmega16U4__) // assume we're using an Arduino Leonardo with 32u4
  DIDR0 = 0xF3;
  DIDR2 = 0x3F;
  #endif
  
  
  // Analogue Comparator Disable
  // When the ACD bit is written logic one, the power to the Analogue Comparator is switched off.
  // This bit can be set at any time to turn off the Analogue Comparator.
  // This will reduce power consumption in Active and Idle mode.
  // When changing the ACD bit, the Analogue Comparator Interrupt must be disabled by clearing the ACIE bit in ACSR.
  // Otherwise an interrupt can occur when the ACD bit is changed.
  ACSR &= ~_BV(ACIE);
  ACSR |= _BV(ACD);
  
  
  // CHOOSE ANY OF THESE <avr/power.h> MACROS THAT YOU NEED.
  // Any *_disable() macro can be reversed by the corresponding *_enable() macro.
  
  // Disable the Analog to Digital Converter module.
  power_adc_disable();
  
  // Disable the Serial Peripheral Interface module.
  power_spi_disable();
  
  // Disable the Two Wire Interface or I2C module.
  power_twi_disable();
  
  // Disable the Timer 0 module. millis() will stop working.
  power_timer0_disable();
  
  // Disable the Timer 1 module.
  power_timer1_disable();
  
  // Disable the Timer 2 module. Used for RTC in Goldilocks 1284p devices.
  power_timer2_disable();
  
  


  
  // initialize serial communication at 9600 bits per second:
  Serial.begin(9600); // uncomment when using wisol
  //Serial.begin(1000000); // comment when using wisol
   //Serial.println(portTICK_PERIOD_MS);
//  wait4Tx();
// Send_Sensors();
 xTaskCreate(
    TaskSend
    ,  (const portCHAR *) "Task Send"
    ,  400  // Stack size
    ,  NULL
    ,  4  // Priority
    ,  &TaskSendHandler );
 


  

  // Now the task scheduler, which takes over control of scheduling individual tasks, is automatically started.
}

void loop()
{
      // There are several macros provided in the  header file to actually put
    // the device into sleep mode.
    // See ATmega328p Datasheet for more detailed descriptions.
     
    // SLEEP_MODE_IDLE
    // SLEEP_MODE_ADC
    // SLEEP_MODE_PWR_DOWN
    // SLEEP_MODE_PWR_SAVE
    // SLEEP_MODE_STANDBY
    // SLEEP_MODE_EXT_STANDBY
//     Serial.println("loop");
//     wait4Tx();
    set_sleep_mode( SLEEP_MODE_PWR_DOWN );
     
    portENTER_CRITICAL();
    
    sleep_enable();
     
    // Only if there is support to disable the brown-out detection.
    // If the brown-out is not set, it doesn't cost much to check.
//    #if defined(BODS) && defined(BODSE)
//    sleep_bod_disable();
//    #endif
     
    portEXIT_CRITICAL();
    
    sleep_cpu(); // Good night.
   
    // Ugh. Yawn... I've been woken up. Better disable sleep mode.
    // Reset the sleep_mode() faster than sleep_disable();
    sleep_reset();
}

/*--------------------------------------------------*/
/*---------------------- Tasks ---------------------*/
/*--------------------------------------------------*/



void TaskSend(void *pvParameters)  // This is a task.
{
  (void) pvParameters;

  
/*

*/

  for (;;)
  {
  float voltage ;
  voltage = (getVoltage() * 3.3 / 1024) * (R1 + R2) / R1; //convert to battery voltage
  //Serial.println(voltage);
  if(voltage <= VOLTAGE_THRESHOLD){
    if(!lowVoltageFlag)
      lowVoltageFlag = 1;
  
    //Serial.println("flag1");
  }
  else{
    if(lowVoltageFlag)
      lowVoltageFlag = 0;
    //Serial.println("flag0");
    
  }
    //Serial.println("Send Task"); //deleteb whenn using wisol
    if(!lowVoltageFlag){
      Send_Sensors();
    //Serial.println("Sending..."); //for debugging delete when using wisol
    delayMicroseconds(500);
    }
    vTaskDelay(600000 /  portTICK_PERIOD_MS);  // 100 ticks = 45 secs  , 20 ticks = 12.3sec  -> time[secs] = 0.40875*ticks + 4.125 (for arduino pro mini)
       //vTaskDelay(30);              //1458ticks ~ 600secs
  }
}



void wait4Tx(){
  Serial.flush();
  // wait for transmit buffer to empty
  while ((UCSR0A & _BV (TXC0)) == 0)
    {}
  }

void Send_Sensors(){
  UINT16_t current_level;
 
  UINT16_t voltage  ;  
  current_level.number = get_level2();
  //Serial.print("Level: "); Serial.println(current_level.number);
  voltage.number = getVoltage();

  const uint8_t payloadSize = 12; //in bytes
//  byte* buf_str = (byte*) malloc (payloadSize);
  uint8_t buf_str[payloadSize];

  buf_str[0] = current_level.bytes[0];
  buf_str[1] = current_level.bytes[1];
  buf_str[2] = voltage.bytes[0];
  buf_str[3] = voltage.bytes[1];
  buf_str[4] = 0;
  buf_str[5] = 0;
  buf_str[6] = 0;
  buf_str[7] = 0;
  buf_str[8] = 0;
  buf_str[9] = 0;
  buf_str[10] = 0;
  buf_str[11] = 0;


  Send_Pload(buf_str);  
  //Serial.println(current_level.number);
  //wait4Tx();
//  free(buf_str);
}

uint16_t get_level2()
{
   //emitter and reciever integrated sensor
   float m1,m2,m3;
   uint16_t result;

  m1 = pulseIn(pwPin1, HIGH);
  //delay(2);
  m2 = pulseIn(pwPin1, HIGH);
  //delay(2);
  m3 = pulseIn(pwPin1, HIGH);

  result = (int)round((((m1/147) + (m2/147) + (m3/147))/3)*2.57);  
 // Serial.println(result);
  return result;
}

void Send_Pload(uint8_t *sendData) {

  char bufer[32];
  char bufer2[24]={0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0};
  const char *header = "AT$SF=";
  const char *ending = "\n";

  strcpy(bufer, header);

  for (int i = 0; i <11; i++)
  {
       bufer2[2*i] = c2h(sendData[i] >> 4);
       bufer2[2*i + 1] = c2h(sendData[i]);
  }

  strcat(bufer,bufer2);
  strcat(bufer,ending);


    //Awakening the Sigfox Module
    digitalWrite(4, HIGH);
    delayMicroseconds(100);
    //Reset del canal para asegurar que manda en la frecuencia correcta
    Serial.print("AT$RC\n");
    wait4Tx();
    //************************
    //Enviamos la informacion por sigfox
    Serial.print(bufer);
    wait4Tx();
    //deshabilitamos el modulo Sigfox
    digitalWrite(4, LOW);
    delayMicroseconds(100);
    
}
char c2h(char c)
{ return "0123456789ABCDEF"[0x0F & (unsigned char)c];
}

uint16_t getVoltage(){
  float voltage;
  power_adc_enable();
  for(uint8_t i = 0; i < 10; i++)
     voltage += analogRead(ADC);

  voltage /= 10;
  
  power_adc_disable();
  return (uint16_t)round(voltage);
  
}

