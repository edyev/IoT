/*
      Programa de flujo por evento v2.6 ->STABLE VERSION

      Rotoplas(c). July 31, 2018.

      Gilberto Mendoza Chavez
      Dennis Alberto Mendoza Solís
      dms.albertmend@gmail.com
      Eddie Vazquez Hernandez
      Config: #define portUSE_WDTO      WDTO_15MS
*/


#include <Arduino_FreeRTOS.h>
#include <avr/interrupt.h>
#include <semphr.h>
#include <avr/pgmspace.h>

#define interruptPin 3
#define hardwareCounterPin  5
#define wisolSleep 4
#define logAddress 10

TaskHandle_t TaskHandle;
SemaphoreHandle_t xSync = NULL;
QueueHandle_t xQueue, xEventQueue;
static TaskHandle_t xTaskToNotify = NULL;

volatile uint16_t duration ;
uint16_t time_counter = 0;

enum parse_keys{
  FLOW_50,
  FLOW_100, 
  FLOWRATE = 12,
  LEVEL}
  parse_flag; 

ISR(TIMER2_OVF_vect){
  uint16_t data = 60000;
  /*Timeout*/
  if(time_counter++ >= 7200)
    xQueueOverwriteFromISR(xQueue, (void*)&data,(TickType_t) 0 );
  
}

void setup()
{
  /*Enable serial communication*/
  Serial.begin(9600);
  /*Queue for sending pulse duration from ISR to TaskFlow*/ 
  xQueue = xQueueCreate(1, sizeof(uint16_t*));
  /*Queue for storing events */
  xEventQueue = xQueueCreate(12, sizeof(uint8_t*));
  /*if(xSync == NULL)
    Serial.println("Error creating semph");
  else
    Serial.println("Semph created");

  if(xEventQueue == NULL)
    Serial.println("eQueue error");
  else
    Serial.println("eQueue created");


  if(xQueue == NULL)
    Serial.println("Queue error");
  else
    Serial.println("Queue created");
      
  /*Setup for Timer1 so it can work as hardware pulse counter*/
  timerCounterSetup();
  
  /*Create task that sends the data through Sigfox, Required Stack: 134*/
  xTaskCreate(TaskSend, "TaskSend", 200, NULL, 2, NULL);
  /*Create task that reads the flow sensor, Required Stack 100*/
  xTaskCreate(TaskFlow, "TaskFlow", 200, NULL, 1, &TaskHandle);
  /*Suspend TaskFlow until it is triggered*/
}


void loop()
{
  /*Hooked to Idle Task, will run when CPU is Idle*/
}


/* TaskSend with priority 2 (Highest) */
static void TaskSend(void* pvParameters)
{
  while (1)
  {
    /*The process must be completed without interruptions*/
    taskENTER_CRITICAL();
    Send_Sensors();
    taskEXIT_CRITICAL();
    /*Wait for 10 minutes to send a new message to Sigfox.
      1 sec =~ 62 tick units*/
    vTaskDelay(600000 / portTICK_PERIOD_MS);
  }
}

/* TaskFlow with priority 1 */
static void TaskFlow(void* pvParameters)
{
  /*Receives pulse duration from ISR*/
  volatile uint16_t data; 
  /*Holds the volume*/
  volatile uint8_t volume;

  while(1){
      
      xQueuePeek(xQueue, &(data), (TickType_t )0);
      //Serial.print(".");Serial.println(data);
      if(data > 50000){
         if(TCNT1 > 10){
          volume = pulses_2_lt(TCNT1);
          if(uxQueueMessagesWaiting(xEventQueue) < 12 && volume > 0)
            xQueueSend(xEventQueue, (void*)&volume,(TickType_t )0);
          
            
          /*Serial.print("Vol->");
          Serial.println(volume);
          Serial.print("Size->");
          Serial.println(uxQueueMessagesWaiting(xEventQueue));
          

          
          
          
          Serial.println("Finished!");
           */
         }
          TCNT1 = 0;
          time_counter = 0;
          TCCR2B = 0;
          TCNT2 = 0;
          detachInterrupt(digitalPinToInterrupt(interruptPin));
          attachInterrupt(digitalPinToInterrupt(interruptPin), trigger_int, FALLING);
     
      }
      
  }
}

void timerCounterSetup() {
  //EEPROM.write(logAddress, SETUP);
  /*Declare our interrupt pin and hardware counter pin as inputs*/
  duration = 0;
  pinMode(interruptPin, INPUT);
  pinMode(hardwareCounterPin, INPUT);
  pinMode(wisolSleep, OUTPUT);
  bitSet(TIMSK2, 0);
  /*Enable interrupt from interrupt pin in RISING edge, and trigger trigger_int*/
  attachInterrupt(digitalPinToInterrupt(interruptPin), trigger_int, RISING);
  /*Reset timer/counter control register A*/
  TCCR1A = 0;
  /*Configure Timer1 with external clock source on rising edge*/
  bitSet(TCCR1B , CS12);
  bitSet(TCCR1B , CS11);
  bitSet(TCCR1B , CS10);
  /*Restart Timer1 count*/
  TCNT1 = 0;
  
  
}


void trigger_int() {
  /*This is the ISR for INT1*/
  BaseType_t *pxHigherPriorityTaskWoken;
  //Serial.println(EICRA, HEX);
  if (EICRA == 0x0C){
  detachInterrupt(digitalPinToInterrupt(interruptPin));
  //xSemaphoreGiveFromISR(xSync,&pxHigherPriorityTaskWoken  );
  bitSet(TCCR2B, CS21);
  //bitSet(TCCR2B, CS20);
  
  attachInterrupt(digitalPinToInterrupt(interruptPin), trigger_int, FALLING);
  
  }
  else{
    detachInterrupt(digitalPinToInterrupt(interruptPin));
    TCCR2B =0;

    duration = (time_counter << 8) | TCNT2;
    xQueueOverwriteFromISR(xQueue,(void*) &duration,pxHigherPriorityTaskWoken);
    time_counter = 0;
    TCNT2 = 0;
    bitSet(TCCR2B, CS21);
    //bitSet(TCCR2B, CS20);

    attachInterrupt(digitalPinToInterrupt(interruptPin), trigger_int, FALLING);
    
  }
  
}
char c2h(char c)
{
  /*Convert to hexadecimal*/
  return "0123456789ABCDEF"[0x0F & (unsigned char)c];
}

uint8_t pulses_2_lt(uint16_t var) {

  uint8_t result;
  /*
     Sunwoald 3/8 QC
     y = 0.0008x - 0.0145
     where y: volume in lt and x: pulses
  */
  //result = (uint8_t)round((0.0008*var - 0.0145)*10); //volume in dlt (decilitres)

  /*
     Sunwoald 1/4 QC
     y = 0.0004x - 0.0258
     where y: volume in lt and x: pulses
     result = (uint8_t)round((0.0004*var - 0.0258)*10); //volume in dlt (decilitres)
     64.5 < x otherwise result is negative
  */

  
  result = (uint8_t)round((0.0004 * var - 0.0258) * 20); //volume in 0.05 lt


  result = max(result, 0);

  return result;
}

void Send_Sensors() {
  /*Set the payloadSize to 12 bytes*/
  const uint8_t payloadSize = 12;
  /*Initialize buffers
  Store buffer in flash instead of RAM
  Helps with stability issues */

  char PROGMEM bufer[34]; 
  char bufer2[24] = {0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0};
  const char *header = "AT$SF=";
  const char *ending = "\n";
  /*copying events in buf_str*/
  
  volatile uint8_t data ;
  /*volatile uint16_t*/
  
  /*xQueueReceive(xEventQueue, &(data), (TickType_t)0);
  Serial.println(data);
  xQueueReceive(xEventQueue, &(data), (TickType_t)0);
  Serial.println(data);
  xQueueReceive(xEventQueue, &(data), (TickType_t)0);
  Serial.println(data);
  xQueueReceive(xEventQueue, &(data), (TickType_t)0);
  Serial.println(data);
  xQueueReceive(xEventQueue, &(data), (TickType_t)0);
  Serial.println(data);*/
  uint8_t n_events = uxQueueMessagesWaiting(xEventQueue);
  for(uint8_t i = 0; i < 12; i++){
     if(i < n_events)
       xQueueReceive(xEventQueue, &(data), (TickType_t)0);
     else
       data = 0;
     //Serial.println(data);
     bufer2[2 * i] = c2h(data >> 4);
     bufer2[2 * i + 1] = c2h(data);
  }
    

    
  xQueueReset(xEventQueue);
  /*clearing event array*/
  
  /*Copy header to bufer*/
  strcpy(bufer, header);

  /*Fill buffer2 with the water flow info in hexadecimal*/
 
  /*Add water flow info to bufer*/
  strcat(bufer, bufer2);

  /*Add ending to buffer*/
  strcat(bufer, ending);
  
  /*Awakening the Sigfox Module*/
  
  digitalWrite(wisolSleep, HIGH);
  delayMicroseconds(1000);
  /*Channel reset to have the correct frequency*/
  Serial.print(F("AT$RC\n"));
  /*Send data through sigfox*/
  Serial.print(bufer);
  Serial.flush();
  /*Wait for transmit buffer to empty*/
  while ((UCSR0A & _BV (TXC0)) == 0)
  {}
  /*Turn off the Sigfox  module*/
  digitalWrite(wisolSleep, LOW);
  delayMicroseconds(100);
  
}
