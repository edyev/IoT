/*
      Programa de flujo por evento v2.6 ->STABLE VERSION

      Rotoplas(c). July 31, 2018.

      Gilberto Mendoza Chavez
      Dennis Alberto Mendoza Solís
      dms.albertmend@gmail.com

      Config: #define portUSE_WDTO      WDTO_15MS
*/
#include <Arduino_FreeRTOS.h>
#include <avr/interrupt.h>
//#include <EEPROM.h>
#include <avr/pgmspace.h>


#define interruptPin 3
#define hardwareCounterPin  5
#define wisolSleep 4
#define logAddress 10
TaskHandle_t TaskHandle;

uint8_t event_buf[12] = {0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0};
uint8_t event_index = 0;
volatile bool event_active = false;
//bool terminate_flag = 0;
uint8_t trig_counter = 0;

enum parse_keys{
  FLOW_50,
  FLOW_100, 
  FLOWRATE = 12,
  LEVEL}
  parse_flag; 

enum state{
  SETUP = 0x0A,
  SEND = 0x1A,
  FLOW = 0x2A,
  TRIGGER = 0x3A,
  ACTIVE = 0x4A
}_state;
typedef union {
  uint16_t number;
  uint8_t bytes[2];
}   UINT16_t;

void setup()
{
  /*Enable serial communication*/
  Serial.begin(9600);
  /*Setup for Timer1 so it can work as hardware pulse counter*/
  timerCounterSetup();
  /*Create task that sends the data through Sigfox, Required Stack: 134*/
  xTaskCreate(TaskSend, "TaskSend", 134, NULL, 1, NULL);
  /*Create task that reads the flow sensor, Required Stack 100*/
  xTaskCreate(TaskFlow, "TaskFlow", 100, NULL, 2, &TaskHandle);
  /*Suspend TaskFlow until it is triggered*/
  vTaskSuspend(TaskHandle);
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


    if (!(event_active))//(event_index)// && !(event_active)) //&& (TCNT1 > 0))//
    {
      /*Enable interrupt from interruptPin*/
      attachInterrupt(digitalPinToInterrupt(interruptPin), trigger_int, RISING);
      /*Enable interrupts*/
      sei();
      //Serial.println("W");
    }

    vTaskDelay(60000 / portTICK_PERIOD_MS);
  }
}

/* TaskFlow with priority 1 */
static void TaskFlow(void* pvParameters)
{
  unsigned long duration;
  bool terminate_flag = 0;
  while (1) {

    /*Measure delay bewteen pulses from the flow sensor, 0.1 s timeout*/
    duration =  pulseIn(hardwareCounterPin, HIGH, 5000000);
    /*If duration is in the range, then we have activity*/
    event_active = ((duration > 0) && (duration  < 5000000)); //60000
    //Serial.println(event_active);


    if (!(event_active)  && (event_index < 12))
    {
      //EEPROM.write(logAddress, ACTIVE);
      if (TCNT1 > 20) {


        //Serial.println("F");

        /*Save data in event_buf*/
        event_buf[event_index] = pulses_2_lt(TCNT1);
        /*Restart Timer1 Count*/
        TCNT1 = 0;
        //Serial.print("event_index: ");
        //Serial.println(event_index);
        event_index += 1;
        /*Event finished*/
        terminate_flag = 1;

        /*Enable interrupts*/
        attachInterrupt(digitalPinToInterrupt(interruptPin), trigger_int, RISING);
        sei();

      }
      trig_counter = 0;
      /*Suspend this task*/
      vTaskSuspend(NULL);
    }

    /*Wait for 3 ticks and continue*/
    vTaskDelay(2);

  }
}

void timerCounterSetup() {
  //EEPROM.write(logAddress, SETUP);
  /*Declare our interrupt pin and hardware counter pin as inputs*/
  pinMode(interruptPin, INPUT);
  pinMode(hardwareCounterPin, INPUT);
  pinMode(wisolSleep, OUTPUT);
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
  delay(20);
  
}
void trigger_int() {

  trig_counter++;

  if (trig_counter > 10) //This condition seems to help with very small flows which used to cause problems
  {
    //EEPROM.write(logAddress, TRIGGER);
    //Serial.println("S");
    /*Disable interrupts from interruptPin*/
    detachInterrupt(digitalPinToInterrupt(interruptPin));
    /*Disable interrupts*/
    cli();
    /*Resume TaskFlow*/
    if (xTaskResumeFromISR(TaskHandle)) // If the taskYield is required then trigger the same.
    {
      /*Select the highest priority task that is in the Ready state to run*/
      taskYIELD();
    }
    //vTaskResume(TaskHandle);


    if (!(event_active))//(event_index)// && !(event_active)) //&& (TCNT1 > 0))//
    {
      /*Enable interrupt from interruptPin*/
      attachInterrupt(digitalPinToInterrupt(interruptPin), trigger_int, RISING);
      /*Enable interrupts*/
      sei();
      //Serial.println("W");
    }

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
  */

  //result = (uint8_t)round((0.0004*var - 0.0258)*10); //volume in dlt (decilitres)


  result = (uint8_t)round((0.0004 * var - 0.0258) * 20); //volume in 0.05 lt


  result = max(result, 0);

  return result;
}

void Send_Sensors() {
  /*Set the payloadSize to 12 bytes*/
  const uint8_t payloadSize = 12;
  uint8_t buf_str[payloadSize];
  /*Initialize buffers
  Store buffer in flash instead of RAM
  Helps with stability issues */
  char PROGMEM bufer[34]; 
  char bufer2[24] = {0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0};
  const char *header = "AT$SF=";
  const char *ending = "\n";
  /*copying events in buf_str*/
  //EEPROM.write(logAddress, SEND);
  //event_buf[11] = FLOW_100;
  memcpy(buf_str, event_buf, 12);
  Serial.print("Buf_str:");Serial.println(buf_str);
  event_index = 0;

  /*clearing event array*/
  for (int i = 0; i < 12; i++) {
    event_buf[i] = 0;
  }
  /*Copy header to bufer*/
  strcpy(bufer, header);
  Serial.print("buffer:");Serial.println(bufer);

  /*Fill buffer2 with the water flow info in hexadecimal*/
  for (int i = 0; i < 12; i++)
  {
    bufer2[2 * i] = c2h(buf_str[i] >> 4);
    bufer2[2 * i + 1] = c2h(buf_str[i]);
  }
  /*Add water flow info to bufer*/
  strcat(bufer, bufer2);
  Serial.print("buffer:");Serial.println(bufer);

  /*Add ending to buffer*/
  strcat(bufer, ending);
  
  /*Awakening the Sigfox Module*/
  
  digitalWrite(wisolSleep, HIGH);
  delayMicroseconds(100);
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
