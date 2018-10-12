/*
 *    Programa de flujo por evento v3.0 ->BETA VERSION 
 *    
 *    Rotoplas(c). August 29th, 2018.
 *    
 *    Gilberto Mendoza Chavez
 *    Dennis Alberto Mendoza Solís
 *    dms.albertmend@gmail.com
 *    Eduardo Vazquez Hernandez
 *    
 *    Config: #define portUSE_WDTO      WDTO_15MS
 *    #define configUSE_16_BIT_TICKS              0
*/

#include <Arduino_FreeRTOS.h>   
#include <avr/interrupt.h>
#include <semphr.h>
#include <EEPROM.h>

#define interruptPin 2
#define hardwareCounterPin  47
#define wisolSleep 5 //mega acqboard 5
#define breakChar 'C'
#define keyLocation 10
#define solenoidMask 0x01
#define solenoid LED_BUILTIN
#define R_LED 5
#define G_LED 6

TaskHandle_t TaskHandle;  //flow task handle
TaskHandle_t SendHandle;
TaskHandle_t DownlinkHandle;

SemaphoreHandle_t mutex;

TickType_t downlink_delay = 65536 / portTICK_PERIOD_MS;

uint8_t event_buf[12] = {0,0,0,0,0,0,0,0,0,0,0,0};
uint8_t event_index = 0;
volatile bool event_active = false;
//bool terminate_flag = 0;
uint8_t trig_counter = 0;
char rx_data[40] ;
uint8_t rx_count = 0;
uint8_t msg_flag = 0;
uint8_t dl_error = 0;
typedef union {
    uint16_t number;
    uint8_t bytes[2];
}   UINT16_t;

void setup()
{   
    /*Enable serial communication*/
    Serial.begin(9600);
    /*Create mutext to prevent two tasks accessing serial port*/
    do{
      mutex = xSemaphoreCreateMutex();
      _delay(10);
    }
    while( mutex == NULL );
    /*Setup for Timer1 so it can work as hardware pulse counter*/   
    timerCounterSetup();
    /*Create task that sends the data through Sigfox, Required Stack: 134*/
    xTaskCreate(TaskDownlink, "TaskDownlink", 500, NULL, 3, &DownlinkHandle);
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

static void TaskDownlink(void* pvParameters){
  while(1){
      
     xSemaphoreTake(mutex, portMAX_DELAY);  
     detachInterrupt(digitalPinToInterrupt(interruptPin));
     Downlink();
     
     if(key_parse(rx_data)){
       
        digitalWrite(solenoid, HIGH);
        digitalWrite(G_LED, HIGH);//debug
        digitalWrite(R_LED, LOW); //debug
        }
        
     else{
        digitalWrite(solenoid, LOW);
        digitalWrite(G_LED, LOW); //debug
        digitalWrite(R_LED, HIGH); //debug
     }
     attachInterrupt(digitalPinToInterrupt(interruptPin), trigger_int, RISING);
     xSemaphoreGive(mutex);
     
     for(uint8_t i = 0; i < 18; i++)
        vTaskDelay(600000 / portTICK_PERIOD_MS); 
     
  }
}

/* TaskSend with priority 2 (Highest) */
static void TaskSend(void* pvParameters)
{
    while(1)
    {

        /*The process must be completed without interruptions*/
        xSemaphoreTake(mutex, portMAX_DELAY);
        taskENTER_CRITICAL();
        Send_Sensors();
        taskEXIT_CRITICAL();
        xSemaphoreGive(mutex);
        /*Wait for 10 minutes to send a new message to Sigfox. 
        1 sec =~ 62 tick units*/
        vTaskDelay(600000 / portTICK_PERIOD_MS); 
    }
}

/* TaskFlow with priority 1 */
static void TaskFlow(void* pvParameters)
{        
    unsigned long duration;
    bool terminate_flag = 0;
    while(1){

        /*Measure delay bewteen pulses from the flow sensor, 0.1 s timeout*/
        duration =  pulseIn(hardwareCounterPin, HIGH,200000); 
        /*If duration is in the range, then we have activity*/
        event_active = ((duration > 0) && (duration  < 50000)); //60000
        //Serial.println(event_active);
        
        
        if (!(event_active)  && (event_index < 12))
        {
            
            if(TCNT5>10){
             
                  //Serial.println("F");
                  
                  /*Save data in event_buf*/
                  event_buf[event_index] = pulses_2_lt(TCNT5);
                  /*Restart Timer1 Count*/
                  TCNT5 = 0;
                  //Serial.print("event_index: ");
                  //Serial.println(event_index);
                  event_index += 1; 
                  /*Event finished*/
                  terminate_flag = 1;
                
                 /*Enable interrupts*/
                  attachInterrupt(digitalPinToInterrupt(interruptPin), trigger_int, RISING);
                  sei();

            }
            trig_counter=0;
            /*Suspend this task*/
            vTaskSuspend(NULL);
        }
        
        /*Wait for 3 ticks and continue*/       
        vTaskDelay(30 / portTICK_PERIOD_MS);
        
    }
}

void timerCounterSetup(){
    /*Declare our interrupt pin and hardware counter pin as inputs*/
    pinMode(interruptPin, INPUT);
    pinMode(hardwareCounterPin, INPUT); 
    pinMode(wisolSleep, OUTPUT);
    pinMode(solenoid, OUTPUT);
    pinMode(G_LED, OUTPUT);
    pinMode(R_LED, OUTPUT);
    
    /*Guarantees Wisol is awake and ready for tasks*/
    digitalWrite(wisolSleep, LOW);
    delay(100);
    digitalWrite(wisolSleep, HIGH);
    /*Enable interrupt from interrupt pin in RISING edge, and trigger trigger_int*/
    attachInterrupt(digitalPinToInterrupt(interruptPin), trigger_int, RISING);
    /*Reset timer/counter control register A*/
    TCCR5A=0;             
    /*Configure Timer1 with external clock source on rising edge*/
    bitSet(TCCR5B ,CS52); 
    bitSet(TCCR5B ,CS51);  
    bitSet(TCCR5B ,CS50); 
    /*Restart Timer1 count*/ 
    TCNT5=0;
    delay(20);
}
void trigger_int() { 
    
    trig_counter++; 
    
    if (trig_counter > 10) //This condition seems to help with very small flows which used to cause problems
    { 
          
          /*Disable interrupts from interruptPin*/
          detachInterrupt(digitalPinToInterrupt(interruptPin));
          /*Disable interrupts*/
          cli();
          /*Resume TaskFlow*/
          if(xTaskResumeFromISR(TaskHandle)) // If the taskYield is required then trigger the same.
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

uint8_t pulses_2_lt(uint16_t var){

  uint8_t result;
  /*
   * Sunwoald 3/8 QC 
   * y = 0.0008x - 0.0145
   * where y: volume in lt and x: pulses
   */
   result = (uint8_t)round((0.0008*var - 0.0145)*10); //volume in dlt (decilitres) 

  /*
   * Sunwoald 1/4 QC 
   * y = 0.0004x - 0.0258
   * where y: volume in lt and x: pulses
   */
  //result = (uint8_t)round((0.0004*var - 0.0258)*10); //volume in dlt (decilitres)

  result = max(result,0);

  return result;
}

void Send_Sensors() {
    /*Set the payloadSize to 12 bytes*/
    const uint8_t payloadSize = 12; 
    uint8_t buf_str[payloadSize];
    /*Initialize buffers*/
    char bufer[32];
    char bufer2[24]={0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0};
    const char *header = "AT$SF=";
    const char *ending = "\n";
    /*copying events in buf_str*/
    memcpy(buf_str, event_buf, 12); 
    event_index = 0;
  
    /*clearing event array*/
    for(int i=0;i<12;i++){
        event_buf[i]=0;  
    }
    /*Copy header to bufer*/
    strcpy(bufer, header);
    /*Fill buffer2 with the water flow info in hexadecimal*/
    for (int i = 0; i <11; i++)
    {
         bufer2[2*i] = c2h(buf_str[i] >> 4);
         bufer2[2*i + 1] = c2h(buf_str[i]);
    }
    /*Add water flow info to bufer*/
    strcat(bufer,bufer2);
    /*Add ending to buffer*/
    strcat(bufer,ending);
    /*Awakening the Sigfox Module*/
    digitalWrite(wisolSleep, LOW); 
    /*Delay not in real time*/
    delay(70000); //HARD REQUIRMENT DO NOT MODIFY 
    digitalWrite(wisolSleep, HIGH);
    /*Channel reset to have the correct frequency*/
    Serial.print("AT$RC\n");
    /*Send data through sigfox*/
    Serial.print(bufer);
    Serial.flush();
    /*Wait for transmit buffer to empty*/
    while ((UCSR0A & _BV (TXC0)) == 0)
    {}
    /*Turn off the Sigfox  module*/
    Serial.print("AT$P=2\n");
    /*Disregard response to the command*/
    while(Serial.available())
      Serial.read();
    delayMicroseconds(100);
}

void Downlink(){
  char dataBuffer[40];
  char aux_buffer[24]= {0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0};
  const char *header = "AT$SF="; 
  rx_count = 0;
  msg_flag = 0;
  strcpy(dataBuffer, header);
  strcat(dataBuffer, "00000000000000000000beef");
  /*Flag to request downlink*/
  strcat(dataBuffer, ",1\n"); 
  digitalWrite(wisolSleep, LOW); 
  delay(1000); //HARD REQUIRMENT DO NOT MODIFY
  digitalWrite(wisolSleep, HIGH);
  Serial.print("AT$RC\n");
  delay(10);
  Serial.print(dataBuffer);
  delay(5000);
  /*The downlink request is responsed with the following
   * OK
   * OK
   * RX= 000000000
   */
   /*Loop to discard first OK response*/
   
  
  
  char _aux;
  /*Loop to extract the downlink data into global variable rx_data*/
  uint8_t timeout = 0;
  while(1){
  if(Serial.available()){
    
    _aux = Serial.read();
    if((_aux == 'R') || (timeout > 12000)) {
      
      uint8_t n = Serial.readBytesUntil('\n', rx_data,50);
      if (n < 8)
        dl_error = 1;
      else
        dl_error = 0;
      break;
      
      }
    
  }
  
  ++timeout;
  _delay(10);
  
  }   
}


bool key_parse(char* downlink){
  uint8_t key_location = 0;
  uint8_t _downlink[16];
  uint8_t count = 0;
  char hex_char ;
  uint8_t hex_num;
  for(uint8_t i = 2; i < sizeof(rx_data); ++i)
    if( downlink[i] != ' '){
      hex_char = downlink[i];
      sscanf(&hex_char, "%x", &hex_num);
      _downlink[count++] = hex_num;
   }
  /*Resets global rx buffer*/  
  for(uint8_t i = 0; i < sizeof(rx_data); ++i){
    //Serial.println(rx_data[i]);
    rx_data[i] = 'x';
  }
 
    
     // Serial.println(atoi(_downlink[1]));
 
  return (_downlink[keyLocation] & solenoidMask); 
}

void _delay(unsigned long int ms){
  unsigned long int time0 = millis();
  unsigned long int time1;
  
  do{

    time1 = millis();
   
  }while((time1 - time0) < ms);
}


