typedef struct {
  uint8_t e1:3;
  uint8_t e2:3;
  uint8_t e3:3;
  uint8_t e4:3;
  uint8_t e5:3;
  uint8_t e6:3;
  uint8_t e7:3;
  uint8_t e8:3;
  
  
}bit_field;

typedef struct{
  uint8_t e:3;
}bla;
typedef struct {
  uint8_t byte1;
  uint8_t byte2;
  uint8_t byte3;
  
  
  
}bytes;

typedef union{
  bla _bit_fields[8];
  bytes _bytes;
}_events;

_events events;
void setup() {
  // put your setup code here, to run once:
  Serial.begin(9600);
  for(uint8_t i =0; i < 8; i++)
      events._bit_field.e1=4;

  events._bit_field.e1=4;
  events._bit_field.e2=2;
  events._bit_field.e3=2;
  events._bit_field.e4=2;
  events._bit_field.e5=2;
  events._bit_field.e6=2;
  events._bit_field.e7=2;
  events._bit_field.e8=7;
  

}

void loop() {
  // put your main code here, to run repeatedly:
  Serial.println("------------------");
  Serial.println(events._bytes.byte1);
  Serial.println(events._bytes.byte2);
  Serial.println(events._bytes.byte3);
}
