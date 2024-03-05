#include "slf3s_4000b.h"

#define SLF3S_4000B     0x07030501
#define SLF3S_1300F     0x07030202

extern uint32_t  product_num;
extern uint64_t  serial_num;

extern uint16_t  value_flow;
extern uint16_t  value_temp;
extern uint16_t  sig_flag;

uint32_t  last_millis = 0;

uint8_t   res = 0;

void setup() {
  Serial.begin(115200);
  
  if (init_slf3s() != TRUE)
  {
    Serial.println("Error SLF3S init. Stopping proceed..");

    while (1)
      delay(1000);
  }

  Serial.println("SLF3S Complete soft reset");

  res = read_info_slf3s();   // Read product information (Product number & Serial number)  

  if (res == TRUE)
  {
    Serial.println();

    if (product_num == SLF3S_4000B)
      Serial.println("Checked the model : SLF3S-4000B");
    else if (product_num == SLF3S_1300F)
      Serial.println("Checked the model : SLF3S-1300F");
    else
      Serial.println("Unable check the model");

    Serial.print("Product num : 0x");
    Serial.println(product_num, HEX);
    Serial.print("Serial num : 0x");
    Serial.println(serial_num, HEX); 
    Serial.println();
  }
  else if (res == 0)
    Serial.println("Write error: info");
  else if (res == 2)
    Serial.println("Read error: info");
  else if (res == 3)
    Serial.println("CRC error: info");

  Serial.println("Complete init!");
  Serial.println();

  start_meas_slf3s();
  Serial.println("Start flow meter measurement..");
  Serial.println();

}

void loop() {
  if (millis() != last_millis)
  {
    last_millis = millis();

    if ((last_millis % 50) == 0)
      res = read_slf3s();      

    if ((last_millis % 1000) == 1)
    {
      if (res == TRUE)
      {
        Serial.print("Flow: ");
        Serial.print(value_flow);
        Serial.print(" ml/min");

        Serial.print(" | Temp: ");
        Serial.print(value_temp);
        Serial.print(" deg C");

        Serial.print(" | Signal: 0x");
        Serial.print(sig_flag, HEX);
        Serial.print("\r\n");
      }
      else if (res == 2)
        Serial.println("CRC error: data");
      else
        Serial.println("CRC error: data");
    }
  }
  else
    return;
}