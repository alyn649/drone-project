
#include "Arduino.h"
#include "esp32-hal.h"

//
//      Bits encoded as pulses as follows:
//
//      "0":
//         +-------+              +--
//         |       |              |
//         |       |              |
//         |       |              |
//      ---|       |--------------|
//         +       +              +
//         | 1.25us|   2.08 us   |
//
//      "1":
//         +-------------+       +--
//         |             |       |
//         |             |       |
//         |             |       |
//         |             |       |
//      ---+             +-------+
//         |    2.5us    | 0.83us|
//         |        3.33us       |

rmt_data_t led_data[16];

rmt_obj_t* rmt_send = NULL;

void setup() 
{
    Serial.begin(115200);
    
    if ((rmt_send = rmtInit(18, true, RMT_MEM_64)) == NULL)
    {
        Serial.println("init sender failed\n");
    }

    float realTick = rmtSetTick(rmt_send, 100);
    Serial.printf("real tick set to: %fns\n", realTick);

}

uint16_t val = 0;

bool bitCurr;

void loop() 
{
    for (int i = 0; i < 16; i++) {
      bitCurr = (val & (1 << i)) > 0;
                if (bitCurr) {
                    led_data[i].level0 = 1;
                    led_data[i].duration0 = 25;
                    led_data[i].level1 = 0;
                    led_data[i].duration1 = 8;
                } else {
                    led_data[i].level0 = 1;
                    led_data[i].duration0 = 12;
                    led_data[i].level1 = 0;
                    led_data[i].duration1 = 21;
                }
    }
    // Send the data
    rmtWrite(rmt_send, led_data, 16);

    val++;
    
    delay(100);
}
