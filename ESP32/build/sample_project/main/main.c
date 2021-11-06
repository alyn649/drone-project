#include <stdio.h>

#include "sdkconfig.h"
#include "driver/rmt.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_system.h"
#include "esp_spi_flash.h"

// Internal RMT channels allocation to each motor
#define RMT_TX_CHANNEL_M1 RMT_CHANNEL_0
#define RMT_TX_CHANNEL_M2 RMT_CHANNEL_1
#define RMT_TX_CHANNEL_M3 RMT_CHANNEL_2
#define RMT_TX_CHANNEL_M4 RMT_CHANNEL_3

// Motor allocation to exteral GPIO
#define M1_PIN 5
#define M2_PIN 18
#define M3_PIN 19
#define M4_PIN 21

// Functions
void setPacket(rmt_item32_t packetRMT[18], uint16_t val);
void motorRMTSetup();
void motorThrottles(uint16_t m1Throttle, uint16_t m2Throttle, uint16_t m3Throttle, uint16_t m4Throttle);

static const rmt_item32_t highBit = {{{ 25, 1, 8, 0 }}};
static const rmt_item32_t lowBit = {{{ 12, 1, 21, 0 }}};
static const rmt_item32_t pause = {{{ 690, 0, 3, 0 }}}; // 693 -> 21 bit down time
static const rmt_item32_t eotBit = {{{ 0, 1, 0, 0 }}};

rmt_item32_t m1Packet[18];
rmt_item32_t m2Packet[18];
rmt_item32_t m3Packet[18];
rmt_item32_t m4Packet[18];

int sizeOfPacket;

void app_main(void)
{
    // Setup 
    // Setup the RMT device
    motorRMTSetup();

    while (1) {
        // Superloop
        vTaskDelay(1);

        motorThrottles(48, 1000, 512, 0);
    }
}

void motorThrottles(uint16_t m1Throttle, uint16_t m2Throttle, uint16_t m3Throttle, uint16_t m4Throttle) {
    // Convert 1 to 2000 throttle range to rmt packet 
    setPacket(m1Packet, m1Throttle);
    setPacket(m2Packet, m2Throttle);
    setPacket(m3Packet, m3Throttle);
    setPacket(m4Packet, m4Throttle);

    // Write new timings to RMT memory
    rmt_fill_tx_items(RMT_TX_CHANNEL_M1, m1Packet, sizeOfPacket, 0);
    rmt_fill_tx_items(RMT_TX_CHANNEL_M2, m2Packet, sizeOfPacket, 0);
    rmt_fill_tx_items(RMT_TX_CHANNEL_M3, m3Packet, sizeOfPacket, 0);
    rmt_fill_tx_items(RMT_TX_CHANNEL_M4, m4Packet, sizeOfPacket, 0);
}

// Function to setup all 4 motor RMT channels
void motorRMTSetup() {
    // Configs for each motor 
    rmt_config_t m1config = RMT_DEFAULT_CONFIG_TX(M1_PIN, RMT_TX_CHANNEL_M1);
    rmt_config_t m2config = RMT_DEFAULT_CONFIG_TX(M2_PIN, RMT_TX_CHANNEL_M2);
    rmt_config_t m3config = RMT_DEFAULT_CONFIG_TX(M3_PIN, RMT_TX_CHANNEL_M3);
    rmt_config_t m4config = RMT_DEFAULT_CONFIG_TX(M4_PIN, RMT_TX_CHANNEL_M4);

     // 10MHz clock, 100ns ticks
    m1config.clk_div = 8;
    m2config.clk_div = 8;
    m3config.clk_div = 8;
    m4config.clk_div = 8;

    // Set config files
    rmt_config(&m1config);
    rmt_config(&m2config);
    rmt_config(&m3config);
    rmt_config(&m4config);

    // Install driver for each channel
    rmt_driver_install(m1config.channel, 0, 0);
    rmt_driver_install(m2config.channel, 0, 0);
    rmt_driver_install(m3config.channel, 0, 0);
    rmt_driver_install(m4config.channel, 0, 0);

    // Enable loopback mode on all channels
    rmt_set_tx_loop_mode(RMT_TX_CHANNEL_M1, true);
    rmt_set_tx_loop_mode(RMT_TX_CHANNEL_M2, true);
    rmt_set_tx_loop_mode(RMT_TX_CHANNEL_M3, true);
    rmt_set_tx_loop_mode(RMT_TX_CHANNEL_M4, true);

    // Initilsie the motor packets to a no-fly value 
    setPacket(m1Packet, 0);
    setPacket(m2Packet, 0);
    setPacket(m3Packet, 0);
    setPacket(m4Packet, 0);

    // Single time computation of packet size (never changes)
    sizeOfPacket = sizeof(m1Packet) / sizeof(m1Packet[0]);

    // Start each channel
    rmt_tx_start(RMT_TX_CHANNEL_M1, true);
    rmt_tx_start(RMT_TX_CHANNEL_M2, true);
    rmt_tx_start(RMT_TX_CHANNEL_M3, true);
    rmt_tx_start(RMT_TX_CHANNEL_M4, true);

    // Write intial packets
    rmt_fill_tx_items(RMT_TX_CHANNEL_M1, m1Packet, sizeOfPacket, 0);
    rmt_fill_tx_items(RMT_TX_CHANNEL_M2, m2Packet, sizeOfPacket, 0);
    rmt_fill_tx_items(RMT_TX_CHANNEL_M3, m3Packet, sizeOfPacket, 0);
    rmt_fill_tx_items(RMT_TX_CHANNEL_M4, m4Packet, sizeOfPacket, 0);
}

// Function takes 0 to 2000 throttle range and sets the timings for RMT
void setPacket(rmt_item32_t packetRMT[18], uint16_t val) {
    // Check if value is a telemetry command
    bool telem = val < 48; 

     // Write in telem bit and left shift val
    uint16_t packet = (val << 5) | (telem << 4);

    // Compute checksum (every 4th bit xor'ed, masked with last 4 bits)
    packet |= ((packet >> 4) ^ (packet >> 8) ^ (packet >> 12)) & (0b1111);

    // Convert int to RMT bit timings
    for(int i = 0; i < 16; i++) {
        if(packet & (1 << (15-i))) {
            packetRMT[i] = highBit;
        } else {
            packetRMT[i] = lowBit;
        }
    }

    // 21 bit space and EOT
    packetRMT[16] = pause;
    packetRMT[17] = eotBit;
}