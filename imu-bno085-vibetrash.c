/**
 * Raspbery Pi Pico 2 W SPI Communication with ADAFRUIT BNO085 IMU BREAKOUT BOARD
 * by Henry Huy Vu.
 * 
 * The following is code to talk to a BNO085 accelerometer and gyroscope.
 * 
 * NOTE: The IMU device is capable of being driven at 3.3V AND 5V, 
   however for simplicity, the Pico GPIO (and therefore SPI) is driven at 3.3V.
   
   Connections on the Raspberry Pi Pico board and a generic BNO085 board, other
   boards may vary.

   3.3v (pin 36) -> VCC on BNO085 board
   GND (pin 38)  -> GND on BNO085 board
   GPIO 5 (pin 7) Chip select -> CS on BNO085 board
   GPIO 6 (pin 9) SCK/spi0_sclk -> SCL on BNO085 board
   GPIO 7 (pin 10) MOSI/spi0_tx -> DI on BNO085 board
   GPIO 20 (pin 27) SCK/spi0_sclk -> RST on BNO085 board
   GPIO 21 (pin 26) MOSI/spi0_tx -> INT on BNO085 board

   Note: SPI devices can have a number of different naming schemes for pins. See
   the Wikipedia page at https://en.wikipedia.org/wiki/Serial_Peripheral_Interface
   for variations.
*/

#include <stdio.h>
#include <string.h>
#include "pico/stdlib.h"
#include "pico/binary_info.h"
#include "hardware/spi.h"

// Define SPI pins for Pico (using SPI0)
#define SPI_PORT spi0
#define PIN_MISO 4      // SDA on BNO085 board
#define PIN_CS   5      // CS on BNO085 board
#define PIN_SCK  6      // SCL on BNO085 board
#define PIN_MOSI 7      // DI on BNO085 board

// Define BNO085 control pins
#define PIN_RST 21    // RST on BNO085 board
#define PIN_INT 20      // INT on BNO085 board

// --- SH-2 Protocol Definitions ---
// Based on common SH-2 usage and datasheets.

/** SHTP Header structure (4 bytes)
 * SPI transmits data MSB first and is byte oriented (all data is passed in 8-bit segments)
 * Any number of bytes can be transferred in a single transaction (chip select assertion)
 * All data is previxed with a 4-byte header:
 * Byte | Field
 * 0    | Length LSB
 * 1    | Length MSB
 * 2    | Channel
 * 3    | SeqNum
 */
typedef struct {
    uint8_t length_lsb;
    uint8_t length_msb;
    uint8_t channel;
    uint8_t seq_num;
} shtpHeader;

// === SHTP Command Registers ===
#define REGISTER_CHANNEL_COMMAND        0x00 // SHTP Command Channel
#define REGISTER_CHANNEL_EXECUTABLE     0x01 // Executable challen allows the host ot reset the BNO085 and provide details of its operating mode
#define REGISTER_CHANNEL_CONTROL    0x02 // Sensor Hub Channel (for hub-level commands)
#define REGISTER_CHANNEL_REPORTS    0x03 // Normal Sensor Data Channel
#define REGISTER_CHANNEL_WAKE_REPORTS   0x04 // Wakeup Sensor Data Channel
#define REGISTER_CHANNEL_GYRO       0x05 // Gyro-integrated Rotation Vector Channel

// === SHTP Report IDs ===
// Source: SH-2-Reference-Manual. Figure 32, Page 38.
// All the ways we can configure/talk to the BNO080. 
// Used for low level communication with the sensor (on channel 2)
#define SHTP_REPORT_ID_GET_FEATURE_REQUEST 0xFE
#define SHTP_REPORT_ID_SET_FEATURE_COMMAND 0xFD
#define SHTP_REPORT_ID_GET_FEATURE_RESPONSE 0xFC
#define SHTP_REPORT_ID_BASE_TIMESTAMP 0xFB
#define SHTP_REPORT_ID_TIMESTAMP_REBASE 0xFA
#define SHTP_REPORT_ID_PRODUCT_ID_REQUEST 0xF9
#define SHTP_REPORT_ID_PRODUCT_ID_RESPONSE 0xF8
#define SHTP_REPORT_ID_FRS_READ_REQUEST 0xF4
#define SHTP_REPORT_ID_FRS_READ_RESPONSE 0xF3
#define SHTP_REPORT_ID_COMMAND_REQUEST 0xF2
#define SHTP_REPORT_ID_COMMAND_RESPONSE 0xF1

// Feature IDs (for Data)
#define SHTP_FEATURE_REPORT_ID_ACCELEROMETER     0x01 // Accelerometer feature report ID
#define SHTP_FEATURE_REPORT_ID_GYROSCOPE         0x02 // Gyroscope feature report ID
#define SHTP_FEATURE_REPORT_ID_MAGNETOMETER      0x03 // Magnetometer feature report ID
#define SHTP_FEATURE_REPORT_ID_LINEAR_ACCEL      0x04 // Linear Acceleration feature report ID
#define SHTP_FEATURE_REPORT_ID_ROTATION_VECTOR   0x05 // Rotation Vector feature report ID
#define SHTP_FEATURE_REPORT_ID_GRAVITY           0x06 // Gravity feature report ID

#define SHTP_FEATURE_REPORT_ID_GAME_ROT_VECTOR   0x08 // Game Rotation Vector feature report ID
#define SHTP_FEATURE_REPORT_ID_GEOMAG_ROT_VECTOR 0x09

#define SHTP_FEATURE_REPORT_ID_TAP_DETECTOR      0x10
#define SHTP_FEATURE_REPORT_ID_STEP_COUNTER      0x11
#define SHTP_FEATURE_REPORT_ID_STABILITY_CLASSIFIER 0x13
#define SHTP_FEATURE_REPORT_ID_ACCELEROMETER_RAW 0x14 // Raw Accelerometer feature report ID
#define SHTP_FEATURE_REPORT_ID_PERSONAL_ACTIVITY_CLASSIFIER 0x1E

// === Sensor Metadata === 
// Source: SH-2-Reference-Manual. Figure 27, Page 30.
// These are used to read the metadata for each sensor type
#define FRS_RECORD_ID_ACCELEROMETER 0xE302
#define FRS_RECORD_ID_GYROSCOPE_CALIBRATED 0xE306
#define FRS_RECORD_ID_MAGNETIC_FIELD_CALIBRATED 0xE309
#define FRS_RECORD_ID_ROTATION_VECTOR 0xE30B

// === Command IDs ===
// Source: SH-2-Reference-Manual. Figure 42. Page 44.
// These are used to calibrate, initialize, set orientation, tare etc the sensor
#define COMMAND_ERRORS 1
#define COMMAND_COUNTER 2
#define COMMAND_TARE 3
#define COMMAND_INITIALIZE 4
#define COMMAND_DCD 6
#define COMMAND_ME_CALIBRATE 7
#define COMMAND_DCD_PERIOD_SAVE 9
#define COMMAND_OSCILLATOR 10
#define COMMAND_CLEAR_DCD 11

// Max buffer size for SHTP packets (in bytes)
#define MAX_PACKET_SIZE 300 // Sufficient for advertisement and sensor data


// --- Global Buffers ---
uint8_t rx_buffer[MAX_PACKET_SIZE];
uint8_t tx_buffer[MAX_PACKET_SIZE];

// === Helper Functions ===
// --- SPI Chip Select ---
static inline void cs_select() {
    asm volatile("nop \n nop \n nop");  // nop is a no-operation to consume one clock cycle
    gpio_put(PIN_CS, 0);  // Active low
    asm volatile("nop \n nop \n nop");
}
// --- SPI Chip Deselect ---
static inline void cs_deselect() {
    asm volatile("nop \n nop \n nop");
    gpio_put(PIN_CS, 1);
    asm volatile("nop \n nop \n nop");
}

// --- BNO085 Hardware Reset ---
void bno085_reset() {
    gpio_put(PIN_RST, 0); // Pull RST low (Active low)
    sleep_ms(10);        // Hold low for at least 10ms
    gpio_put(PIN_RST, 1); // Pull RST high
    sleep_ms(100);       // Wait for BNO085 to boot up
}

// === Send an SHTP packet ===
// data_len: length of the payload
// channel: SHTP channel
// seq_num: sequence number for the channel
void shtp_send_packet(uint16_t data_len, uint8_t channel, uint8_t seq_num) {
    // Construct the SHTP header
    shtpHeader header;   // Declares a variable named "header" of the type shtpHeader
    header.length_lsb = (uint8_t)(data_len & 0xFF);
    header.length_msb = (uint8_t)((data_len >> 8) & 0xFF);  // Bitwise right shift; Moves the 8 upper bits into the lower 8 bit position
    header.channel = channel;
    header.seq_num = seq_num;

    // Copy header to the beginning of the tx_buffer
    memcpy(tx_buffer, &header, sizeof(shtpHeader));

    // Send the packet via SPI
    cs_select();
    spi_write_blocking(SPI_PORT, tx_buffer, data_len + sizeof(shtpHeader));
    cs_deselect();
}


// === Read an SHTP packet ===
// Returns the number of bytes read, or 0 if no data is available/error
int shtp_read_packet(uint8_t* buffer, size_t buffer_size) {
    if (gpio_get(PIN_INT) == 1) {
        return 0; // No data ready
    }

    // Read only the header first to get the length
    uint8_t header_buffer[sizeof(shtpHeader)];
    cs_select();
    spi_read_blocking(SPI_PORT, 0x00, header_buffer, sizeof(shtpHeader));
    cs_deselect(); // Deselect after reading header

    // --- DEBUG: Print header bytes ---
    printf("Raw Header Bytes: ");
    for (int i = 0; i < sizeof(shtpHeader); i++) {
        printf("0x%02X ", header_buffer[i]);
    }
    printf("\n");
    // --- END DEBUG ---

    shtpHeader header;
    memcpy(&header, header_buffer, sizeof(shtpHeader));

    uint16_t payload_len = (header.length_msb << 8) | header.length_lsb;

    // Check if the received packet is valid and fits in our buffer
    // The total packet size is header size + payload size.
    if (payload_len > 0 && (payload_len + sizeof(shtpHeader)) <= buffer_size) {
        cs_select();    // Now read the actual payload
        // Read the payload data starting from the byte after the header.
        // The header itself is already in header_buffer, which we will copy to buffer.
        spi_read_blocking(SPI_PORT, 0x00, buffer + sizeof(shtpHeader), payload_len);
        cs_deselect(); // Deselect after reading payload

        // Copy the header to the beginning of the buffer for consistency with the rest of the code.
        memcpy(buffer, &header, sizeof(shtpHeader));

        printf("SHTP Packet received: Channel=%02X, SeqNum=%02X, PayloadLen=%d\n",
               header.channel, header.seq_num, payload_len);
        return payload_len + sizeof(shtpHeader); // Total bytes read (header + payload)
    } else if (payload_len == 0) {
        printf("No significant SHTP data received.\n"); // Added newline for clarity
        return 0;
    } else {
        // This indicates an issue: either the payload length is invalid, or it exceeds our buffer capacity.
        printf("Unexpected SHTP packet received: PayloadLen=%d, BufferSize=%d\n",
               payload_len, buffer_size - sizeof(shtpHeader));
        return 0;
    }
}

// === Request a Feature Report (e.g., accelerometer) ===
// feature_id: The ID of the feature to request
// report_interval_us: The desired reporting interval in microseconds
void request_feature(uint8_t feature_id, uint32_t report_interval_us) {
    // Payload for SET_FEATURE_REQUEST (0xFD)
    // Byte 0: Feature ID (1 byte)
    // Byte 1: Reserved (1 byte, usually 0)
    // Bytes 2-5: Report Interval (4 bytes, uint32_t)
    uint16_t payload_len = 1 + 1 + 4;

    // Prepare the tx_buffer for the SET_FEATURE command
    // The header will be added by shtp_send_packet.
    // We put the payload data starting after the header space.
    tx_buffer[sizeof(shtpHeader)] = feature_id;
    tx_buffer[sizeof(shtpHeader) + 1] = 0; // Reserved byte
    tx_buffer[sizeof(shtpHeader) + 2] = (uint8_t)(report_interval_us & 0xFF);
    tx_buffer[sizeof(shtpHeader) + 3] = (uint8_t)((report_interval_us >> 8) & 0xFF);
    tx_buffer[sizeof(shtpHeader) + 4] = (uint8_t)((report_interval_us >> 16) & 0xFF);
    tx_buffer[sizeof(shtpHeader) + 5] = (uint8_t)((report_interval_us >> 24) & 0xFF);

    printf("Requesting Feature ID 0x%02X with interval %u us...\n", feature_id, report_interval_us);
    // Send the SET_FEATURE_REQUEST command (0xFD) on the COMMAND channel (0x00)
    shtp_send_packet(payload_len, REGISTER_CHANNEL_REPORTS, 0); // Sequence number can be 0 for control
}


// === Parse and Print Accelerometer Data ===
void process_accelerometer_data(const uint8_t* packet_data, int packet_length) {
    shtpHeader data_header;
    memcpy(&data_header, packet_data, sizeof(shtpHeader));

    // The actual data starts after the SHTP header and the report ID.
    // The accelerometer data payload is typically 6 bytes (X, Y, Z as int16_t)
    // So, we need to ensure we have at least sizeof(shtpHeader) + 1 (report ID) + 6 (data) bytes.
    const int expected_payload_data_len = 1 + 6; // Report ID + X, Y, Z data

    if (packet_length >= sizeof(shtpHeader) + expected_payload_data_len) {
        // The report ID is at packet_data[sizeof(shtpHeader)]
        // The actual data starts at packet_data[sizeof(shtpHeader) + 1]

        int16_t acc_x = (int16_t)(packet_data[sizeof(shtpHeader) + 1] | (packet_data[sizeof(shtpHeader) + 2] << 8));
        int16_t acc_y = (int16_t)(packet_data[sizeof(shtpHeader) + 3] | (packet_data[sizeof(shtpHeader) + 4] << 8));
        int16_t acc_z = (int16_t)(packet_data[sizeof(shtpHeader) + 5] | (packet_data[sizeof(shtpHeader) + 6] << 8));

        // The scaling factor for accelerometer is typically 1/2048 g per LSB.
        // To convert to G: raw_value / 2048.0
        float acc_x_g = acc_x / 2048.0f;
        float acc_y_g = acc_y / 2048.0f;
        float acc_z_g = acc_z / 2048.0f;

        printf("Accel Data: X=%.3fg, Y=%.3fg, Z=%.3fg\n", acc_x_g, acc_y_g, acc_z_g);
    } else {
        printf("Received incomplete accelerometer data packet. Expected payload length >= %d, got %d.\n",
               expected_payload_data_len, packet_length - sizeof(shtpHeader));
    }
}


/// === Wait For and Process SHTP Report ===
// This function now just reads and prints any packet it finds on the expected channel
// and with the expected report ID, but doesn't strictly return true/false for confirmation.
// The main loop will handle the logic.
void check_for_specific_report(uint8_t expected_channel, uint8_t expected_report_id) {
    int bytes_read = shtp_read_packet(rx_buffer, MAX_PACKET_SIZE);
    if (bytes_read > 0) {
        shtpHeader data_header;
        memcpy(&data_header, rx_buffer, sizeof(shtpHeader));

        // Check if it's the correct channel and report ID
        if (data_header.channel == expected_channel) {
            if (bytes_read > sizeof(shtpHeader)) {
                uint8_t report_id = rx_buffer[sizeof(shtpHeader)];
                if (report_id == expected_report_id) {
                    printf("Found expected report (ID 0x%02X) on channel 0x%02X.\n", expected_report_id, expected_channel);
                    // You might want to parse this specific report here if needed
                } else {
                    printf("Received packet on channel 0x%02X, but expected report ID 0x%02X, got 0x%02X.\n",
                           expected_channel, expected_report_id, report_id);
                }
            } else {
                printf("Received packet on channel 0x%02X, but it has no payload to check report ID.\n", expected_channel);
            }
        }
    }
}

// === Main Script ===
int main() {
    stdio_init_all();

    getchar();  // Wait for user to press Enter
    printf("User input detected. Beginning main()\n");

    // SPI Configuration
    // Set the SPI baud rate to 32.768 kHz (32,768)
    spi_init(SPI_PORT, 32.768 * 1000);
    gpio_set_function(PIN_MISO, GPIO_FUNC_SPI);
    gpio_set_function(PIN_SCK, GPIO_FUNC_SPI);
    gpio_set_function(PIN_MOSI, GPIO_FUNC_SPI);

    gpio_init(PIN_CS);
    gpio_set_dir(PIN_CS, GPIO_OUT);
    gpio_put(PIN_CS, 1);

    gpio_init(PIN_INT);
    gpio_set_dir(PIN_INT, GPIO_IN);
    gpio_pull_up(PIN_INT);

    gpio_init(PIN_RST);
    gpio_set_dir(PIN_RST, GPIO_OUT);
    gpio_put(PIN_RST, 1);
    printf("BNO085 SPI communication initialization complete.\n");
  
    bno085_reset();
    printf("BNO085 reset complete.\n");

    // Wait for the INT pin to go low, indicating the device is ready.
    // Add a small timeout here to prevent infinite loop if device doesn't respond.
    uint32_t reset_wait_start = get_absolute_time();
    while (gpio_get(PIN_INT) == 1) {
        if (absolute_time_diff_us(get_absolute_time(), reset_wait_start) > 5000000) { // 5 seconds timeout
            printf("Timeout waiting for BNO085 interrupt after reset.\n");
            return 1;
        }
        sleep_us(100);
    }
    printf("BNO085 interrupt (INT) asserted (active low). Ready for communication!\n");

    // --- Read Initial Advertisement Packet ---
    memset(rx_buffer, 0, MAX_PACKET_SIZE);
    printf("Attempting to read initial SHTP advertisement packet...\n");
    // Read the advertisement packet. This packet contains SHTP version, channel info, etc.
    // The advertisement packet is sent unsolicited by the BNO08X.
    int adv_bytes_read = shtp_read_packet(rx_buffer, MAX_PACKET_SIZE);
        // Outputs: "Raw Header Bytes: "

    if (adv_bytes_read > 0) {
        printf("Received %d bytes (Advertisement Packet):\n", adv_bytes_read);
        // Print the advertisement data bytes to inspect them
        for (int i = 0; i < adv_bytes_read; i++) {
            printf("0x%02X ", rx_buffer[i]);
            if ((i + 1) % 16 == 0) printf("\n");
        }
        printf("\n");
        printf("Raw advertisement data captured.\n");
    } else {
        printf("Failed to read advertisement packet. Check wiring or reset.\n");
        // It's possible the advertisement packet is missed if not read quickly.
        // For now, we'll proceed. If no data comes later, this is a point to debug.
        return 1;
    }

    // Declare a variable to hold the feature report ID
    // Since the defined values are integers (hexadecimal literals are just a way to represent integers),
    // an 'int' or 'unsigned int' is appropriate. 'unsigned int' is often preferred for IDs.
    unsigned int current_feature_id;

    // --- Request Sensor Data ---
    // Assign a select feature report ID
    current_feature_id = SHTP_FEATURE_REPORT_ID_ACCELEROMETER;
    uint32_t report_interval_us = 60000; // Request data every 60ms (100Hz)
    request_feature(current_feature_id, report_interval_us);

    // --- Wait for a short period to allow the BNO085 to respond ---
    // Give the BNO085 some time to process feature request before checking main loop.
    sleep_ms(200); // Give it 200ms to respond

    printf("\nStarting main loop to read sensor data...\n");
    while (1) {
        // Check if the BNO085 has new data ready (INT pin is low)
        if (gpio_get(PIN_INT) == 0) {
            // Read the packet from the BNO085.
            int bytes_read = shtp_read_packet(rx_buffer, MAX_PACKET_SIZE);
            if (bytes_read > 0) {
                shtpHeader data_header;
                memcpy(&data_header, rx_buffer, sizeof(shtpHeader));

                // Check if it's a data packet on the NORMAL channel (0x03)
                if (data_header.channel == REGISTER_CHANNEL_REPORTS) {
                    // Now process the payload based on its Feature Report ID
                    // The first byte of the payload is the report ID.
                    if (bytes_read > sizeof(shtpHeader)) { // Ensure there's a payload to read report ID from
                        uint8_t report_id = rx_buffer[sizeof(shtpHeader)];

                        if (report_id == current_feature_id) {
                            process_accelerometer_data(rx_buffer, bytes_read);
                        } else {
                            // Handle other sensor types if needed
                            printf("(A) Received 0x%02X channel packet, for Report ID: 0x%02X\n", data_header.channel, report_id);
                        }
                    } else {
                        printf("Received NORMAL channel packet with no payload.\n");
                    }
                } else if (data_header.channel != REGISTER_CHANNEL_REPORTS) {
                    // This is likely a response to our SET_FEATURE_REQUEST or other commands.
                    // Let's check for the expected GET_FEATURE_RESPONSE (0xFC)
                    if (bytes_read > sizeof(shtpHeader)) {
                        uint8_t report_id = rx_buffer[sizeof(shtpHeader)];
                        if (report_id != current_feature_id) {
                            printf("(B) Received 0x%02X channel packet, for Report ID: 0x%02X\n", data_header.channel, report_id);
                            for (int i = 0; i < bytes_read; i++) {
                                printf("0x%02X ", rx_buffer[i]);
                                if ((i + 1) % 16 == 0) printf("\n");
                            };
                        } else {
                            printf("(C) Received 0x%02X channel packet, for Report ID: 0x%02X\n", data_header.channel, report_id);
                        }
                    } else {
                        printf("Received COMMAND channel packet with no payload.\n");
                    }
                } else {
                    printf("Unexpected error. Channel %02X, PayloadLen=%d\n", data_header.channel, bytes_read);
                }
            }
        }
        // A small delay to avoid overwhelming the CPU and give the SPI peripheral time.
        sleep_us(100);
    }

    return 0; // Should not be reached in the while(1) loop
}