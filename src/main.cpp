#include <Arduino.h>
#include <Wire.h>
#include <Adafruit_MPU6050.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>
#include <Smoothed.h>
#include "crsf.h"

// Pins
#define PIN_SDA              8
#define PIN_SCL              9
#define PIN_VBAT             0
#define PIN_SW_A             1
#define PIN_SW_B             4
#define JOYSTICK_HORI        2
#define JOYSTICK_VERT        3
#define JOYSTICK_BTN         7
#define PIN_CRSF_HALF_DUPLEX 6
#define PIN_CRSF_DUMMY       5 // Need a dummy pin for the TX/RX not being used

// Other defines
#define VOLTAGE_SCALER ((float) 0.001411965811965812)
#define I2C_ADDR_DISPLAY 0x3C
#define I2C_ADDR_IMU 0x68
#define LOOP_PERIOD_MS 100
#define SerialCRSF Serial1
#define CRSF_SERIAL_TX_BUFFER_SIZE 128
#define CRSF_SERIAL_RX_BUFFER_SIZE 256

#define CHANNEL_VALUE_MIN 988
#define CHANNEL_VALUE_MAX 2012
#define CHANNEL_VALUE_MID 1500

#define CHANNEL_JOSTICK_HORI 2
#define CHANNEL_JOSTICK_VERT 3
#define CHANNEL_SW_A         4
#define CHANNEL_SW_B         5

// Instantiations
static Adafruit_SSD1306 display(128, 64, &Wire, -1);
static Adafruit_MPU6050 imu;

// Variable
static sensors_event_t acc;
static sensors_event_t gyro;
static sensors_event_t temp;
static int vbat_raw;
static float vbat;
static Smoothed <float> vbat_avg;

// Inputs
static int sw_a;
static int sw_b;
static int joystick_hori;
static int joystick_vert;
static int joystick_btn;

// CRSF
static CRSF crsf;
static uint8_t crsf_tx_buf[CRSF_MAX_PACKET_SIZE];
static CRSF_PacketChannelData channel_data;
static CRSF_PacketLinkStatistics link_statistics;
static CRSF_PacketDeviceInfo device_info;
static bool received_device_info = false;
static int crsf_read = 0;
static int crsf_written = 0;
static int crsf_rx_pkts = 0;
static int crsf_tx_pkts = 0;

// Helper functions
static uint16_t switch_value_to_channel(const int raw_value);
static void draw_display();
static void print_info();
static void print_buf(const uint8_t* buf, int size);
static void print_device_info();


void setup()
{
    Serial.begin(921600);
    SerialCRSF.begin(400000, SERIAL_8N1, PIN_CRSF_DUMMY, PIN_CRSF_HALF_DUPLEX, true);
    SerialCRSF.setTxBufferSize(CRSF_SERIAL_TX_BUFFER_SIZE);
    SerialCRSF.setRxBufferSize(CRSF_SERIAL_RX_BUFFER_SIZE);
    Wire.setPins(PIN_SDA, PIN_SCL);
    Wire.setClock(400000);
    Wire.begin();

    pinMode(PIN_VBAT, INPUT);
    analogReadResolution(12);

    pinMode(PIN_SW_A, INPUT);
    pinMode(PIN_SW_B, INPUT);
    pinMode(JOYSTICK_HORI, INPUT);
    pinMode(JOYSTICK_VERT, INPUT);
    pinMode(JOYSTICK_BTN, INPUT_PULLUP);

    if (!display.begin(SSD1306_SWITCHCAPVCC, I2C_ADDR_DISPLAY))
    {
        while (1)
        {
            Serial.println("Failed to initialize display...");
            delay(1000);
        }
    }

    display.clearDisplay();
    display.setTextColor(SSD1306_WHITE);
    display.setTextSize(1);
    display.display();

    if (!imu.begin(I2C_ADDR_IMU, &Wire))
    {
        display.print("IMU init fail");
        while (1)
        {
            Serial.println("Failed to initialize display...");
            delay(1000);
        }
    }

    vbat_avg.begin(SMOOTHED_AVERAGE, 20);

    imu.setAccelerometerRange(MPU6050_RANGE_4_G);
    imu.setGyroRange(MPU6050_RANGE_500_DEG);
    imu.setFilterBandwidth(MPU6050_BAND_94_HZ);

    // Set default values for channel data
    for (int i = 0; i < 16; i++)
    {
        channel_data.channels[i] = 1500;
    }
    channel_data.channels[4] = 1000; // Arm throttle
}

static uint32_t last_ping = 0;

void loop()
{
    static uint32_t last_loop = 0;

    if ((millis() - last_loop) < LOOP_PERIOD_MS)
    {
        return;
    }

    // Read IMU
    imu.getEvent(&acc, &gyro, &temp);

    // Read battery voltage
    vbat_raw = analogRead(PIN_VBAT);
    vbat_avg.add((float) vbat_raw * VOLTAGE_SCALER);
    vbat = vbat_avg.get();

    // Read input channels
    sw_a = analogRead(PIN_SW_A);
    sw_b = analogRead(PIN_SW_B);
    joystick_hori = analogRead(JOYSTICK_HORI);
    joystick_vert = analogRead(JOYSTICK_VERT);
    joystick_btn = digitalRead(JOYSTICK_BTN);

    // Set I/O to channels
    channel_data.channels[CHANNEL_JOSTICK_HORI] = map(joystick_hori, 0, 4095, CHANNEL_VALUE_MIN, CHANNEL_VALUE_MAX);
    channel_data.channels[CHANNEL_JOSTICK_VERT] = map(joystick_vert, 0, 4095, CHANNEL_VALUE_MIN, CHANNEL_VALUE_MAX);
    channel_data.channels[CHANNEL_SW_A] = switch_value_to_channel(sw_a);
    channel_data.channels[CHANNEL_SW_B] = switch_value_to_channel(sw_b);

    // Read CRSF data
    crsf_frame_type_t frame_type;

    while (SerialCRSF.available())
    {
        uint8_t byte = SerialCRSF.read();
        crsf_read++;
        if (crsf.parse_byte(byte, &frame_type))
        {
            Serial.printf("! CRSF: %02x\n", frame_type);
            crsf_rx_pkts++;
            switch (frame_type)
            {
                case CRSF_FRAMETYPE_LINK_STATISTICS:
                    link_statistics = crsf.get_link_statistics();
                    break;
                case CRSF_FRAMETYPE_DEVICE_INFO:
                    received_device_info = true;
                    device_info = crsf.get_device_info();
                    print_device_info();
                    break;
                case CRSF_FRAMETYPE_RADIO_ID:
                    break;
                case CRSF_FRAMETYPE_FLIGHT_MODE:
                    break;
                case CRSF_FRAMETYPE_BATTERY_SENSOR:
                    break;
                case CRSF_FRAMETYPE_GPS:
                    break;
            }
        }
    }

    // Decide what CRSF packet to send
    int crsf_pkt_size = 0;
    if (!received_device_info)
    {
        Serial.println("Requesting device info");
        crsf_pkt_size = crsf.pack_device_ping(crsf_tx_buf);
    }
    else
    {
        crsf_pkt_size = crsf.pack_channel_data(channel_data, crsf_tx_buf);
    }

    // First we'll send a CRSF packet
    SerialCRSF.setPins(PIN_CRSF_DUMMY, PIN_CRSF_HALF_DUPLEX);
    crsf_written += SerialCRSF.write(crsf_tx_buf, crsf_pkt_size);
    crsf_tx_pkts++;
    SerialCRSF.flush(); // Flush to ensure all data in TX buffers is sent

    // After we've sent the packet, we'll swap to RX and start reading input data from transmitter
    SerialCRSF.setPins(PIN_CRSF_HALF_DUPLEX, PIN_CRSF_DUMMY);

    // Draw the display
    draw_display();

    // Print debug info
    static uint32_t last_print = 0;
    if ((millis() - last_print) > 1000)
    {
        print_info();
        print_device_info();
        last_print = millis();
    }

    last_loop = millis();
}

static void print_buf(const uint8_t* buf, int size)
{
    for (int i = 0; i < size; i++)
    {
        Serial.printf("%02x ", buf[i]);
    }
    Serial.print("\n");
}

static void print_device_info()
{
    Serial.printf("Discovered CRSF Device:");
    Serial.printf("Name: %s\n", device_info.display_name);
    Serial.printf("HW version: %d.%d.%d\n", device_info.hardware_version[1], device_info.hardware_version[2], device_info.hardware_version[3]);
    Serial.printf("SW version: %d.%d.%d\n", device_info.software_version[1], device_info.software_version[2], device_info.software_version[3]);
    Serial.printf("Available params: %d\n", device_info.nbr_of_config_params);
    Serial.printf("Protocol version: %d\n", device_info.protocol_version);
}

static uint16_t switch_value_to_channel(const int raw_value)
{
    if (raw_value < 2000)
    {
        return CHANNEL_VALUE_MID;
    }
    else if (raw_value > 2500)
    {
        return CHANNEL_VALUE_MAX;
    }
    else
    {
        return CHANNEL_VALUE_MIN;
    }
}

static void draw_display()
{
    // Drawing the display takes long.. Causing us to get loop period of like 60ms ... (:
    display.clearDisplay();
    display.setCursor(0, 0);
    display.printf("%.1f %.1f %.1f\n", acc.acceleration.x, acc.acceleration.y, acc.acceleration.z);
    display.printf("%.1f %.1f %.1f\n", gyro.gyro.x, gyro.gyro.y, gyro.gyro.z);
    display.printf("BAT: %.2f V\n", vbat);
    display.display();
}

static void print_info()
{
    Serial.printf("[ACC] X: %.2f, Y: %.2f, Y: %.2f\n", acc.acceleration.x, acc.acceleration.y, acc.acceleration.z);
    Serial.printf("[GYRO] X: %.2f, Y: %.2f, Y: %.2f\n", gyro.gyro.x, gyro.gyro.y, gyro.gyro.z);
    Serial.printf("[BATT] raw: %d, vbat: %.2f\n", vbat_raw, vbat);
    Serial.printf("[INPUT] SW A: %d, SW B: %d, JH: %d, JV: %d, JB: %d\n", sw_a, sw_b, joystick_hori, joystick_vert, joystick_btn);
    Serial.print("[CHANNELS] ");
    for (int i = 0; i < 16; i++) Serial.printf("%u ", channel_data.channels[i]);
    Serial.print("\n");
    Serial.printf("[CRSF] Read: %d, RX: %d, TX: %d, written: %d\n", crsf_read, crsf_rx_pkts, crsf_tx_pkts, crsf_written);
    Serial.printf("[ELRS] RSSI: %d, LQ: %d\n", link_statistics.uplink_rssi_ant1, link_statistics.downlink_link_quality);
    Serial.print("\n");
}