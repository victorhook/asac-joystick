#ifndef CRSF_H
#define CRSF_H

#include "stdint.h"
#include "stdbool.h"

/*
    CRFS Protocol details heavily inspired from this post: https://github.com/ExpressLRS/ExpressLRS/wiki/CRSF-Protocol

    CRFS In short:
        Serial protocol, that uses baud rate 420000, 8 data bits, 1 stop, no parity.
        Data is separated into *frames* with the following structure:
        | sync | len | frame_type | payload | crc |
        CRC is an 8-bit crc with poly 0xD5, including frame_type and payload.
*/

#define CRSF_MAX_PACKET_SIZE  64
#define CRSF_MAX_PAYLOAD_SIZE 60
#define CRSF_HEADER_SIZE 4


// -- CRSF Packet types and sync bytes -- //

typedef enum
{
    CRSF_FRAMETYPE_GPS                       = 0x02,
    CRSF_FRAMETYPE_BATTERY_SENSOR            = 0x08,
    CRSF_FRAMETYPE_HEARTBEAT                 = 0x0B,
    CRSF_FRAMETYPE_LINK_STATISTICS           = 0x14,
    CRSF_FRAMETYPE_OPENTX_SYNC               = 0x10,
    CRSF_FRAMETYPE_RADIO_ID                  = 0x3A,
    CRSF_FRAMETYPE_RC_CHANNELS_PACKED        = 0x16,
    CRSF_FRAMETYPE_SUBSET_RC_CHANNELS_PACKED = 0x17,
    CRSF_FRAMETYPE_LINK_STATISTICS_RX        = 0x1C,
    CRSF_FRAMETYPE_LINK_STATISTICS_TX        = 0x1D,
    CRSF_FRAMETYPE_ATTITUDE                  = 0x1E,
    CRSF_FRAMETYPE_FLIGHT_MODE               = 0x21,
    // Extended Header Frames, range: 0x28 to 0x96
    CRSF_FRAMETYPE_DEVICE_PING              = 0x28,
    CRSF_FRAMETYPE_DEVICE_INFO              = 0x29,
    CRSF_FRAMETYPE_PARAMETER_SETTINGS_ENTRY = 0x2B,
    CRSF_FRAMETYPE_PARAMETER_READ           = 0x2C,
    CRSF_FRAMETYPE_PARAMETER_WRITE          = 0x2D,
    CRSF_FRAMETYPE_COMMAND                  = 0x32,
    // MSP commands
    CRSF_FRAMETYPE_MSP_REQ                  = 0x7A,  // response request using msp sequence as command
    CRSF_FRAMETYPE_MSP_RESP                 = 0x7B,  // reply with 58 byte chunked binary
    CRSF_FRAMETYPE_MSP_WRITE                = 0x7C,  // write with 8 byte chunked binary (OpenTX outbound telemetry buffer limit)
    CRSF_FRAMETYPE_DISPLAYPORT_CMD          = 0x7D  // displayport control command
} crsf_frame_type_t;

typedef enum
{
    CRSF_ADDRESS_CRSF_TRANSMITTER  = 0xEE, // Going to the transmitter module,
    CRSF_ADDRESS_RADIO_TRANSMITTER = 0xEA, // Going to the handset,
    CRSF_ADDRESS_FLIGHT_CONTROLLER = 0xC8, // Going to the flight controller,
    CRSF_ADDRESS_CRSF_RECEIVER     = 0xEC, // Going to the receiver (from FC),
} crsf_sync_t;


// -- CRSF Packets -- //

typedef struct
{
    uint16_t channels[16];
} CRSF_PacketChannelData;

typedef struct
{
    // Uplink is the connection from the ground to the UAV and downlink the opposite direction.
    uint8_t uplink_rssi_ant1;          // Uplink RSSI Ant. 1 ( dBm * -1 )
    uint8_t uplink_rssi_atn2;          // Uplink RSSI Ant. 2 ( dBm * -1 )
    uint8_t uplink_link_quality;       // Uplink Package success rate / Link quality ( % )
    int8_t  uplink_snr;                // Uplink SNR ( dB, or dB*4 for TBS I believe )
    uint8_t diversity_active_antenna;  // Diversity active antenna ( enum ant. 1 = 0, ant. 2 = 1 )
    uint8_t rf_mode;                   // RF Mode ( 500Hz, 250Hz etc, varies based on ELRS Band or TBS )
    uint8_t uplink_tx_power;           // Uplink TX Power ( enum 0mW = 0, 10mW, 25 mW, 100 mW, 500 mW, 1000 mW, 2000mW, 50mW )
    uint8_t downlink_rssi;             // Downlink RSSI ( dBm * -1 )
    uint8_t downlink_link_quality;     // Downlink package success rate / Link quality ( % )
    int8_t  downlink_snr;              // Downlink SNR ( dB )
}__attribute__((packed)) CRSF_PacketLinkStatistics;

typedef struct
{
    char display_name[16];
    char serial_number[4];
    uint8_t hardware_version[4];
    uint8_t software_version[4];
    uint8_t nbr_of_config_params;
    uint8_t protocol_version;
} CRSF_PacketDeviceInfo;


typedef union
{
    CRSF_PacketChannelData channel_data;
    CRSF_PacketLinkStatistics statistics;
    CRSF_PacketDeviceInfo device_info;
} CRSF_Packet;


/*
    Reference: https://github.com/crsf-wg/crsf/wiki
*/
class CRSF
{
    public:
        CRSF();
        ~CRSF();
        /**
         * @brief Parses a single byte in the CRSF state machine.
         *
         * @param byte Byte to parse
         * @param frame_type If a new packet has been succesfully parsed, this
         *      will be filled with the type of the new packet.
         * @return true if a new packet has been found.
         */
        bool parse_byte(const uint8_t byte, crsf_frame_type_t* frame_type);

        int pack_channel_data(const CRSF_PacketChannelData& channel_data, uint8_t* buf);

        int pack_device_ping(uint8_t* buf);

        CRSF_PacketChannelData get_channel_data();
        CRSF_PacketLinkStatistics get_link_statistics();
        CRSF_PacketDeviceInfo get_device_info();

    private:
        // -- Typedefs -- //
        typedef struct
        {
            uint8_t sync;
            uint8_t len;        // Length of bytes that follow, including type, payload, and CRC (PayloadLength+2). Overall packet length is PayloadLength+4 (dest, len, type, crc), or LEN+2 (dest, len)
            uint8_t frame_type;
            uint8_t payload[CRSF_MAX_PAYLOAD_SIZE];
            uint8_t crc;
        }__attribute__((packed)) crsf_packet_raw_t;

        typedef enum
        {
            PARSE_STATE_SYNC      = 0,
            PARSE_STATE_LEN       = 1,
            PARSE_STATE_TYPE      = 2,
            PARSE_STATE_PAYLOAD   = 3,
            PARSE_STATE_CRC       = 4
        } parse_state_t;

        typedef struct
        {
            unsigned ch0  : 11;
            unsigned ch1  : 11;
            unsigned ch2  : 11;
            unsigned ch3  : 11;
            unsigned ch4  : 11;
            unsigned ch5  : 11;
            unsigned ch6  : 11;
            unsigned ch7  : 11;
            unsigned ch8  : 11;
            unsigned ch9  : 11;
            unsigned ch10 : 11;
            unsigned ch11 : 11;
            unsigned ch12 : 11;
            unsigned ch13 : 11;
            unsigned ch14 : 11;
            unsigned ch15 : 11;
        }__attribute__((packed)) crsf_channels_t;

        // -- Attributes -- //
        uint8_t           _buf[CRSF_MAX_PACKET_SIZE];
        crsf_packet_raw_t _curr_pkt;
        CRSF_Packet       _last_pkt;
        uint8_t           _frame_bytes_received;
        uint8_t           _payload_bytes_to_receive;
        uint8_t           _payload_bytes_received;
        uint32_t          _parse_errors;
        parse_state_t     _state;

        // -- Methods -- //
        crsf_frame_type_t handle_new_packet();
        bool valid_sync(const uint8_t byte);
        bool valid_length(const uint8_t byte);
        bool valid_frame_type(const uint8_t byte);
        bool valid_crc(const uint8_t byte);
        void reset_rx_state_machine();
        uint8_t calculate_crc(const uint8_t* data, const uint8_t len);
};

#endif /* CRSF_H */
