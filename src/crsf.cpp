#include "crsf.h"

#include <string.h>
#include <Arduino.h>


// Helpful for debugging
static void print_buf(const uint8_t* buf, int size)
{
    for (int i = 0; i < size; i++)
    {
        Serial.printf("%02x ", buf[i]);
    }
    Serial.print("\n");
}


CRSF::CRSF()
{

}

int CRSF::pack_device_ping(uint8_t* buf)
{
    // HOST: C8 04 28 00 EA 54
    int payload_size = 2;
    buf[0] = CRSF_ADDRESS_CRSF_TRANSMITTER;
    buf[1] = 4;
    buf[2] = CRSF_FRAMETYPE_DEVICE_PING;
    buf[3] = 0x00;
    buf[4] = 0xEA;
    buf[5] = calculate_crc(&buf[2], payload_size + 1);
    return CRSF_HEADER_SIZE + payload_size;
}

int CRSF::pack_channel_data(const CRSF_PacketChannelData& channel_data, uint8_t* buf)
{
    crsf_channels_t channels;
    channels.ch0 = map(channel_data.channels[0], 988, 2012, 172, 1811);
    channels.ch1 = map(channel_data.channels[1], 988, 2012, 172, 1811);
    channels.ch2 = map(channel_data.channels[2], 988, 2012, 172, 1811);
    channels.ch3 = map(channel_data.channels[3], 988, 2012, 172, 1811);
    channels.ch4 = map(channel_data.channels[4], 988, 2012, 172, 1811);
    channels.ch5 = map(channel_data.channels[5], 988, 2012, 172, 1811);
    channels.ch6 = map(channel_data.channels[6], 988, 2012, 172, 1811);
    channels.ch7 = map(channel_data.channels[7], 988, 2012, 172, 1811);
    channels.ch8 = map(channel_data.channels[8], 988, 2012, 172, 1811);
    channels.ch9 = map(channel_data.channels[9], 988, 2012, 172, 1811);
    channels.ch10 = map(channel_data.channels[10], 988, 2012, 172, 1811);
    channels.ch11 = map(channel_data.channels[11], 988, 2012, 172, 1811);
    channels.ch12 = map(channel_data.channels[12], 988, 2012, 172, 1811);
    channels.ch13 = map(channel_data.channels[13], 988, 2012, 172, 1811);
    channels.ch14 = map(channel_data.channels[14], 988, 2012, 172, 1811);
    channels.ch15 = map(channel_data.channels[15], 988, 2012, 172, 1811);

    uint8_t payload_size = sizeof(crsf_channels_t);

    buf[0] = CRSF_ADDRESS_CRSF_TRANSMITTER;              // Sync
    buf[1] = 2 + payload_size;                // Length: Payload + Type and CRC
    buf[2] = CRSF_FRAMETYPE_RC_CHANNELS_PACKED;          // Frametype
    memcpy(&buf[3], &channels, payload_size); // Payload

    uint8_t crc = calculate_crc(&buf[2], payload_size+1);
    buf[3 + payload_size] = crc; // CRC

    return CRSF_HEADER_SIZE + payload_size;
}

bool CRSF::parse_byte(const uint8_t byte, crsf_frame_type_t* frame_type)
{
    //printf("State: %d, Len: %d, Payload bytes: %d, %02x\n",
    //        _state, _curr_pkt.len, _payload_bytes_received, byte);
    bool new_packet = false;
    _buf[_frame_bytes_received] = byte;
    _frame_bytes_received++;

    if (_frame_bytes_received > CRSF_MAX_PACKET_SIZE)
    {
        // Somehow parsed way too many bytes, let's reset
        reset_rx_state_machine();
    }

    switch (_state)
    {
        case PARSE_STATE_SYNC:
            if (valid_sync(byte))
            {
                _state = PARSE_STATE_LEN;
                _curr_pkt.sync = byte;
            }
            else
            {
                reset_rx_state_machine();
                _parse_errors++;
            }
            break;
        case PARSE_STATE_LEN:
            if (valid_length(byte))
            {
                // Length of the CRSF packet includes {Type, Payload, CRC}
                _curr_pkt.len = byte;
                _payload_bytes_to_receive = _curr_pkt.len - 2;
                _state = PARSE_STATE_TYPE;
            }
            else
            {
                _parse_errors++;
                reset_rx_state_machine();
            }
            break;
        case PARSE_STATE_TYPE:
            if (valid_frame_type(byte))
            {
                _curr_pkt.frame_type = byte;
                _state = PARSE_STATE_PAYLOAD;
            }
            else
            {
                reset_rx_state_machine();
                _parse_errors++;
            }
            break;
        case PARSE_STATE_PAYLOAD:
            _curr_pkt.payload[_payload_bytes_received] = byte;
            _payload_bytes_received++;
            if (_payload_bytes_received >= _payload_bytes_to_receive)
            {
                _state = PARSE_STATE_CRC;
            }
            break;
        case PARSE_STATE_CRC:
            _curr_pkt.crc = byte;
            if (valid_crc(byte))
            {
                //Serial.printf("CRC: %02x, ", byte);
                *frame_type = handle_new_packet();
                new_packet = true;
            }
            else
            {
                _parse_errors++;
            }
            reset_rx_state_machine();
            break;
        default:
            _parse_errors++;
            reset_rx_state_machine();
            break;
    }

    return new_packet;
}

// -- Private helpers -- //

crsf_frame_type_t CRSF::handle_new_packet()
{
    crsf_frame_type_t frame_type = (crsf_frame_type_t) _curr_pkt.frame_type;


    crsf_channels_t* channels;
    CRSF_PacketLinkStatistics* statistics;
    CRSF_PacketDeviceInfo* device_info;
    char* device_name;
    int name_offset;

    switch (frame_type)
    {
        case CRSF_FRAMETYPE_RC_CHANNELS_PACKED:
            _last_pkt.channel_data = CRSF_PacketChannelData{};

            // Fill packet with channel data
            channels = (crsf_channels_t*) _curr_pkt.payload;
            _last_pkt.channel_data.channels[0]  = channels->ch0;
            _last_pkt.channel_data.channels[1]  = channels->ch1;
            _last_pkt.channel_data.channels[2]  = channels->ch2;
            _last_pkt.channel_data.channels[3]  = channels->ch3;
            _last_pkt.channel_data.channels[4]  = channels->ch4;
            _last_pkt.channel_data.channels[5]  = channels->ch5;
            _last_pkt.channel_data.channels[6]  = channels->ch6;
            _last_pkt.channel_data.channels[7]  = channels->ch7;
            _last_pkt.channel_data.channels[8]  = channels->ch8;
            _last_pkt.channel_data.channels[9]  = channels->ch9;
            _last_pkt.channel_data.channels[10] = channels->ch10;
            _last_pkt.channel_data.channels[11] = channels->ch11;
            _last_pkt.channel_data.channels[12] = channels->ch12;
            _last_pkt.channel_data.channels[13] = channels->ch13;
            _last_pkt.channel_data.channels[14] = channels->ch14;
            _last_pkt.channel_data.channels[15] = channels->ch15;
            break;
        case CRSF_FRAMETYPE_LINK_STATISTICS:
            statistics = (CRSF_PacketLinkStatistics*) _curr_pkt.payload;
            memcpy(&_last_pkt.statistics, statistics, sizeof(CRSF_PacketLinkStatistics));
            // Fill rx state with link statistics
            _last_pkt.statistics.uplink_rssi_ant1 = statistics->uplink_rssi_ant1;
            _last_pkt.statistics.uplink_link_quality = statistics->uplink_link_quality;
            break;
        case CRSF_FRAMETYPE_DEVICE_INFO:
            _last_pkt.device_info = CRSF_PacketDeviceInfo{};

            // First two bytes of payload is [EA, EE] = extended packet dest/src (handset/txmodule)
            device_name = (char*) &_curr_pkt.payload[2];

            // First field is display name (string) null-terminated. Not fixed length... So we have to check its length
            name_offset = strnlen(device_name, 16);
            strncpy(_last_pkt.device_info.display_name, device_name, name_offset);

            // Rest of fields thus requires some byte offsets to be parsed correctly :)
            memcpy(_last_pkt.device_info.serial_number,    &_curr_pkt.payload[2+name_offset + 1], 4);
            memcpy(_last_pkt.device_info.hardware_version, &_curr_pkt.payload[2+name_offset + 1 + 4], 4);
            memcpy(_last_pkt.device_info.software_version, &_curr_pkt.payload[2+name_offset + 1 + 8], 4);
            _last_pkt.device_info.nbr_of_config_params    = _curr_pkt.payload[2+name_offset + 1 + 12];
            _last_pkt.device_info.protocol_version        = _curr_pkt.payload[2+name_offset + 1 + 13];
            break;
        default:
            // TOOD
            //printf("UNKNOWN CRSF FRAME TYPE 0x%02x\n", packet->frame_type);
            _parse_errors++;
            break;
    }

    return frame_type;
}

CRSF_PacketDeviceInfo CRSF::get_device_info()
{
    return _last_pkt.device_info;
}

CRSF_PacketChannelData CRSF::get_channel_data()
{
    return _last_pkt.channel_data;
}

CRSF_PacketLinkStatistics CRSF::get_link_statistics()
{
    return _last_pkt.statistics;
}

bool CRSF::valid_sync(const uint8_t byte)
{
    return (
        (byte == CRSF_ADDRESS_CRSF_TRANSMITTER) ||
        (byte == CRSF_ADDRESS_RADIO_TRANSMITTER) ||
        (byte == CRSF_ADDRESS_FLIGHT_CONTROLLER) ||
        (byte == CRSF_ADDRESS_CRSF_RECEIVER)
    );
}
bool CRSF::valid_length(const uint8_t byte)
{
    // Pretty sure max payload is 64 bytes
    return byte < CRSF_MAX_PAYLOAD_SIZE;
}
bool CRSF::valid_frame_type(const uint8_t byte)
{
    // Highest known frame type (I think?)
    // This is not 100% reliable, but if we get a frame that is unknown
    // we will discard it anyways.
    return byte < 0x7D;
}
bool CRSF::valid_crc(const uint8_t byte)
{
    uint8_t crc = calculate_crc(&_curr_pkt.frame_type, _curr_pkt.len-1);
    return crc == byte;
}
void CRSF::reset_rx_state_machine()
{
    _state = PARSE_STATE_SYNC;
    _payload_bytes_received = 0;
    _payload_bytes_to_receive = 0;
    _frame_bytes_received = 0;
}
uint8_t CRSF::calculate_crc(const uint8_t* data, const uint8_t len)
{
    // CRSF uses 8-bit CRC with poly 0xD5.
    // Not 100% how this really works but meh :)
    uint8_t poly = 0xD5;
    uint8_t crc = 0;
    for (int i = 0; i < len; i++)
    {
        //printf("CRC: 0x%02x\n", data[i]);
        crc ^= data[i];

        for (int i = 0; i < 8; i++)
        {
            if (crc & 0x80)
            {
                crc = (crc << 1) ^ poly;
            }
            else
            {
                crc = crc << 1;
            }
        }

    }
    return crc;
}

