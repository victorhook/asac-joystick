#!/usr/bin/env python3

from threading import Thread
import time
from serial import Serial
import sys
from queue import Queue


# e8 48 4 80 80 84 80 80 e7
#
# c8 8 4 80 80 84 80 80 e7

from enum import IntEnum

class CRSF_Dest(IntEnum):
    CRSF_ADDRESS_CRSF_TRANSMITTER  = 0xEE # Going to the transmitter module,
    CRSF_ADDRESS_RADIO_TRANSMITTER = 0xEA # Going to the handset,
    CRSF_ADDRESS_FLIGHT_CONTROLLER = 0xC8 # Going to the flight controller,
    CRSF_ADDRESS_CRSF_RECEIVER     = 0xEC # Going to the receiver (from FC),


class CRSF:
    ''' Crossfire parse machine '''

    # | dest | len | frame_type | payload | crc |

    def __init__(self) -> None:
        self._pkt = []
        self._byte = 0

    def parse_byte(self, byte: bytes) -> bool:
        data = ord(byte)
        if self._byte == 0:
            if data == CRSF_Dest.CRSF_ADDRESS_FLIGHT_CONTROLLER:
                print(' '.join(hex(b).zfill(2)[2:] for b in self._pkt))
                self._pkt = [data]
            else:
                self._pkt.append(data)


def reader(s: Serial, crsf: CRSF) -> None:
    while True:
        data = s.read(1)
        #crsf.parse_byte(data)
        #continue
        sys.stdout.write(f'{hex(ord(data))[2:].zfill(2)} ')
        #if data == b'\xee':
        #    sys.stdout.write('\n')
        sys.stdout.flush()

def writer(s: Serial, tx: Queue) -> None:
    while True:
        data = tx.get()
        s.write(data)
        s.flush()

# [sync] [len] [type] [payload] [crc8]
#0xee 0x92 0x75 0x1d 0x1f 0xfa 0xc2 0x7 0xc5 0xfe 0x23 0xcf 0xd3 0x41 0xf2 0x41 0xa9 0x7f 0xa2 0xd6 0xda 0x22 0x76 0xaa 0xa5 0x5e 0x75 0xee 


if __name__ == '__main__':
    port = sys.argv[1]
    if len(sys.argv) == 3:
        baud = int(sys.argv[2])
    else:
        baud = 400000
    print(f'Port: {port}, baud: {baud}')
    s = Serial(port, baudrate=baud)
    tx = Queue()

    crsf = CRSF()

    Thread(target=reader, daemon=True, args=(s, crsf)).start()
    Thread(target=writer, daemon=True, args=(s, tx)).start()

    while True:
        time.sleep(1)