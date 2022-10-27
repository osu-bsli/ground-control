import serial_data_controller
import serial
import struct
import packet_util
import crc
import enum

PACKET_TYPE_ALTITUDE =      1
PACKET_TYPE_COORDINATES =   2
PACKET_TYPE_C =             4
PACKET_TYPE_D =             8

PACKET_TYPES = [
    PACKET_TYPE_ALTITUDE,
    PACKET_TYPE_COORDINATES,
    PACKET_TYPE_C,
    PACKET_TYPE_D,
]

PAYLOAD_SIZE = {
    PACKET_TYPE_ALTITUDE:    4,
    PACKET_TYPE_COORDINATES: 8,
    PACKET_TYPE_C:           4,
    PACKET_TYPE_D:           1,
}

PAYLOAD_FORMAT = {
    PACKET_TYPE_ALTITUDE:    '>f',
    PACKET_TYPE_COORDINATES: '>ff',
    PACKET_TYPE_C:           '>i',
    PACKET_TYPE_D:           '>?',
}


CHECKSUM_CALCULATOR = crc.CrcCalculator(crc.Crc16.CCITT)


def get_packet_types(n: int):
    """
    Splits a packet type bitflag into multiple packet type integers.
    Code from <https://www.spatialtimes.com/2014/07/binary-flags-with-python/>

    - n: The integer to be interpreted as a bitflag.

    Usage: `for packet_type in get_packet_types(flags):`
    """
    while n:
        b = n & (~n+1)
        yield b
        n ^= b

def create_packet(types: int, data: tuple) -> bytes:
    """
    Constructs a binary packet.

    - types: Integer bitflag of packet types.
    - data: Tuple of data. This is the packet's payload, ordered according to the packet type priority.
    """

    header = struct.pack('>h', types)
    # footer = struct.pack('>h', checksum)

    body = bytes()
    type_flags: list[int] = get_packet_types(types)
    idx_data = 0
    for type_flag in type_flags:

        if type_flag == PACKET_TYPE_ALTITUDE:
            body = body + struct.pack('>f', data[idx_data])
            idx_data += 1
        
        elif type_flag == PACKET_TYPE_COORDINATES:
            body = body + struct.pack('>ff', data[idx_data], data[idx_data + 1])
            idx_data += 2
        
        elif type_flag == PACKET_TYPE_C:
            body = body + struct.pack('>i', data[idx_data])
            idx_data += 1
        
        elif type_flag == PACKET_TYPE_D:
            body = body + struct.pack('>?', data[idx_data])
            idx_data += 1
    
    checksum = CHECKSUM_CALCULATOR.calculate_checksum(header + body)
    footer = struct.pack('>i', checksum)

    return header + body + footer

class IliadDataController(serial_data_controller.SerialDataController):

    def __init__(self) -> None:
        super().__init__()

        self.data_buffer = bytearray()
    
    def update(self) -> None:
        if self.is_open():
            
            # Poll serial port and put anything there into data_buffer
            if self.port.in_waiting > 0:
                data = self.port.read_all()
                self.data_buffer += bytearray(data)
            
            # If the buffer has enough data, try to parse some of it
            if len(self.data_buffer) > 10:
                
                self.idx_cursor = 0 # Current index in data_buffer.

                packet_types_bytes: bytes
                packet_types: list[int] = []
                packet_payload_bytes = bytearray()
                packet_payload: dict[int] = {}
                packet_checksum_bytes: bytes
                packet_checksum: int

                # Parse header:
                if len(self.data_buffer) - self.idx_cursor >= 2:
                    packet_types_bytes = self.data_buffer[self.idx_cursor:(self.idx_cursor + 2)]
                    (packet_types_raw,) = struct.unpack('>h', packet_types_bytes)
                    packet_types: list[int] = list(packet_util.get_packet_types(packet_types_raw))
                    self.idx_cursor += 2
                
                for packet_type in PACKET_TYPES:
                    if packet_type in packet_types:
                        payload_size = PAYLOAD_SIZE[packet_type]
                        payload_format = PAYLOAD_FORMAT[packet_type]
                        if len(self.data_buffer) - self.idx_cursor >= payload_size:
                            payload_bytes = self.data_buffer[self.idx_cursor:(self.idx_cursor + payload_size)]
                            payload = struct.unpack(payload_format, payload_bytes)
                            self.idx_cursor += payload_size

                            packet_payload_bytes += payload_bytes
                            packet_payload[packet_type] = payload
                
                # Parse footer:
                if len(self.data_buffer) - self.idx_cursor >= 4:
                    packet_checksum_bytes = self.data_buffer[self.idx_cursor:(self.idx_cursor + 4)]
                    (packet_checksum,) = struct.unpack('>i', packet_checksum_bytes)
                    self.idx_cursor += 4
                
                # Check packet checksum:
                is_ok = False
                if CHECKSUM_CALCULATOR.verify_checksum(packet_types_bytes + packet_payload_bytes, packet_checksum):
                    is_ok = True
                
                if is_ok:
                    print(f'[OK] {packet_types_bytes.hex()} {packet_payload_bytes.hex()} {packet_checksum_bytes.hex()}')
                    print(f'\t{packet_types} {packet_payload} {packet_checksum}')
                else:
                    print(f'[BAD] {packet_types_bytes.hex()} {packet_payload_bytes.hex()} {packet_checksum_bytes.hex()}')
                    print(f'\tExpected {CHECKSUM_CALCULATOR.calculate_checksum(packet_types_bytes + packet_payload_bytes)}, got {packet_checksum}')

                if is_ok:
                    self.data_buffer = self.data_buffer[self.idx_cursor:]
                    self.idx_cursor = 0
                else:
                    self.data_buffer = self.data_buffer[1:]
                    self.idx_cursor = 0
                    # TODO: Need to track if we are in "network recover" state.
                
                # The basic algorithm for recovery is:
                # If crc doesn't match up, discard one byte at a time and try to parse again.

# Test cases
if __name__ == '__main__':

    # Test update()
    test = IliadDataController()
    test.set_config({
        'port_name': 'COM2',
        'port_baud_rate': 9600,
        'port_stop_bits': serial.STOPBITS_ONE,
        'port_parity': serial.PARITY_NONE,
        'port_byte_size': serial.EIGHTBITS,
    })
    test.open()
    test.update()
    test.close()
    
    # Test packet parsing
    test = IliadDataController()
    test.set_config({
        'port_name': 'COM2',
        'port_baud_rate': 9600,
        'port_stop_bits': serial.STOPBITS_ONE,
        'port_parity': serial.PARITY_NONE,
        'port_byte_size': serial.EIGHTBITS,
    })
    test.open()
    with serial.Serial(
        port='COM1',
        baudrate=9600,
        stopbits=serial.STOPBITS_ONE,
        parity=serial.PARITY_NONE,
        bytesize=serial.EIGHTBITS,
        timeout=3,
        write_timeout=3,
    ) as port:
        port.write(packet_util.create_packet(packet_util.PACKET_TYPE_ALTITUDE, (42.0,)))
        for i in range(1000):
            test.update()
    test.close()

