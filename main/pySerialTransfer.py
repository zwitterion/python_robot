import os
import serial
import serial.tools.list_ports
from CRC import CRC


class InvalidSerialPort(Exception):
    pass


CONTINUE = 2
NEW_DATA = 1
NO_DATA = 0
CRC_ERROR = -1
PAYLOAD_ERROR = -2
STOP_BYTE_ERROR = -3

START_BYTE = 0x7E
STOP_BYTE = 0x81

MAX_PACKET_SIZE = 0xFE

find_start_byte = 0
find_overhead_byte = 1
find_payload_len = 2
find_payload = 3
find_crc = 4
find_end_byte = 5


def msb(val):
    return byte_val(val, num_bytes(val) - 1)


def lsb(val):
    return byte_val(val, 0)


def byte_val(val, pos):
    return int.from_bytes(((val >> (pos * 8)) & 0xFF).to_bytes(2, 'big'), 'big')


def num_bytes(val):
    num_bits = val.bit_length()
    num_bytes = num_bits // 8

    if num_bits % 8:
        num_bytes += 1
    
    if not num_bytes:
        num_bytes = 1

    return num_bytes


def constrain(val, min_, max_):
    if val < min_:
        return min_
    elif val > max_:
        return max_
    return val


def open_ports():
    '''
    Description:
    ------------
    Lists serial port names

    :return port_list: list - all serial ports currently available
    '''
    port_list = []

    for port in serial_ports():
        try:
            s = serial.Serial(port)
            s.close()
            port_list.append(port)
        except (OSError, serial.SerialException):
            pass

    return port_list


def serial_ports():
    return [p.device for p in serial.tools.list_ports.comports(include_links=True)]


class SerialTransfer(object):
    def __init__(self, port, baud=115200, restrict_ports=True):
        '''
        Description:
        ------------
        Initialize transfer class and connect to the specified USB device

        :param port: int or str - port the USB device is connected to
        :param baud: int        - baud (bits per sec) the device is configured for
        :param restrict_ports: boolean   - Only allow port selection from auto detected list. Default True.

        :return: void
        '''

        self.txBuff = [' ' for i in range(MAX_PACKET_SIZE - 1)]
        self.rxBuff = [' ' for i in range(MAX_PACKET_SIZE - 1)]

        self.bytesRead = 0
        self.status = 0
        self.overheadByte = 0xFF

        self.state = find_start_byte
        if restrict_ports:
            self.port_name = None
            for p in serial_ports():
                if p == port or os.path.split(p)[-1] == port:
                    self.port_name = p
                    break

            if self.port_name is None:
                raise InvalidSerialPort('Invalid serial port specified.\
                    Valid options are {ports},  but {port} was provided'.format(
                    **{'ports': serial_ports(), 'port': port}))
        else:
            self.port_name = port

        self.crc = CRC()
        self.connection = serial.Serial()
        self.connection.port = self.port_name
        self.connection.baudrate = baud

    def open(self):
        '''
        Description:
        ------------
        Open serial port and connect to device if possible

        :return: bool - True if successful, else False
        '''

        if not self.connection.is_open:
            try:
                self.connection.open()
                return True
            except serial.SerialException as e:
                print(e)
                return False
        return True

    def close(self):
        '''
        Description:
        ------------
        Close serial port

        :return: void
        '''
        if self.connection.is_open:
            self.connection.close()

    def calc_overhead(self, pay_len):
        '''
        Description:
        ------------
        Calculates the COBS (Consistent Overhead Stuffing) Overhead
        byte and stores it in the class's overheadByte variable. This
        variable holds the byte position (within the payload) of the
        first payload byte equal to that of START_BYTE

        :param pay_len: int - number of bytes in the payload

        :return: void
        '''

        self.overheadByte = 0xFF

        for i in range(pay_len):
            if self.txBuff[i] == START_BYTE:
                self.overheadByte = i
                break

    def find_last(self, pay_len):
        '''
        Description:
        ------------
        Finds last instance of the value START_BYTE within the given
        packet array

        :param pay_len: int - number of bytes in the payload

        :return: int - location of the last instance of the value START_BYTE
                       within the given packet array
        '''

        if pay_len <= MAX_PACKET_SIZE:
            for i in range(pay_len - 1, 0, -1):
                if self.txBuff[i] == START_BYTE:
                    return i
        return -1

    def stuff_packet(self, pay_len):
        '''
        Description:
        ------------
        Enforces the COBS (Consistent Overhead Stuffing) ruleset across
        all bytes in the packet against the value of START_BYTE

        :param pay_len: int - number of bytes in the payload

        :return: void
        '''

        refByte = self.find_last(pay_len)

        if (not refByte == -1) and (refByte <= MAX_PACKET_SIZE):
            for i in range(pay_len - 1, 0, -1):
                if self.txBuff[i] == START_BYTE:
                    self.txBuff[i] = refByte - i
                    refByte = i

    def send(self, message_len):
        '''
        Description:
        ------------
        Send a specified number of bytes in packetized form

        :param message_len: int - number of bytes from the txBuff to send as
                                  payload in the packet

        :return: bool - whether or not the operation was successful
        '''

        stack = []
        message_len = constrain(message_len, 0, MAX_PACKET_SIZE)

        try:
            self.calc_overhead(message_len)
            self.stuff_packet(message_len)
            found_checksum = self.crc.calculate(self.txBuff, message_len)

            stack.append(START_BYTE)
            stack.append(self.overheadByte)
            stack.append(message_len)

            for i in range(message_len):
                if type(self.txBuff[i]) == str:
                    val = ord(self.txBuff[i])
                else:
                    val = int(self.txBuff[i])

                stack.append(val)

            stack.append(found_checksum)
            stack.append(STOP_BYTE)

            stack = bytearray(stack)

            if self.open():
                self.connection.write(stack)
                # temp
                #for c in stack:
                #    print(c)

            return True

        except:
            import traceback
            traceback.print_exc()

            return False

    def unpack_packet(self, pay_len):
        '''
        Description:
        ------------
        Unpacks all COBS-stuffed bytes within the array

        :param pay_len: int - number of bytes in the payload

        :return: void
        '''

        testIndex = self.recOverheadByte
        delta = 0

        if testIndex <= MAX_PACKET_SIZE:
            while self.rxBuff[testIndex]:
                delta = self.rxBuff[testIndex]
                self.rxBuff[testIndex] = START_BYTE
                testIndex += delta

            self.rxBuff[testIndex] = START_BYTE

    def read_data(self, callback):
        '''
        Description:
        ------------
        Parses incoming serial data, analyzes packet contents,
        and reports errors/successful packet reception

        '''

        if self.open():
            while True:
                recChar = int.from_bytes(self.connection.read(1),
                                            byteorder='big')

                if self.state == find_start_byte:
                    if recChar == START_BYTE:
                        self.state = find_overhead_byte

                elif self.state == find_overhead_byte:
                    self.recOverheadByte = recChar
                    self.state = find_payload_len

                elif self.state == find_payload_len:
                    if recChar <= MAX_PACKET_SIZE:
                        self.bytesToRec = recChar
                        self.payIndex = 0
                        self.state = find_payload
                    else:
                        self.bytesRead = 0
                        self.state = find_start_byte
                        self.status = PAYLOAD_ERROR
                        return self.bytesRead

                elif self.state == find_payload:
                    if self.payIndex < self.bytesToRec:
                        self.rxBuff[self.payIndex] = recChar
                        self.payIndex += 1

                        if self.payIndex == self.bytesToRec:
                            self.state = find_crc

                elif self.state == find_crc:
                    found_checksum = self.crc.calculate(
                        self.rxBuff, self.bytesToRec)

                    if found_checksum == recChar:
                        self.state = find_end_byte
                    else:
                        self.bytesRead = 0
                        self.state = find_start_byte
                        self.status = CRC_ERROR
                        self.state = find_start_byte

                elif self.state == find_end_byte:
                    self.state = find_start_byte

                    if recChar == STOP_BYTE:
                        self.unpack_packet(self.bytesToRec)
                        self.bytesRead = self.bytesToRec
                        self.status = NEW_DATA
        
                        # success
                        callback(self.rxBuff, self.bytesRead)
                        self.bytesRead = 0

                    self.bytesRead = 0
                    self.state = find_start_byte

                else:
                    print('ERROR: Undefined state: {}'.format(self.state))

                    self.bytesRead = 0
                    self.state = find_start_byte
                    #return self.bytesRead

        return 


if __name__ == '__main__':
    try:
        link = SerialTransfer('COM13')

        link.txBuff[0] = 'h'
        link.txBuff[1] = 'i'
        link.txBuff[2] = '\n'

        link.send(3)

        while not link.available():
            if link.status < 0:
                print('ERROR: {}'.format(link.status))

        print('Response received:')

        response = ''
        for index in range(link.bytesRead):
            response += chr(link.rxBuff[index])

        print(response)
        link.close()

    except KeyboardInterrupt:
        link.close()
