import serial


def open_serial(port, baud, timeout):
    ser = serial.Serial(port=port, baudrate=baud, timeout=timeout)
    if ser.isOpen():
        return ser
    else:
        print("SERIAL ERROR")


def close(ser):
    ser.close()


def write_data(ser, data):
    ser.write(data)


def read_data(ser, size=1):
    return ser.read(size)


def to_hex(val):
    return chr(val)


def decode_data(data):
    res = ""
    for d in data:
        res += hex(ord(d)) + " "

    return res


def checksum(data):
    return (~data) & 0xff


if __name__ == "__main__":

    # Header1
    header1 = 0xFF

    # Header2
    header2 = 0xFF
    
    # we open the port
    serial_port = open_serial("/dev/ttyACM0", 1000000, timeout=0.1)

    #id of the motor (here 1), you need to change
    data_id = 61

    # instruction write= 0x03
    data_instruction = 0x03

    # lenght of the packet
    data_lenght = 0x04

    # LED address=0x19
    data_param1 = 0x19  

    # write 0x01
    data_param2 = 0x01 

    # checksum
    data_checksum = checksum(
        data_id + data_lenght + data_instruction + data_param1 + data_param2
    )
    to_send=[header1, header2, data_id, data_lenght, data_instruction, data_param1, data_param2, data_checksum]

    to_send_bytes= bytes(to_send)

    serial_port.write(to_send_bytes)
    print(to_send_bytes)
    
    #header1=OxFF header2=OxFF data_id=61 data_lenght=0x04 data_instruction=0x03 data_param1=0x19 data_param2=0x01 checksum