import time

def mb_master(trame, data_in, data_out):
    # Dummy implementation of Mb_master function
    # Replace with actual implementation
    return 0

def process_modbus(rs232, trame, data_in, data_out):
    result = mb_master(trame, data_in, data_out)

    if result < 0:
        if result == -1:
            rs232.write("send 16 MBRTU error : unknown function\r\n")
        elif result == -2:
            rs232.write("send 16 MBRTU crc error\r\n")
        elif result == -3:
            rs232.write("send 16 MBRTU timeout error\r\n")
        elif result == -4:
            rs232.write("send 16 MBRTU error : func 16 bad slave answer\r\n")

    rs232.flush()

    trame['function'] = 3
    trame['address'] = 200
    trame['length'] = 30

    time.sleep(0.005)  # NutSleep(5)

    result = mb_master(trame, data_in, data_out)

    if result < 0:
        if result == -1:
            rs232.write("read 3 MBRTU error : unknown function\r\n")
        elif result == -2:
            rs232.write("read 3 MBRTU crc error\r\n")
        elif result == -3:
            rs232.write("read 3 MBRTU timeout error\r\n")
        elif result == -4:
            rs232.write("read 3 MBRTU error : function code 3 bad slave answer\r\n")

def read_rs232(rs232):
    while True:
        line = rs232.readline()
        if line:
            print(f"Received: {line.strip()}")
        time.sleep(1)

# Beispielaufruf der Funktion
class MockRS232:
    def write(self, message):
        print(message, end='')

    def flush(self):
        pass

    def readline(self):
        # Dummy implementation for reading from RS232
        return "dummy data from RS232\n"

rs232 = MockRS232()
trame = {'function': 16, 'address': 0, 'length': 0}
data_in = [0] * 30
data_out = [0] * 30

# Start RS232 reading in a separate thread
import threading
rs232_thread = threading.Thread(target=read_rs232, args=(rs232,))
rs232_thread.daemon = True
rs232_thread.start()

# Process Modbus communication
process_modbus(rs232, trame, data_in, data_out)
