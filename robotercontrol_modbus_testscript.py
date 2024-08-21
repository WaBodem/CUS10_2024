

'''
Licht, Kamera und Laser werden bereits über mqtt gesteuert - keine modbus integration mehr notwendig

nur Gasalarm und Halt Ref, Park, Blinklicht und Schachtlicht sowie Joystick und Hub und CD, CS, KD, KS
KS 151
CD 152
HUB 154


'''

from pymodbus.client import ModbusSerialClient
from time import sleep, time
import random
import threading


def send_random_values(stop_event):
    value=40
    address = 150
    #Füllen der Register 150 bis 155 mit Zufallswerten zwischen 0 und 1023
    values_to_write = [512 for _ in range(6)]
    values_to_write.append(1)  # Letztes Register auf 0 setzen
    success = modbus.write_multiple_registers(slave=1, address=address, values=values_to_write)
    print(f"Write successful: {success}")
    print(f"Written values: {values_to_write}")

    while not stop_event.is_set():
        #erhöhre einen zähler um 1 und sende diesen an die SPS register 171 per modbus  
        address=171
        value = (value + 1) if (value + 1) <= 65535 else 40  # Erhöhe den Wert und setze ihn auf 40, wenn er 65535 erreicht
        success = modbus.write_multiple_registers(slave=1, address=address, values=[value])
        #print(f"Write successful: {success}")
        sleep(0.005)  # 5 ms warten
               # Lesen des Werts aus dem Register 171 zur Überprüfung
        # registers = modbus.read_holding_registers(slave=1, address=address, count=1)
        # read_value = registers[0]
        # print(f"Read value from register {address}: {read_value}")
        # sleep(0.005)



class Modbus:
    def __init__(self, port, baudrate, bytesize=8, parity='N', stopbits=1, timeout=1):
        try:
            print(f"Opening serial port: {port} with baudrate: {baudrate}, bytesize: {bytesize}, parity: {parity}, stopbits: {stopbits}, timeout: {timeout}")
            self.client = ModbusSerialClient(
                port=port,
                baudrate=baudrate,
                bytesize=bytesize,
                parity=parity,
                stopbits=stopbits,
                timeout=timeout
            )
            connection = self.client.connect()
            if connection:
                print("Serial port opened successfully")
            else:
                raise Exception("Failed to open serial port")
        except Exception as e:
            print(f"Error opening serial port: {e}")
            raise

    def read_holding_registers(self, slave, address, count):
        response = self.client.read_holding_registers(address, count, slave=slave)
        if not response.isError():
            return response.registers
        else:
            print("Error reading registers")
            return None

    def write_multiple_registers(self, slave, address, values):
        response = self.client.write_registers(address, values, slave=slave)
        if not response.isError():
            return True
        else:
            print("Error writing registers")
            return False

    def close(self):
        if self.client:
            print("Closing serial port")
            self.client.close()
            print("Serial port closed")

# Methode zum expliziten Schließen der seriellen Schnittstelle
def close_serial_port(port):
    try:
        client = ModbusSerialClient(port=port)
        if client.connect():
            client.close()
            print(f"Serial port {port} closed")
    except Exception as e:
        print(f"Error closing serial port {port}: {e}")

# Beispiel zur Verwendung der Klasse
modbus = None
try:
    close_serial_port('/dev/ttyACM0')  # Schließen der seriellen Schnittstelle, falls sie noch geöffnet ist
    modbus = Modbus(port='/dev/ttyACM0', baudrate=38400, bytesize=8, parity='E', stopbits=1)

    # Event zum Stoppen des Threads
    stop_event = threading.Event()
    # Starten des Threads
    thread_sendrandom = threading.Thread(target=send_random_values, args=(stop_event,)
                                         )
    thread_sendrandom.daemon = True
    thread_sendrandom.start()
    sleep(1)
    
    #write to holding register 156  1 joystick init ok
    address=156
    value =0 #0=False 1=True
    success = modbus.write_multiple_registers(slave=1, address=address, values=[value]) 
    print(f"87 Write 'Joystick init ok' successful: {success}")

    
    # Lesen der Register 100 bis 106
    address=150
    registers = modbus.read_holding_registers(slave=1, address=address, count=6)
    print(f"66 Read Holding Registers {address}: {registers}")


    #Lesen der Register 200 bis 230
    start_address = 200
    register_count = 30  #Antwortspeicher 200-230
    registers = modbus.read_holding_registers(slave=1, address=start_address, count=register_count)
    print(f"Registers 200-230: {registers}")

    
    address=159
    registers = modbus.read_holding_registers(slave=1, address=address, count=1)
    print(f"Registers 159: {registers}")    
    address=160 
    registers = modbus.read_holding_registers(slave=1, address=address, count=1)
    print(f"Registers 160: {registers}")    
    
    #lese antwortspeicher 211-214
    address=211
    registers = modbus.read_holding_registers(slave=1, address=address, count=3)
    print(f"Registers 211-214: {registers}")
    # Wandle Antwortspeicher in String um und extrahiere nur die ersten drei Zeichen
    response = ''.join([chr(x) for x in registers[:3]])
    print(f"89 Antwort auf Befehl L1: {response}")

    # Füllen der Register 150 bis 155 mit Zufallswerten zwischen 0 und 1023
    # values_to_write = [random.randint(0, 1023) for _ in range(6)]
    # address=150
    # success = modbus.write_multiple_registers(slave=1, address=address, values=values_to_write)
    # print(f"Write successful: {success}")
    # print(f"Written values: {values_to_write}")
    
    # #write to holding register 159 'G'
    # address=159
    # value = 71
    # success = modbus.write_multiple_registers(slave=1, address=address, values=[value]) 
    # print(f"86 Write 'G' successful: {success}")
    
    
#befehlsspeicher in sps zurücksetzen adresse 159 wert 0
    address=159
    value = 0   
    success = modbus.write_multiple_registers(slave=1, address=address, values=[value])
    
    
    #write to holding register 159 160 'L' '1' - Schachtlicht Ein 1 Aus 0
    address=159 
    value = 76 #'L'
    success = modbus.write_multiple_registers(slave=1, address=address, values=[value])
    print(f"115 Write 'L' successful: {success}")
    address=160
    value = 49 #49 = 1 48 = 0
    success = modbus.write_multiple_registers(slave=1, address=address, values=[value])
    print(f"119 Write '1' successful: {success}")       

    #befehlsspeicher in sps zurücksetzen adresse 159 wert 0
    address=159
    value = 0   
    success = modbus.write_multiple_registers(slave=1, address=address, values=[value])
    
    
    #write to holding register 159 160 'B' '1' Schachtlicht Ein 1 Aus 0
    address=159 
    value = 66 #'B'
    success = modbus.write_multiple_registers(slave=1, address=address, values=[value])
    print(f"131 Write 'B' successful: {success}")
    address=160
    value = 49 #49 = 1 48 = 0
    success = modbus.write_multiple_registers(slave=1, address=address, values=[value])
    print(f"135 Write '1' successful: {success}")            
                    
   #befehlsspeicher in sps zurücksetzen adresse 159 wert 0
    address=159
    value = 0   
    success = modbus.write_multiple_registers(slave=1, address=address, values=[value])

    #write to holding register 159 160 'G' Roboter Freigabe
    address=159 
    value = 71 #'G
    success = modbus.write_multiple_registers(slave=1, address=address, values=[value])
    print(f"146 Write 'G' successful: {success}")
   
     
    #befehlsspeicher in sps zurücksetzen adresse 159 wert 0
    address=159
    value = [0,0,0,0,0,0]
    success = modbus.write_multiple_registers(slave=1, address=address, values=value)

    #referenzwerte in sps zurücksetzen adresse 20-25 wert 0
    address=20
    value = [48,48,48,48,48,48]
    success = modbus.write_multiple_registers(slave=1, address=address, values=value)

    #write to holding register 158 160 'Ref' Roboter Refernzieren
    address=50 
    values = [82,48]#'R A referenzieren alle Achsen
    success = modbus.write_multiple_registers(slave=1, address=address, values=values)
    print(f"159 Write 'R' successful: {success}")
    
    address=159 
    values = [82,65]#'R A referenzieren alle Achsen
    success = modbus.write_multiple_registers(slave=1, address=address, values=values)
    print(f"159 Write 'R' successful: {success}")
    
   #write to holding register 158 160 'Ref not ok' Roboter Refernzieren
    address=25 
    values = 48#'noch nicht refereziert
    success = modbus.write_multiple_registers(slave=1, address=address, values=values)
    print(f"208 Write 'VR25 0' successful: {success}")

    # #lese antwortspeicher 150-180
    # address=150
    # registers = modbus.read_holding_registers(slave=1, address=address, count=30)
    # print(f"213 Registers 150-180: {registers}")
   


    # #lese antwortspeicher 211-214
    # address=200
    # registers = modbus.read_holding_registers(slave=1, address=address, count=27)
    # print(f"223 Registers 200-230: {registers}")
    #warte bis referenzierung abgeschlossen
    while True:
        address=25
        registers = modbus.read_holding_registers(slave=1, address=address, count=1)
        #print(f"229 Registers 25: {registers}")
        if registers[0] == 63:
            break
        sleep(.25)
    #print registers 20-25
    address=20
    registers = modbus.read_holding_registers(slave=1, address=address, count=6)
    print(f"236 Registers 20-25: {registers}")
      #move über joystick aktivieren
    address=3 
    values = 49#'1=True möglich
    success = modbus.write_multiple_registers(slave=1, address=address, values=values)
    print(f"240 Write 'VR3 1' successful: {success}")
    #print status kommunikation
    address=5
    registers = modbus.read_holding_registers(slave=1, address=address, count=1)
    print(f"236 Kom_ok Status 5: {registers[0]}")
    #print status joystick
    address=156
    registers = modbus.read_holding_registers(slave=1, address=address, count=1)
    print(f"254 jOystick Status 156 ok wenn 49: {registers[0]}")
  #kommunikations id
    address=171
    registers = modbus.read_holding_registers(slave=1, address=address, count=1)
    sleep(.002)
    print(f"253 Registers 171: {registers[0]}")
    #aktueller speed hub
    address=103
    registers = modbus.read_holding_registers(slave=1, address=address, count=1)
    sleep(.002)
    print(f"253 Speed Hub 103: {registers}")


 #input   
    input(  "279 Press Enter to continue... nach dem referenzieren vor dem fahren von hub und cd")
    print("280 Start fahren von hub")
    #move hub mit joystick
    #move_Befehl  0 setzen
    address=6 #move_Befehl
    value = 0  
    success = modbus.write_multiple_registers(slave=1, address=address, values=value)
    address=4 #keypad_Befehl
    value = 0  
    success = modbus.write_multiple_registers(slave=1, address=address, values=value)

    address=82
    registers = modbus.read_holding_registers(slave=1, address=address, count=1)
    sleep(.002)
    print(f"275 Hub Lock 82: {registers}")
    address=3 #move_ok
    value = 1  
    success = modbus.write_multiple_registers(slave=1, address=address, values=value)

    # #joystick init ok
    # address=156
    # value = 1 #1=True 0=False
    # success = modbus.write_multiple_registers(slave=1, address=address, values=[value])
    #fahre hub
    #schreibe in den Befehlsspeicher 154 den wert 305 im takt von 150ms für die dauer von 3sekunden
    endtime=time()+3
    while time() < endtime:
        # Schreiben des v Hub wertes in das Register
        address=150
        values = [512,512,512,512,1000,512,1] # achsen1-5 6=KLicht 7=joystick init ok 
        success = modbus.write_multiple_registers(slave=1, address=address, values=values)
        if success:
            print(f"294 Write successful: {success}")
        else:
            print(f"296 Write failed: {success}")
        sleep(0.003)

        # Lesen des kom statuses aus reg 5
        address = 5
        result=[]
        result = modbus.read_holding_registers(slave=1, address=address, count=1)
        sleep(0.001)
        print(f"308 Read kom_ok status 5: {result[0]}")
    sleep(0.001)
    address=150
    values = [512,512,512,512,512,512,1] # achsen1-5 6=joystick init ok 
    success = modbus.write_multiple_registers(slave=1, address=address, values=values)

    #fahre CD
    print("331 Start fahren von CD")
    #schreibe in den Befehlsspeicher 154 den wert 305 im takt von 150ms für die dauer von 3sekunden
    endtime=time()+3
    while time() < endtime:
        # Schreiben des v Hub wertes in das Register
        address=150
        values = [512,512,900,512,512,512,1] # achsen1-5 6=joystick init ok 
        success = modbus.write_multiple_registers(slave=1, address=address, values=values)
        if success:
            print(f"294 Write successful: {success}")
        else:
            print(f"296 Write failed: {success}")
        sleep(0.003)

        # Lesen des kom statuses aus reg 5
        address = 5
        result=[]
        result = modbus.read_holding_registers(slave=1, address=address, count=1)
        sleep(0.001)
        print(f"308 Read kom_ok status 5: {result[0]}")
    sleep(0.001)
    address=150
    values = [512,512,512,512,512,512,1] # achsen1-5 6=joystick init ok 
    success = modbus.write_multiple_registers(slave=1, address=address, values=values)

   #fahre KS
    print("357 Start fahren von CD")
    #schreibe in den Befehlsspeicher 154 den wert 305 im takt von 150ms für die dauer von 3sekunden
    endtime=time()+2
    while time() < endtime:
        # Schreiben des v Hub wertes in das Register
        address=150
        values = [512,700,512,512,512,512,1] # achsen1-5 6=joystick init ok 
        success = modbus.write_multiple_registers(slave=1, address=address, values=values)
        if success:
            print(f"294 Write successful: {success}")
        else:
            print(f"296 Write failed: {success}")
        sleep(0.003)

        # Lesen des kom statuses aus reg 5
        address = 5
        result=[]
        result = modbus.read_holding_registers(slave=1, address=address, count=1)
        sleep(0.001)
        print(f"308 Read kom_ok status 5: {result[0]}")
    sleep(0.001)
    address=150
    values = [512,512,512,512,512,512,1] # achsen1-5 6=joystick init ok 
    success = modbus.write_multiple_registers(slave=1, address=address, values=values)

#fahre CS
    print("383 Start fahren von CS")
    #schreibe in den Befehlsspeicher 154 den wert 305 im takt von 150ms für die dauer von 3sekunden
    endtime=time()+3
    while time() < endtime:
        # Schreiben des v Hub wertes in das Register
        address=150
        values = [512,512,512,800,512,512,1] # achsen1-5 6=joystick init ok 
        success = modbus.write_multiple_registers(slave=1, address=address, values=values)
        if success:
            print(f"294 Write successful: {success}")
        else:
            print(f"296 Write failed: {success}")
        sleep(0.003)

        # Lesen des kom statuses aus reg 5
        address = 5
        result=[]
        result = modbus.read_holding_registers(slave=1, address=address, count=1)
        sleep(0.001)
        print(f"308 Read kom_ok status 5: {result[0]}")
    sleep(0.001)
    address=150
    values = [512,512,512,512,512,512,1] # achsen1-5 6=joystick init ok 
    success = modbus.write_multiple_registers(slave=1, address=address, values=values)

#fahre KD
    print("409 Start fahren von KD")
    #schreibe in den Befehlsspeicher 154 den wert 305 im takt von 150ms für die dauer von 3sekunden
    endtime=time()+3
    while time() < endtime:
        # Schreiben des v Hub wertes in das Register
        address=150
        values = [800,512,512,512,512,512,1] # achsen1-5 6=joystick init ok 
        success = modbus.write_multiple_registers(slave=1, address=address, values=values)
        if success:
            print(f"294 Write successful: {success}")
        else:
            print(f"296 Write failed: {success}")
        sleep(0.003)

        # Lesen des kom statuses aus reg 5
        address = 5
        result=[]
        result = modbus.read_holding_registers(slave=1, address=address, count=1)
        sleep(0.001)
        print(f"308 Read kom_ok status 5: {result[0]}")
    sleep(0.001)
    address=150
    values = [512,512,512,512,512,512,1] # achsen1-5 6=joystick init ok 
    success = modbus.write_multiple_registers(slave=1, address=address, values=values)


    #akt analogspeicher v u licht lesen
    address=100
    registers = modbus.read_holding_registers(slave=1, address=address, count=6)
    sleep(.002)
    print(f"316 v  analog 100-105: {registers}")
    #kommunikations id
    address=171
    registers = modbus.read_holding_registers(slave=1, address=address, count=1)
    sleep(.002)
    print(f"321 Kom_ok 171: {registers[0]}")
    #befehlsspeicher in sps zurücksetzen adresse 159 wert 0
    address=159
    value = 0   
    success = modbus.write_multiple_registers(slave=1, address=address, values=value)
    
#input
    input(  "370 Press Enter to continue...nach dem fahren vor dem parken")
   #write to holding register 158 160 'Ref not ok' Roboter Refernzieren
    address=25 
    value = 49#'noch nicht refereziert
    success = modbus.write_multiple_registers(slave=1, address=address, values=values)
    print(f"208 Write 'VR25 1' successful: {success}")
  #write to holding register 158 160 'Ref not ok' Roboter Refernzieren
    address=50 
    values = [80,65]#'P
    success = modbus.write_multiple_registers(slave=1, address=address, values=values)
    print(f"242 Write 'VR50 80 65' successful: {success}")

    #write to holding register 159 160 'G' Roboter Freigabe
    addresss=159 
    values= [80,65] #'P A
    success = modbus.write_multiple_registers(slave=1, address=address, values=values)
    print(f"248 Write 'P A' successful: {success}")
    sleep(.02)
  #bus park register 158 160 'P' 'A' Roboter Parken
    address=158
    registers = modbus.read_holding_registers(slave=1, address=address, count=1)
    sleep(.002)
    print(f"350 Registers 158: {registers}")
  
#input  
    input(  "Press Enter to continue...nach dem Parken")
                     
   #befehlsspeicher in sps zurücksetzen adresse 159 wert 0
    address=159
    value = 0   
    success = modbus.write_multiple_registers(slave=1, address=address, values=[value])

    #write to holding register 159 160 'V' Roboterversion abfagen
    value = 86 # V
    success = modbus.write_multiple_registers(slave=1, address=address, values=[value])
    print(f"254 Write 'V' successful: {success}")
    sleep(.02)
   #lese antwortspeicher 211-214
    address=211
    registers = modbus.read_holding_registers(slave=1, address=address, count=8)
    sleep(.002)
    print(f"260 Registers 211: {registers}")
    # Wandle Antwortspeicher in String um und extrahiere nur die ersten drei Zeichen
    response = ''.join([chr(x) for x in registers[:8]])
    print(f"263 Antwort auf Befehl V: {response}")

    # #lese antwortspeicher 200-210
    # address=200
    # registers = modbus.read_holding_registers(slave=1, address=address, count=10)
    # print(f"Registers 211-218: {registers}")
    # # Wandle Antwortspeicher in String um und extrahiere nur die ersten drei Zeichen
    # response = ''.join([chr(x) for x in registers[:1]])
    # print(f"268 Antwort auf Achsabfrage: {response}")
    # response = ''.join([chr(x) for x in registers[2:3]])
    # print(f"270 Antwort auf Achsabfrage: {response}")
    # response = ''.join([chr(x) for x in registers[4:5]])
    # print(f"272 Antwort auf Achsabfrage: {response}")
    
    #blinklicht aus und schachtlicht aus
    
    #befehlsspeicher in sps zurücksetzen adresse 159 wert 0
    address=159
    value = 0   
    success = modbus.write_multiple_registers(slave=1, address=address, values=[value])
    
    #write to holding register 159 160 'L' '0' - Schachtlicht Ein 1 Aus 0
    address=159
    value = 76 #'L'
    success = modbus.write_multiple_registers(slave=1, address=address, values=[value])
    print(f"277 Write 'L' successful: {success}")
    address=160
    value = 48 #49 = 1 48 = 0
    success = modbus.write_multiple_registers(slave=1, address=address, values=[value]) 
    print(f"281 Write '0' successful: {success}")
    
    #befehlsspeicher in sps zurücksetzen adresse 159 wert 0
    address=159
    value = 0   
    success = modbus.write_multiple_registers(slave=1, address=address, values=[value])


    #write to holding register 159 160 'B' '0' Schachtlicht Ein 1 Aus 0
    address=159
    value = 66 #'B'
    success = modbus.write_multiple_registers(slave=1, address=address, values=[value])
    print(f"286 Write 'B' successful: {success}")
    address=160
    value = 48 #49 = 1 48 = 0
    success = modbus.write_multiple_registers(slave=1, address=address, values=[value])
    print(f"290 Write '0' successful: {success}")

        
                 
    #stop and kill thread thread_sendrandom
    sleep(1)
    print("Stopping thread...")
    stop_event.set()
    thread_sendrandom.join()
    print("Thread stopped")
    modbus.close()
    print("Modbus closed")
    print("Script exit")

finally:
    if modbus:
        modbus.close()
