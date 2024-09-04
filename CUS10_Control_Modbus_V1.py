#!/usr/bin/python3
# -*- coding:utf-8 -*-

'''
Licht, Kamera und Laser werden bereits über mqtt gesteuert - keine modbus integration mehr notwendig

nur Gasalarm und Halt Ref, Park, Blinklicht und Schachtlicht sowie Joystick und Hub und CD, CS, KD, KS
KS 151
CD 152
HUB 154

todo:
wenn modbus unterbricht dann mqtt message senden und neu starten oder 

'''

from pymodbus.client import ModbusSerialClient
from time import sleep, time
import random
import threading
import serial
import RPi.GPIO as GPIO
import paho.mqtt.client as mqtt
import cfg  # Stellen Sie sicher, dass cfg die BROKER-Konfiguration enthält
import setproctitle


#setup gpio asoutput for roboter  LED referenziert
GPIO.setmode(GPIO.BCM)

GPIO.setup(cfg.PIN_ROBOTERREFERENZIERT, GPIO.OUT)
GPIO.output(cfg.PIN_ROBOTERREFERENZIERT, GPIO.LOW)

#setup gpio asoutput for roboter  LED kommunikation
GPIO.setup(cfg.PIN_ROBOTERKOMMUNIKATION, GPIO.OUT)
GPIO.output(cfg.PIN_ROBOTERKOMMUNIKATION, GPIO.LOW)

#setup gpio asoutput for roboter  Ref/Park
GPIO.setup(cfg.PIN_REFPARK, GPIO.OUT)
GPIO.output(cfg.PIN_REFPARK, GPIO.LOW)

#set process title
setproctitle.setproctitle("CUS10_Control_Modbus_V1.py")

previous_register_206 = None
previous_register_207 = None
unchanged_counter = 0

bmodbusread = False


posdata_alt = [0] * 10  # Beispielpuffer für "data_alt"

data_rs232pc_in = [0] * 30  # Beispielpuffer für "data_rs232pc_in"


# MQTT-Konfiguration
broker = cfg.BROKER
client = mqtt.Client()

def on_connect(client, userdata, flags, rc):
    print("Verbunden mit dem Broker")

#def callback für mqtt on_message
def on_message(client, userdata, message):
    #print("Nachricht erhalten: ", str(message.payload.decode("utf-8")))
    #print("Nachricht erhalten: ", message.topic, message.payload.decode("utf-8"))
    #print("message.topic: ", message.topic)
    # wenn topi cfg.MQTT_ID+cfg.MQTT_TOPIC+cfg.JOYSTICKVAL_TOPIC+'/CHCS' dann setze wert in register 150
    #153 cs, 152 cd, 151 ks, 150 kd, 154 hub, 155 licht
    if message.topic == cfg.MQTT_ID+cfg.MQTT_TOPIC+cfg.JOYSTICKVAL_TOPIC+cfg.JOYSTICKMODBUSVR150_TOPIC:
        address=150
        values = message.payload.decode("utf-8")
        #remove [ and ] from string
        values = values.replace('[','').replace(']','')
        #create list of integer values of string values
        values = [int(x) for x in values.split(',')]
        #add int value 1 to list values for init joystick ok
        values.append(1)
        success = modbus.write_multiple_registers(slave=1, address=address, values=values)
        #print(f"Write successful: {success}")
        #joystick init led ok setzen, da sonst kein mqtt wert gesendet worden wäre
        GPIO.output(cfg.PIN_JOYSTICKINIT, GPIO.HIGH)    


#funct subscribe mqtt topics
def subscribe_mqtt_topics():
    #subscrib joystic values
    print("Subscribing to topic:", cfg.MQTT_ID+cfg.MQTT_TOPIC+cfg.JOYSTICKVAL_TOPIC+cfg.JOYSTICKMODBUSVR150_TOPIC)
    client.subscribe(cfg.MQTT_ID+cfg.MQTT_TOPIC+cfg.JOYSTICKVAL_TOPIC+cfg.JOYSTICKMODBUSVR150_TOPIC)


#setup gpio20 as input for Roboter_is_connected
GPIO.setmode(GPIO.BCM)
GPIO.setup(20, GPIO.IN, pull_up_down=GPIO.PUD_UP)

#setup gpio8 as output for  LED Roboter_is_connected
GPIO.setup(8, GPIO.OUT)

#wenn Roboter verbunden dann LED an
if GPIO.input(20) == 1:
    GPIO.output(8, GPIO.HIGH)
    print("Roboter verbunden")
else:
    GPIO.output(8, GPIO.LOW)
    print("Roboter nicht verbunden")

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
                print("36 Serial Modbus port opened successfully")
            else:
                raise Exception("Failed to open serial port")
        except Exception as e:
            print(f"43 Error opening serial port: {e}")
            raise
        
    def read_holding_registers(self, slave, address, count):
        response = self.client.read_holding_registers(address, count, slave=slave)
        if not response.isError():
            return response.registers
        else:
            print("51 Error reading registers")
            #exit the program - sende mqtt message dass modbus nicht verbunden und neu startet
            client.publish(cfg.BROKER, cfg.MQTT_ID+cfg.MQTT_TOPIC+cfg.OS_TOPIC, "Modbus nicht verbunden RESTART!")
            exit()
            return None

    def write_multiple_registers(self, slave, address, values):
        response = self.client.write_registers(address, values, slave=slave)
        if not response.isError():
            return True
        else: 
            print("58 Error writing registers")
            #exit the program - sende mqtt message dass modbus nicht verbunden und neu startet
            client.publish(cfg.BROKER, cfg.MQTT_ID+cfg.MQTT_TOPIC+cfg.OS_TOPIC, "Modbus nicht verbunden RESTART!")
            

    def close(self):
        if self.client:
            print("64 Closing serial port")
            self.client.close()
            print("66 Serial port closed")

# Beispielhafte Klasse, um die RS232-Kommunikation zu simulieren
class RS232:
    def __init__(self, port, baudrate=38400, timeout=1):
        self.ser = serial.Serial(port, baudrate, timeout=timeout)
        self.data_rs232pc_in = [0] * 35  # Beispielpuffer für "data_rs232pc_in"
        print("73 serial port "+ port +" opened")

    def read_from_serial(self, length):
        return self.ser.read(length)

    def write_to_serial(self, data):
        self.ser.write(data)

    def close(self):
        self.ser.close()


#read modbus daten
def read_modbus_data(modbus):
    global bmodbusread, posdata_alt
    try:
        while bmodbusread: #warte bis modbus frei zum lesen
            sleep(.001) #kurze pause um cpu zu entlasten
            print('x', end="")
        bmodbusread = True
        result = modbus.read_holding_registers(slave=1, address=200, count=30)
        sleep(.002)
        bmodbusread = False
        if hasattr(result, 'isError') and result.isError():
            print("Fehler beim Lesen der Modbus-Daten")
        elif isinstance(result, list):
            #prüfe ob in register 200-210  ein wert größer 0 vorhanden ist wenn ja dann print werte von register 200-210    
            if any(x > 0 and x < 65335 for x in result[0:10]):
                #speichere register 200-210 in posdata_alt und vergleiche mit neuem wert, wenn wert sich ändert dann print
                if posdata_alt != result[0:10]:
                    posdata_alt = result[0:10]
                    print("100 Modbus-Daten:", result[0:10], time())
              #else:
                #drucke cahr . ohne zeilenumbruch
                #print(".", end="")

        else:
            print("Unerwartetes Ergebnisformat:", result)
    except Exception as e:
        print(f"Fehler bei der Modbus-Kommunikation: {e}")




# RS232-Thread-Funktion
def rs232_thread(rs232):
    global data_rs232pc_in
    buffer = bytearray(35)  # Puffer für eingehende Daten (max 35 Zeichen)
    offset = 0
    new_command = False

    while True:
        # Daten von RS232 lesen
        got = rs232.read_from_serial(35 - offset)

        if got:
            buffer[offset:offset + len(got)] = got
            offset += len(got)
            # Prüfen, ob im gesamten buffer 13 und 10 stehen
            if 13 in buffer and 10 in buffer:
                # Finde die Position des ersten Vorkommens von 13 (CR)
                first_cr_index = buffer.index(13)
                new_command = True
                # Setze alle Zeichen im buffer ab dem ersten \r auf \x00
                for i in range(first_cr_index, len(buffer)):
                    buffer[i] = 0

            # Debug-Ausgabe, um den modifizierten buffer zu überprüfen
            cleaned_data = [byte for byte in buffer if byte != 0]
            print("cleaned_data:", cleaned_data)


        # Verarbeiten des empfangenen Befehls
        if new_command:
            
            #prüfe ob im buffer ':' mehr ale 2 mal vorhanden ist wenn ja dann ist es ein gültiger befehl
            if cleaned_data[0:35].count(58) > 2:
                print("114 Fahrbefehl cleaned_data[0:35]:", ''.join(chr(c) for c in cleaned_data[0:35] if c != 0))
                #prüfe ob 9. zeichen ein F ist wenn ja setze 9. bis 21. zeichen zu string zusammen
                if cleaned_data[0] == 70: #F
                    print("107 F Befehl data_rs232pc_in[0:35]:", ''.join(chr(c) for c in cleaned_data[0:35] if c != 0))
                    #setze 9. bis 21. zeichen zu string zusammen
                    command = ''.join(chr(c) for c in cleaned_data[0:35] if c != 0)
                    print("109 Befehl data_rs232pc_in[0:35]:", command)
                    #splitte string in liste
                    command_list = command.split(':')
                    # #ordne command_list[2] zu AchseNr -KD 150, KS 151, CD 152, CS 153, HUB 154
                    if command_list[1] == 'KD':
                        AchseNr = 0
                    elif command_list[1] == 'KS':
                        AchseNr = 1
                    elif command_list[1] == 'CD':
                        AchseNr = 2
                    elif command_list[1] == 'CS':
                        AchseNr = 3
                    elif command_list[1] == 'HUB':
                        AchseNr = 4
                    #Roboter frei fahren von Achse command_list[2] mit Geschwindigkeit command_list[3] für Dauer command_list[4]
                    Rob_freiFahren(AchseNr, int(command_list[2]), int(command_list[3]))
            
            #prüfe ob in data command 'L1' oder 'L0' vorhanden wenn ja schalte schachtlicht ein oder aus
            if cleaned_data[0:2] == [76,49]: #L1
                Rob_SchachtLicht(1)
            elif cleaned_data[0:2] == [76,48]: #L0
                Rob_SchachtLicht(0)

            # prüfe ob Befehl Blinklich am Roboter ein oder ausgeschaltet werden soll
            if cleaned_data[0:2] == [66,49]: #B1
                Rob_Blinklicht(1)
                print('Blinklicht ein')  
            elif cleaned_data[0:2] == [66,48]: #B0
                Rob_Blinklicht(0)
                print('Blinklicht aus')
            
            # Referenz Befehl R alle Achsen
            if cleaned_data[0:2] == [82,65]: #RA
                print('Referenzfahren')  
                Rob_Referenz(modbus,'A') #alle achsen referenzieren
            
           # Roboter Parken
            if cleaned_data[0] == 80: #P

                print('Roboter Parken')  
                Rob_Parken(modbus) #Roboter parken

             
           # Roboter Freigabe
            if cleaned_data[0] == 71: #G
                print('Roboter Freigabe')  
                Rob_Freigabe() #Roboter freigeben

             
           # Roboter Halt
            if cleaned_data[0] == 72: #H
                print('Roboter Halt')  
                Rob_Halt() #Roboter halt

              
            # Zurückschreiben an rs232 des ehaltenen Befehls + 'OK'
               # Konvertieren Sie cleaned_data[9:21] in ein bytearray, bevor Sie es verketten
            # Filtern Sie nur ASCII-Zeichen (32-126) und fügen Sie 'OK' und '\r\n' hinzu
            filtered_command = bytearray(c for c in cleaned_data[0:35] if 32 <= c <= 126)
            command = filtered_command + bytearray("OK\r\n", 'ascii')
            rs232.write_to_serial(command)

            # Rücksetzen für den nächsten Befehl
            new_command = False
            offset = 0
            buffer = bytearray(12)

        # Kurze Pause, um CPU-Last zu reduzieren
        sleep(0.005)  # Entspricht NutSleep(5) im C-Code

def send_random_values(stop_event):
    global modbus
    value=40
    address = 150
    #Füllen der Register 150 bis 155 mit 512
    values_to_write = [512 for _ in range(6)]
    values_to_write.append(1)  # Letztes Register auf 1 setzen - Joystick init ok
    success = modbus.write_multiple_registers(slave=1, address=address, values=values_to_write)
    #print(f"Write successful: {success}")
    print(f"Written values: {values_to_write}")
    value=1
    while not stop_event.is_set():
        #erhöhre einen zähler um 1 und sende diesen an die SPS register 171 per modbus  
        address=171
        value = (value + 1) if (value + 1) <= 65535 else 1  # Erhöhe den Wert und setze ihn auf 40, wenn er 65535 erreicht
        success = modbus.write_multiple_registers(slave=1, address=address, values=[value])
        #print(f"Write successful: {success}")
        if success:
            GPIO.output
        sleep(0.005)  # 5 ms warten
        #read modbus data register 171
        result = modbus.read_holding_registers(slave=1, address=171, count=1)
        if result:
            GPIO.output(cfg.PIN_ROBOTERKOMMUNIKATION, GPIO.LOW)
            sleep(0.005)  # 5 ms warten


# Methode zum expliziten Schließen der seriellen Schnittstelle
def close_serial_port(port):
    try:
        client = ModbusSerialClient(port=port)
        if client.connect():
            client.close()
            print(f"180 Serial port {port} closed")
    except Exception as e:
        print(f"Error closing serial port {port}: {e}")
        

def Rob_SchachtLicht(ein_aus):
    global modbus
    #befehlsspeicher in sps zurücksetzen adresse 159 wert 0
     #write to holding register 159 160 'L' '1' - Schachtlicht Ein 1 Aus 0
    address=159 
    if ein_aus == 1:
        values = [76,49] #L1
    else:
        values = [76,48] #L0
    success = modbus.write_multiple_registers(slave=1, address=address, values=values)
    print(f"115 Write 'L 1/0' successful: {success}")  
    sleep(.002)
    #befehlsspeicher in sps zurücksetzen adresse 159 wert 0
    address=159
    value = 0   
    success = modbus.write_multiple_registers(slave=1, address=address, values=[value])
    sleep(.002)
    
def Rob_Blinklicht(ein_aus):
    global modbus
    #write to holding register 159 160 'B' '1' Schachtlicht Ein 1 Aus 0
    address=159 
    #prüfe ob ein_aus 1 oder 0 ist
    if ein_aus == 1:
        values = [66,49] #B1
    else:
        values = [66,48] #B0
    success = modbus.write_multiple_registers(slave=1, address=address, values=values)
    sleep(.002)
    print(f"207 Write 159 B0/1 successful: {success}")      
    sleep(.002)
    #befehlsspeicher in sps zurücksetzen adresse 159 wert 0
    address=159
    value = 0   #0 = clear
    success = modbus.write_multiple_registers(slave=1, address=address, values=[value])
    sleep(.002)
    
#Referenzfahrt - zuerst HUB, dann KS, dann KD dann CD dann CS
#Referenzfahrt
def Rob_Referenz(modbus,Achse):    
    global  previous_register_206, previous_register_207, unchanged_counter
    
    #befehlsspeicher in sps zurücksetzen adresse 159 wert 0
    address=159
    value = [0,0,0,0,0,0]
    success = modbus.write_multiple_registers(slave=1, address=address, values=value)
    sleep(.002)

#             ' 0 ... keine Referenz, 1 ... Referenz OK, 2 ... Referenz verloren
#             ref_ok_kd = 20 ' speicherplatz fur ref_ok_cs
#             ref_ok_cd = 21 ' speicherplatz fur ref_ok_cd
#             ref_ok_cs = 22 ' speicherplatz fur ref_ok_ks
#             ref_ok_hub = 23 ' speicherplatz fur ref_ok_kd
#             ref_ok_ks = 24 ' speicherplatz fur ref_ok_hub
#             ref_ok = 25' speicherplatz fur ref_ok, alles achsen referenziert

    address=20
    value = [0,0,0,0,0,0]
    success = modbus.write_multiple_registers(slave=1, address=address, values=value)
    result = modbus.read_holding_registers(slave=1, address=address, count=6)
    print('301 ref_ok_achsen 20-25: ', result)
    sleep
    #write to holding register 158 160 'Ref' Roboter Refernzieren
    address=50 
    if str(Achse) == "A":
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
    print(f"276 Write 'VR25 0' successful: {success}")

    #warten bis referenzierung abgeschlossen wenn nach 20sec noch nicht abgeschlossen dann abbruch
    start_time = time()
    timeout = 40  # Sekunden
    bmodbusread = True
    sleep(.003)
    while True:
        address=25
        register25 = modbus.read_holding_registers(slave=1, address=address, count=1)
       #print(f"229 Registers 25: {register25}")
        address=23
        register23 = modbus.read_holding_registers(slave=1, address=address, count=1)
        #print(f"229 Registers 23: {register23}")
        #lese register 206,207 und prüfe ob sich der wert geändert hat
    # Lese Register 206 und 207
        address = 206
        registers = modbus.read_holding_registers(slave=1, address=address, count=2)
        #print(f"Registers 206-207: {registers}")

        # Werte der Register 206 und 207 extrahieren
        current_register_206 = registers[0]
        current_register_207 = registers[1]

        # Prüfen, ob sich die Werte geändert haben
        if previous_register_206 is not None and previous_register_207 is not None:
            if current_register_206 == previous_register_206 and current_register_207 == previous_register_207:
                unchanged_counter += 1
                #print(f"Die Werte haben sich nicht geändert. Zähler: {unchanged_counter}")
            else:
                unchanged_counter = 0  # Zähler zurücksetzen, wenn sich die Werte geändert haben

        # Aktuelle Werte als vorherige Werte speichern
        previous_register_206 = current_register_206
        previous_register_207 = current_register_207

        # Breche die Schleife ab, wenn der Zähler 50 erreicht
        if unchanged_counter >= 8 and register23[0] == 0:
            print("Die Werte haben sich 8 Mal nicht geändert. Schleife wird abgebrochen.")
            unchanged_counter=0
            break
        if register25[0] == 63:
            break
        if time() - start_time > timeout:
            print("Abbruch: Referenzierung nicht abgeschlossen nach 35 Sekunden.")
            break
        sleep(.25)

    address=20
    registers = modbus.read_holding_registers(slave=1, address=address, count=6)
    print(f"236 Registers 20-25: {registers}")
      #move über joystick aktivieren
    address=3 
    values = 49#'1=True möglich
    success = modbus.write_multiple_registers(slave=1, address=address, values=values)
    print(f"299 Write 'VR3 1' successful: {success}")
    #print status kommunikation
    address=5
    registers = modbus.read_holding_registers(slave=1, address=address, count=1)
    print(f"303 Kom_ok Status 5: {registers[0]}")
    #print status joystick
    address=156
    registers = modbus.read_holding_registers(slave=1, address=address, count=1)
    print(f"307 jOystick Status 156 ok wenn 49: {registers[0]}")
  #kommunikations id
    address=171
    registers = modbus.read_holding_registers(slave=1, address=address, count=1)
    sleep(.002)
    print(f"312 Registers 171: {registers[0]}")
    #aktueller speed hub
    address=103
    registers = modbus.read_holding_registers(slave=1, address=address, count=1)
    bmodbusread = False
    sleep(.002)
    print(f"317 Speed Hub 103: {registers}")
    #befehlsspeicher in sps zurücksetzen adresse 159 wert 0
    address=159
    value = [0,0,0,0,0,0]
    success = modbus.write_multiple_registers(slave=1, address=address, values=value)

    print("Referenzfahrt abgeschlossen")
    
    
#Roboter Parken
def Rob_Parken(modbus):
     #write to holding register 159 160 'Antwort ok"
    address=2 
    values= [49] #1
    success = modbus.write_multiple_registers(slave=1, address=address, values=values)
    address=25 
    value = [48]#'noch nicht refereziert
    success = modbus.write_multiple_registers(slave=1, address=address, values=value)
    print(f"208 Write 'VR25 0' successful: {success}")
  #write to holding register 158 160 'Ref not ok' Roboter Refernzieren
    address=50 
    values = [80]#'PA
    success = modbus.write_multiple_registers(slave=1, address=address, values=values)
    print(f"242 Write 'VR50 80 65' successful: {success}")

    #write to holding register 159  
    addresss=159 
    values= [80,65] #'P
    success = modbus.write_multiple_registers(slave=1, address=address, values=values)

     
    #bus park register 158 160 'P' 'A' Roboter Parken
    address=158
    registers = modbus.read_holding_registers(slave=1, address=address, count=1)
    sleep(.002)
    print(f"350 Registers 158: {registers}")

    sleep(10)
    print('445 Roboter Parken ENDE')

#Roboter freigeben
def Rob_Freigabe():
    global modbus
    #befehlsspeicher in sps zurücksetzen adresse 159 wert 0
    address=159
    value = 0   
    success = modbus.write_multiple_registers(slave=1, address=address, values=[value])

    #write to holding register 159 160 'G' Roboter Freigabe
    address=159 
    value = 71 #'G
    success = modbus.write_multiple_registers(slave=1, address=address, values=[value])
    print(f"146 Write 'G' successful: {success}")
    
#Roboter Halt
def Rob_Halt():
    global modbus
    #befehlsspeicher in sps zurücksetzen adresse 159 wert 0
    address=159
    value = 0   
    success = modbus.write_multiple_registers(slave=1, address=address, values=[value])

    #write to holding register 159 160 'G' Roboter Freigabe
    address=159 
    value = 72 #'H
    success = modbus.write_multiple_registers(slave=1, address=address, values=[value])
    print(f"146 Write 'H' successful: {success}")  


#Roboter Achsen frei fahren nach Zeit commando: F:KD:Geschwindigkeit:Dauer
def Rob_freiFahren(AchsNr, Geschwindigkeit,DauerSekunden): 
    global modbus 
   #fahre CD
    print("331 Start fahren von Achse", AchsNr, "mit Geschwindigkeit", Geschwindigkeit, "und Dauer", DauerSekunden) 
    #schreibe in den Befehlsspeicher 154 den wert 305 im takt von 150ms für die dauer von 3sekunden
    endtime=time()+DauerSekunden
    while time() < endtime:
        # Schreiben des v Hub wertes in das Register
        address=150
        values = [512,512,512,512,512,512,1] # achsen1-5 6=joystick init ok 
        #wert in values an listenstelle AchsNr auf Geschwindigkeit setzen
        values[AchsNr] = Geschwindigkeit
        success = modbus.write_multiple_registers(slave=1, address=address, values=values)
        # if success:
        #     print(f"294 Write successful: {success}")
        # else:
        #     print(f"296 Write failed: {success}")
        # sleep(0.003)

    sleep(0.001)
    address=150
    values = [512,512,512,512,512,512,1] # achsen1-5 6=joystick init ok 
    success = modbus.write_multiple_registers(slave=1, address=address, values=values)
    print("Roboter fahren beendet")

def Rob_Version(modbus):
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
    print(f"498 Registers 211-214: {registers}")
    sleep(.002)
    #print(f"260 Registers 211: {registers}")
    # Wandle Antwortspeicher in String um und extrahiere nur die ersten drei Zeichen
    response = ''.join([chr(x) for x in registers[:8]])
    print(f"\n481 Antwort von Roboter auf Befehl V: {response}")
    print("Start Hauptprogramm")
    #befehlsspeicher in sps zurücksetzen adresse 159 wert 0
    address=159
    value = 0   
    success = modbus.write_multiple_registers(slave=1, address=address, values=[value])


# hauptprogramm

modbus = None
try:

    #zuerst pc schnittstelle und modbus schnittstelle schließen und öffnen

    close_serial_port('/dev/ttyACM0')  # Schließen der seriellen Schnittstelle, falls sie noch geöffnet ist
    close_serial_port('/dev/ttyUSB0')  # Schließen der seriellen Schnittstelle, falls sie noch geöffnet ist

    modbus = Modbus(port='/dev/ttyACM0', baudrate=38400, bytesize=8, parity='E', stopbits=1)
    # check if modbus is open
    print("530 Modbus opened", modbus)

    # Event zum Stoppen des Threads
    stop_event = threading.Event()
    # Starten des Threads
    thread_sendrandom = threading.Thread(target=send_random_values, args=(stop_event,)
                                         )
    thread_sendrandom.daemon = True
    thread_sendrandom.start()
    sleep(.1)
    
    # RS232-Verbindung zum PC aufbauen
    rs232 = RS232(port='/dev/ttyUSB0')  #  oder ein anderer verfügbarer Port

    # Starten des RS232-Threads
    thread = threading.Thread(target=rs232_thread, args=(rs232,))
    thread.start()
    
    #prüfen ob roboter erreichbar
    Rob_Version(modbus)
    sleep(.002)

    #wenn alles ok - mqtt client starten
    # Verbindung zum MQTT-Broker herstellen
    print("Verbindung zum Broker herstellen", broker)
    client.connect(broker)

    client.on_connect = on_connect
    client.on_message = on_message
    #starte mqtt listener für joystick
    subscribe_mqtt_topics()
    client.loop_start()
    print("Verbindung zum Broker hergestellt")

    while True:
            # Überprüfen Sie den Modbus-Statu
            read_modbus_data(modbus)
            sleep(.1)  # Haupt-Thread läuft weiter
except KeyboardInterrupt:
    rs232.close()
    print("Stopping threads...")
    thread.join()
    #stop and kill thread thread_sendrandom
    stop_event.set()
    thread_sendrandom.join()
    print("Thread stopped")
    modbus.close()
    print("Modbus closed")
    print("Script exit")

finally:
    if modbus:
        print("571 Closing Modbus")
        modbus.close()
    #exit script
    print("574 Script exit")
    exit()
    
