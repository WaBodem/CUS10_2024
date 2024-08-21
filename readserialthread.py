import time
import threading
import serial

# Beispielhafte Klasse, um die RS232-Kommunikation zu simulieren
class RS232:
    def __init__(self, port, baudrate=9600, timeout=1):
        self.ser = serial.Serial(port, baudrate, timeout=timeout)
        self.data_in = [0] * 30  # Beispielpuffer für "data_in"

    def read_from_serial(self, length):
        return self.ser.read(length)

    def write_to_serial(self, data):
        self.ser.write(data)

    def close(self):
        self.ser.close()

# RS232-Thread-Funktion
def rs232_thread(rs232):
    buffer = bytearray(12)  # Puffer für eingehende Daten (max 12 Zeichen)
    offset = 0
    new_command = False

    while True:
        # Daten von RS232 lesen
        got = rs232.read_from_serial(12 - offset)
        if got:
            buffer[offset:offset + len(got)] = got
            offset += len(got)

            # Prüfen, ob das letzte Zeichen ein CR oder LF ist
            if buffer[offset - 1] in [13, 10]:  # CR oder LF
                new_command = True

        # Verarbeiten des empfangenen Befehls
        if new_command:
            # Schreiben des Befehls in den "data_in"-Puffer
            rs232.data_in[9:21] = buffer[:offset]

            # Debug: Befehl anzeigen
            print("Befehl:", ''.join(chr(c) for c in rs232.data_in[9:21] if c != 0))

            # Rücksetzen für den nächsten Befehl
            new_command = False
            offset = 0
            buffer = bytearray(12)

        # Kurze Pause, um CPU-Last zu reduzieren
        time.sleep(0.005)  # Entspricht NutSleep(5) im C-Code

# Beispielhafte Verwendung des RS232-Threads
if __name__ == "__main__":
    # RS232-Verbindung simulieren
    rs232 = RS232(port='COM1')  # COM1 oder ein anderer verfügbarer Port

    # Starten des RS232-Threads
    thread = threading.Thread(target=rs232_thread, args=(rs232,))
    thread.start()

    # Hier könnte zusätzlicher Code laufen

    # Schließen der RS232-Verbindung (beim Programmende)
    try:
        while True:
            time.sleep(1)  # Haupt-Thread läuft weiter
    except KeyboardInterrupt:
        rs232.close()
        thread.join()
