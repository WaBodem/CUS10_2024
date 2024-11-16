#!/usr/bin/python3

"""
2021 WB - Compass HMC6343 Abfrage CUS10

sudo apt install i2c-tools
#sudo apt install python3-smbus
sudo apt install pyhton3-pip
#sudo pip3 install smbus2
sudo apt install python3-setuptools
sudo pip3 install quick2wire-api
sudo pip3 install paho-mqtt
sudo apt install python3-dev
sudo pip3 install setproctitle
"""

from hmc6343 import HMC6343

from time import sleep, time
import paho.mqtt.client as mqtt
#import configparser
import sys
import setproctitle
from subprocess import call, check_output

bKompass=False

broker = '192.168.10.10'
port = 1883

setproctitle.setproctitle("CUS10_Kompass")


# def read_ini():
#     global port, broker, startpwm
#     inifile = configparser.ConfigParser()   
#     if len(sys.argv) > 1:
#         print(sys.argv[1])
#         inifile.read(str(sys.argv[1]),encoding='utf-8')
#     else:
#         inifile.read('KK_CUS5.ini', encoding='utf-8')
#     
#     broker = inifile.get('mqtt','broker')
#     port = inifile.get('mqtt','port')
#     port = int(port)     # wird sonst als string eingelesen
#     startpwm = inifile.get('LED', 'startpwm')
#     startpwm = float(startpwm) # wird sonst als string eingelesen
#     print("Daten aus Inifile erfolgreich geladen")    
#     
    
# -------------------------pruefen ob verbindung zu broker-----------------------------------------
def on_connect(client, userdata, flags, rc):
    global bconnect
    if rc==0:
        print("connected OK Returned code=",rc)
        bconnect = True
    else:
        print("Bad connection Returned code=",rc)
        bconnect = False
# -------------------------wenn verbindung zu broker abbricht-----------------------------------------
def on_disconnect(client, userdata, rc):
    print('Kompass disconnected')
    print("disconnecting reason  "  +str(rc))
# -------------------------On-Message Aktionen-----------------------------------------
def on_message(client, userdata, message):
    global bKompass
    print("%s : %s" % (message.topic, message.payload))
    
    mqttmsg = str(message.payload.decode())
    
    mqtttopic = message.topic
    
    if mqtttopic== "CUS10/C/S/K" and mqttmsg.find("?")>=0:
        bKompass=True
        

     
      
# -------------------------MQTT-Initialisierung----------------------------------------- 
def mqtt_init():
    global topic_sub, port, topic_pub, broker, client
    
    topic_sub = 'CUS10/C/S/K'
    client = mqtt.Client("CUS10_KOMPASS")  # create new instance
    client.on_connect = on_connect
    client.connect(broker, port)  # connect to broker
    print('broker', broker)
    client.loop_start()
    sleep(3)
    if bconnect:
        client.on_message = on_message
        client.on_disconnect = on_disconnect
        client.subscribe(topic_sub, qos=0)
        client.loop_start()    # start the loop
        print("MQTT-Initialisierung abgeschlossen")
    elif not bconnect:
        print("MQTT-Initialisierung fehlgeschlagen")
    
   
    

def main():
    global bKompass
    print("CUS10 - Kompassabfrage HMC6343 START!")
    livetick= 10 #sec
    livetimer=time()+livetick
    compass=HMC6343()
    while True :
        sleep(0.05)
        if bKompass:
            sheading= str(compass.readHeading())+'°'
            client.publish("CUS10/R/S/K", sheading)
            print("Kompass: ", sheading, '°')
            bKompass = False
#         if bled_change:
#             p1.ChangeDutyCycle(dc1)
#             bled_change = False
        if time()>livetimer:
            livetimer=time()+livetick
            sheading= str(compass.readHeading())+'°'
            client.publish("CUS10/R/S/K", sheading)
            print("Kompass: ", sheading)
            #CPU Temp
            cputemp=check_output(["cat", "/etc/armbianmonitor/datasources/soctemp"])
            s_cputemp=str(int(int(cputemp.decode())/1000))
            print("CPU Temp.: ", s_cputemp, "°C")
            client.publish("CUS10/R/S/SS", 'LIVECont SK_CPU Temp: '+ s_cputemp + '°C' )

            

    
if __name__ == "__main__":
    print('Programm startet...')
    try:
#        read_ini()
#        gpio()
        mqtt_init()
        main()
    except KeyboardInterrupt:
#        p1.stop()
#        p2.stop()
        #GPIO.cleanup()
        sys.exit()
