''' VLS CUS10 WB Juli24
RP2040
'''
#from DfrobotGP8403 import *
import utime
from sys import exit

from machine import Pin, PWM
from time import sleep
from time import ticks_diff, ticks_ms, sleep_ms

debounce_time = 50  # milliseconds

VLS_Status_tick = 500 #msec
VLS_Status_tick_start = ticks_ms()
live_cnter=0


''' Frequenzumrichter
- Hz Angabe ist hoch und raus fahren
Q12 horizontal + Freq hineinfahren -hinaus
Q13 vertikal -Freq=hochfahren +Freq ab

- Freq wird direkt am Lenze MContr Eingestellt - Display Taste < dann auf ab  taste 

'''

# Pin definitions
PWM_Pin_Lenze=1
ES_Hinten = 15
ES_Vorne = 14
ES_Oben = 13
ES_Unten = 12
pbEinladen = 11
pbAusladen = 10
OP_Lenze_Freigabe = 9
OP_Lenze_Richtung = 8
OP_RelaisVertikale = 6
OP_RelaisHorizontale = 7
OP_VLS_move_Led=0
# Pin setup
pins = {
    "ES_Hinten": Pin(ES_Hinten, Pin.IN, Pin.PULL_DOWN),
    "ES_Vorne": Pin(ES_Vorne, Pin.IN, Pin.PULL_DOWN),
    "ES_Oben": Pin(ES_Oben, Pin.IN, Pin.PULL_DOWN),
    "ES_Unten": Pin(ES_Unten, Pin.IN, Pin.PULL_DOWN),
    "pbEinladen": Pin(pbEinladen, Pin.IN, Pin.PULL_DOWN),
    "pbAusladen": Pin(pbAusladen, Pin.IN, Pin.PULL_DOWN),
    "OP_Lenze_Freigabe": Pin(OP_Lenze_Freigabe, Pin.OUT),
    "OP_Lenze_Richtung": Pin(OP_Lenze_Richtung, Pin.OUT),
    "OP_RelaisVertikale": Pin(OP_RelaisVertikale, Pin.OUT),
    "OP_RelaisHorizontale": Pin(OP_RelaisHorizontale, Pin.OUT),
    "OP_VLS_move_Led": Pin(OP_VLS_move_Led, Pin.OUT),
    "VLS_alive_LED": Pin('LED', Pin.OUT) #ACHTUNG neu weder Pin0 noch Pin25
}
#Lenze


def fdbg(line):
    print(line, "debug")
# Constants******************************7*88
MAX_SPEED_VERLADESTATION_VER = 25000
MAX_SPEED_VERLADESTATION_HOR = 25000
RICHTUNG_HOR_EINFAHREN = 1
RICHTUNG_HOR_AUSFAHREN =0
RICHTUNG_VER_AUF = 0
RICHTUNG_VER_AB = 1

def init_verladestation():
    # Set initial states for outputs
    pins["OP_Lenze_Freigabe"].value(1)
    pins["OP_Lenze_Richtung"].value(0)
    pins["OP_RelaisVertikale"].value(1)
    pins["OP_RelaisHorizontale"].value(1)
# pwm setting 0
#     pwm.duty_u16(0)
    #PWM 0-10V set to 0
#     DAC.set_dac_out_voltage(0,0)
    print("Verladestation initialized")
    for i in range(5):
        pins["OP_VLS_move_Led"].value(1)
        sleep(.075)
        pins["OP_VLS_move_Led"].value(0)
        sleep(.075)

def move_lenze(richtung, speed):
    global  pins, bmove
    # Simulate setting speed with PWM (as an example, replace with actual speed setting)
#     pwm.freq(1000)
#     pwm.duty_u16(speed)
    # Output value from DAC channel 0
    # Value in mV = 0-5000 or 0-10000 depending on range
#     DAC.set_dac_out_voltage(speed,0)
    pins["OP_Lenze_Richtung"].value(richtung)
    pins["OP_Lenze_Freigabe"].value(0)
    print(f"Moving Lenze in direction {richtung} with speed {speed}")
    pins["OP_VLS_move_Led"].value(1)
    bmove=True

def stop_lenze():
    global bmove,pins
#    pwm.duty_u16(0)
#     DAC.set_dac_out_voltage(0,0)
    sleep(.1)  # Simulate ramp down
    bmove=False
    pins["OP_Lenze_Freigabe"].value(1)
    pins["OP_RelaisVertikale"].value(1)
    sleep(.05)
    pins["OP_RelaisHorizontale"].value(1)
    print("115 Lenze stopped")
    pins["OP_VLS_move_Led"].value(0)
    

def roboter_einladen():
    global bmove
    if pins["pbEinladen"].value() == 0 and pins["pbAusladen"].value() == 0 and pins["OP_Lenze_Freigabe"].value()==1:
        print("VLS: no action")
        pins["OP_VLS_move_Led"].value(0)
        bmove=False
        return -1
    #vls vorne  - dann hochfahren
    if pins["ES_Vorne"].value() == 0  and pins["ES_Oben"].value() == 1 and pins["pbEinladen"].value() == 1 and pins["pbAusladen"].value() == 0:
        pins["OP_RelaisVertikale"].value(0)
        move_lenze(RICHTUNG_VER_AUF, MAX_SPEED_VERLADESTATION_VER)
        print("110 hochfahren")
        
        while pins["ES_Oben"].value() == 1 and pins["pbEinladen"].value() == 1:
            bmove=True
        stop_lenze()
    #wenn VLS irgendwo dazwischen
    if pins["ES_Unten"].value() == 1 and pins["ES_Oben"].value() == 1 and pins["ES_Hinten"].value() == 1 and pins["pbEinladen"].value() == 1  and pins["pbAusladen"].value() == 0:
        pins["OP_RelaisHorizontale"].value(0)
        move_lenze(RICHTUNG_VER_AUF, MAX_SPEED_VERLADESTATION_HOR)
        while pins["ES_Hinten"].value() == 1 and pins["pbEinladen"].value() == 1:
            bmove=True
        stop_lenze()

    #wenn vls oben aber noch nicht hinten
    if pins["ES_Oben"].value() == 0 and pins["ES_Hinten"].value() == 1 and pins["pbEinladen"].value() == 1  and pins["pbAusladen"].value() == 0:
        pins["OP_RelaisHorizontale"].value(0)
        move_lenze(RICHTUNG_HOR_EINFAHREN, MAX_SPEED_VERLADESTATION_HOR)
        while pins["ES_Hinten"].value() == 1 and pins["pbEinladen"].value() == 1:
            bmove=True
        stop_lenze()
    #wenn vls hinten oben dann nach unten fahren ca. .5 sec
    if pins["ES_Hinten"].value() == 0 and pins["ES_Oben"].value() == 0 and pins["pbEinladen"].value() == 1 and pins["pbAusladen"].value() == 0:
        pins["OP_RelaisVertikale"].value(0)
        move_lenze(RICHTUNG_VER_AB, MAX_SPEED_VERLADESTATION_VER)
        sleep(0.5)
        stop_lenze()
        print("138 Stop moving")
    return 0

def roboter_ausladen():
    global bmove
    if pins["pbEinladen"].value() == 0 and pins["pbAusladen"].value() == 0 and pins["OP_Lenze_Freigabe"].value()==1:
        print("148 VLS: no action")
        pins["OP_VLS_move_Led"].value(0)
        return -1
    #wenn vls irgendwo
    if pins["ES_Hinten"].value() == 1 and pins["ES_Oben"].value() == 1 and pins["ES_Vorne"].value() == 1 and pins["pbAusladen"].value() == 1  and pins["pbEinladen"].value() == 0:
        pins["OP_RelaisVertikale"].value(0)
        move_lenze(RICHTUNG_VER_AUF, MAX_SPEED_VERLADESTATION_VER)
        
        while pins["ES_Oben"].value() == 1 and pins["pbAusladen"].value() == 1:
            bmove=True
            print('1.','')
        stop_lenze()
    #wenn vls hinten aber noch nicht oben
    if pins["ES_Hinten"].value() == 0 and pins["ES_Oben"].value() == 1 and pins["pbAusladen"].value() == 1  and pins["pbEinladen"].value() == 0:
        pins["OP_RelaisVertikale"].value(0)
        move_lenze(RICHTUNG_VER_AUF, MAX_SPEED_VERLADESTATION_VER)
        
        while pins["ES_Oben"].value() == 1 and pins["pbAusladen"].value() == 1:
            bmove=True
            print('2.','')
        stop_lenze()
    #wenn vls oben aber nicht vorne
    if pins["ES_Oben"].value() == 0 and pins["ES_Vorne"].value() == 1 and pins["pbAusladen"].value() == 1  and pins["pbEinladen"].value() == 0:
        pins["OP_RelaisHorizontale"].value(0)
        move_lenze(RICHTUNG_HOR_AUSFAHREN, MAX_SPEED_VERLADESTATION_HOR)
        
        while pins["ES_Vorne"].value() == 1 and pins["pbAusladen"].value() == 1:
            bmove=True
            print('3.','')
        stop_lenze()
    #wenn vls vorne oben
    if pins["ES_Vorne"].value() == 0 and pins["ES_Unten"].value() == 1 and pins["pbAusladen"].value() == 1  and pins["pbEinladen"].value() == 0:
        pins["OP_RelaisVertikale"].value(0)
        move_lenze(RICHTUNG_VER_AB, MAX_SPEED_VERLADESTATION_VER)
        
        while pins["ES_Unten"].value() == 1 and pins["pbAusladen"].value() == 1:
            bmove=True
            print('4.','')        
        stop_lenze()
    return 0

def fES_Status(live_cnter):
        if pins["ES_Hinten"].value()== 1:
            statusES_Hinten="1 nicht geschaltet"
        else:
            statusES_Hinten="0 nicht geschaltet"
        if pins["ES_Vorne"].value()== 1:
            statusES_Vorne="1 nicht geschaltet"
        else:
            statusES_Vorne="0 geschaltet"
        if pins["ES_Oben"].value()== 1:
            statusES_Oben="1 nicht geschaltet"
        else:
            statusES_Oben="0 geschaltet"
        if pins["ES_Unten"].value()== 1:
            statusES_Unten="1 nicht geschaltet"
        else:
            statusES_Unten="0  geschaltet"
            
        print(live_cnter,"VLS Status:")
        print("\tES_Hinten = ", statusES_Hinten)
        print("\tES_Vorne = ", statusES_Vorne)
        print("\tES_Oben = ", statusES_Oben)
        print("\tES_Unten = ", statusES_Unten)
        VLS_Status_tick_start = ticks_ms()


def fPB_Status(live_cnter):
        if pins["pbEinladen"].value()== 1:
            statuspbEinladen="1  geschaltet"
        else:
            statuspbEinladen="0 nicht geschaltet"
        if pins["pbAusladen"].value()== 1:
            statuspbAusladen="1  geschaltet"
        else:
            statuspbAusladen="0 nicht geschaltet"
        
        print(live_cnter,"VLS Status PB Button:")
        print("\tPB Einladen = ", statuspbEinladen)
        print("\tPB Ausladen = ", statuspbAusladen)

def debounce(pin):
    """Simple debounce function that checks the pin state remains stable."""
    initial_state = pin.value()
    sleep_ms(debounce_time)
    return initial_state == pin.value()

# Initialize the system
init_verladestation()

sleep(1)
print("VLS Bereit")

#START TESTS

# print("PB Test")
# i = 1
# while True:
#     fPB_Status(i)
#     i +=1
#     sleep(1)

# print("test lenze richtung")
# pins["OP_Lenze_Freigabe"].value(0)
# sleep(2)
# for i in range(10):
#     pins["OP_Lenze_Richtung"].value(0)
#     print("Relais 3 schalten")
#     sleep(5)
#     pins["OP_Lenze_Richtung"].value(1)
#     print("Relais3 ausschalten")
#     sleep(5)
# pins["OP_Lenze_Freigabe"].value(1)
# print("Ende test lenze richtung")


# print("test q12 q13")
# print("IO TEst")
# for i in range(10):
#     pins["OP_RelaisVertikale"].high()
#     pins["OP_RelaisHorizontale"].high()
#     pins["OP_RelaisVertikale"].value(1)
#     pins["OP_RelaisHorizontale"].value(1)
#     sleep(2)
#     print("going low")
#     pins["OP_RelaisVertikale"].value(0)
#     pins["OP_RelaisHorizontale"].value(0)
#     pins["OP_RelaisVertikale"].low()
#     pins["OP_RelaisHorizontale"].low()
#     sleep(1)
#     
# print("IO TEst Ende") 
#
# maunell fahren
# while True:
#     richtung= input("pwm test fahre in richtung 0/1 (0 Auf/Vor 1 Ab/Zurück-1 eXIT): ")
#     if int(richtung) == -1:
#         exit()
#     else:
#         motorwahl= input("Motor Relais Q12, Q13 wählen 1 = Horizontal / 2 = Vertikal: ")
#         zeitinsec= input("Fahrzeit in sec (-1 EXIT): ")
#         if int(motorwahl)==2:
#             
#             pins["OP_RelaisVertikale"].value(0)
#             pins["OP_RelaisHorizontale"].value(1)
#         elif int(motorwahl)==1:
#             pins["OP_RelaisHorizontale"].value(0)
#             pins["OP_RelaisVertikale"].value(1)
#         else:
#             pins["OP_RelaisVertikale"].value(0)
#             pins["OP_RelaisHorizontale"].value(0)
#         move_lenze(int(richtung), 20000)
#         sleep(int(zeitinsec))
#         stop_lenze()

# print("PWM Test") 
# pins["OP_RelaisHorizontale"].value(0)
# for i in range(10,60,10):
#             print(i*100)
#             move_lenze(1, 100*i)
#             sleep(10)
#             stop_lenze()
#             sleep(5)


# for i in range(50):
#     fES_Status()
#     sleep(.5)


# for i in range(50):
#     fPB_Status()
#     sleep(.5)

#ENDE TESTS

# Main loop
while True:
    if debounce(pins["pbEinladen"]):
        if pins["pbEinladen"].value() == 1:
            print("keyEinladen")
            pins["OP_VLS_move_Led"].value(1)
            roboter_einladen()

    if debounce(pins["pbAusladen"]):
        if pins["pbAusladen"].value() == 1:
            print("keyAusladen")
            pins["OP_VLS_move_Led"].value(1)
            roboter_ausladen()

    if pins["pbAusladen"].value() == 0 and pins["pbEinladen"].value() == 0:
        pins["OP_VLS_move_Led"].value(0)


    if ticks_diff(ticks_ms(), VLS_Status_tick_start) > VLS_Status_tick:
        live_cnter +=1
        fES_Status(live_cnter)
        fPB_Status(live_cnter)
        VLS_Status_tick_start = ticks_ms()
        pins["VLS_alive_LED"].toggle()
