BROKER='192.168.10.10'
MQTT_ID='CUS10'
JS_TOPIC='/JOYSTICK'
MODBUS_TOPIC='/MODBUS'
DIST_TOPIC='/R/S/D/distanz'
LASERCMD_TOPIC='/C/S/D'
INKLINX_TOPIC='/R/S/I/X'
HUB_TOPIC='/HUB'
CD_TOPIC='/CD'
CS_TOPIC='/CS'
KS_TOPIC='/KS'
KD_TOPIC='/KD'
LICHT_TOPIC='/LICHT'
KAMERALICHT_TOPIC='/C/A/KAMERALICHT'
OS_TOPIC='/OS'
INIT_TOPIC='/INIT'
JOYSTICKINITRAW_TOPIC='/RAW'
JOYSTICKVAL_TOPIC='/VAL'
JOYSTICKMODBUSVR150_TOPIC='/VR150'
MQTT_JoystickRawTimer= 5 #sec
MQTT_JoystickTimer=.1 #sec
Lichttimer=.1
LICHTPWMSTEPS=5
LICHTSCHWELLENWERTJOYST_OBEN=650
LICHTSCHWELLENWERTJOYST_UNTEN=350

ADC_REF=5.0 #Referenzspannung, anpassen falls nötig
joystickoffset_zul=40

# Mapping von Namen zu Kanalnummern
channel_mapping = {
    'CS': {'channel': 0, 'zulkaliboffset': 35},
    'HUB': {'channel': 1, 'zulkaliboffset': 35},
    'CD': {'channel': 2, 'zulkaliboffset': 35},
    'LICHT': {'channel': 3, 'zulkaliboffset': 300},  # Kein Offset angegeben
    'KS': {'channel': 4, 'zulkaliboffset': 35},
    'KD': {'channel': 5, 'zulkaliboffset': 35}
}

PINO_JOYSTICKINIT=5
PINO_ROBOTERREFERENZIERT=23
PINO_ROBOTERKOMMUNIKATION=15
PINI_REFPARK=21
PINI_ROBOTERANGESTECKT=20
PINO_ROBOTERVERBUNDENLED=8
PINO_REFPARKFAHRTLED=24

CD_OFFSET=17000
CS_GEAR=100
CS_USTEPS=32
INKLINX_CORR=0.62

