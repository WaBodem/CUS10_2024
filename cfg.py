BROKER='192.168.10.10'
MQTT_ID='CUS10'
MQTT_TOPIC='/JOYSTICK'
HUB_TOPIC='/HUB'
CD_TOPIC='/CD'
CS_TOPIC='/CS'
KS_TOPIC='/KS'
KD_TOPIC='/KD'
LICHT_TOPIC='/LICHT'
OS_TOPIC='/OS'
INIT_TOPIC='/INIT'
JOYSTICKINITRAW_TOPIC='/RAW'
JOYSTICKVAL_TOPIC='/VAL'
JOYSTICKMODBUSVR150_TOPIC='/VR150'
MQTT_JoystickRawTimer= 5 #sec
MQTT_JoystickTimer=.1 #sec

ADC_REF=5.0 #Referenzspannung, anpassen falls nötig
joystickoffset_zul=25

# Mapping von Namen zu Kanalnummern
channel_mapping = {
    'CS': 0,
    'HUB': 1,
    'CD': 2,
    'LICHT': 3,
    'KS': 4,
    'KD': 5
}

PIN_JOYSTICKINIT=5
