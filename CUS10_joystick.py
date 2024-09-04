#!/usr/bin/python
# -*- coding:utf-8 -*-
import paho.mqtt.client as mqtt

import cfg  # Stellen Sie sicher, dass cfg die BROKER-Konfiguration enthält

from time import sleep, time
import ADS1263
import RPi.GPIO as GPIO

bJoystickInit = False
# Verwenden der Namen, um die Kanäle anzusprechen
channel_names = ['CS', 'HUB', 'CD', 'LICHT', 'KS', 'KD']



# #setup gpio20 as input for Roboter_is_connected
GPIO.setmode(GPIO.BCM)
# #setup gpio8 as output for  LED Roboter_is_connected
GPIO.setup(cfg.PIN_JOYSTICKINIT, GPIO.OUT)
GPIO.output(cfg.PIN_JOYSTICKINIT, GPIO.LOW)


# ADC1 test part
TEST_ADC1       = True
# ADC2 test part
TEST_ADC2       = False
# ADC1 rate test part, For faster speeds use the C program
TEST_ADC1_RATE   = False
# RTD test part 
TEST_RTD        = False     


def scale_32bit_to_1023(value):
    max_32bit = 2**32 - 1
    scaled_value = (value / max_32bit) * 1023
    return int(scaled_value)

# value_voltage: Spannung im Bereich 0 bis cfg.ADC_REF
def scale_voltage_to_10bit(value_voltage, ref_voltage):
    max_10bit = 1023  # 2^10 - 1
    return int((value_voltage / ref_voltage) * max_10bit)

# MQTT-Konfiguration
broker = cfg.BROKER
client = mqtt.Client()

def on_connect(client, userdata, flags, rc):
    print("Verbunden mit dem Broker")

#function to read joystick values 20 times for calibration and check if average is between 2.4 and 2.6V for each axis
#axis HUB = adc1 channel 0  axis CD = adc1 channel 1    axis CS = adc1 channel 2    axis KS = adc1 channel 3    axis KD = adc1 channel 4    axis LICHT = adc1 channel 5 
def init_joystick():
    global bJoystickInit, channel_names

    ADC = ADS1263.ADS1263()
    if (ADC.ADS1263_init_ADC1('ADS1263_50SPS') == -1):
        exit()
    ADC.ADS1263_SetMode(0)  # 0 is singleChannel, 1 is diffChannel

    channelList = [cfg.channel_mapping[name] for name in channel_names]  
 
    # Initialisieren der Summen für die Durchschnittsberechnung
    sums = {channel: 0 for channel in channelList}

    for _ in range(20):
        ADC_Value = ADC.ADS1263_GetAll(channelList)  # get ADC1 value
        for i, channel in enumerate(channelList):
            if ADC_Value[i] >> 31 == 1:
                value = cfg.ADC_REF * 2 - ADC_Value[i] * cfg.ADC_REF / 0x80000000
                print(f"ADC1 IN {channel_names[channel]} = -{value}")
            else:
                value = ADC_Value[i] * cfg.ADC_REF / 0x7fffffff  # 32bit
                print(f"ADC1 IN {channel_names[channel]} = {value} 32bit val: {ADC_Value[i]}")
            sums[channel] += value
        for channel in channelList:
            print("\33[2A")

    # Berechnung der Durchschnittswerte
    averages = {channel: sums[channel] / 20 for channel in channelList}

   
    # Überprüfung der Durchschnittswerte
    bJoystickInit = True
    client.publish(cfg.MQTT_ID + cfg.MQTT_TOPIC + cfg.INIT_TOPIC + cfg.JOYSTICKINITRAW_TOPIC, "zu Offset von 512: "+str(cfg.joystickoffset_zul))
    for channel, avg in averages.items():
        client.publish(cfg.MQTT_ID + cfg.MQTT_TOPIC + cfg.INIT_TOPIC + cfg.JOYSTICKINITRAW_TOPIC +'/CH'+str(channel_names[channel]), "v: " + str(round(avg,4))+ ' 10bitval: '+ str(scale_voltage_to_10bit(avg, cfg.ADC_REF)) +" Offset: " + str(scale_voltage_to_10bit(avg, cfg.ADC_REF)-512))
        if scale_voltage_to_10bit(avg,cfg.ADC_REF) > 512+cfg.joystickoffset_zul or scale_voltage_to_10bit(avg,cfg.ADC_REF) < 512-cfg.joystickoffset_zul:
            print(f"Channel {channel_names[channel]} average {avg}V is outside the calibration range")
            bJoystickInit = False
        else:
            print(f"Channel {channel_names[channel]} average {avg}V is within the calibration range")
            #ADC.ADS1263_Exit()
    #wenn bJoystickInit = True, sende mqtt Nachricht "Joystick init ok"
    if bJoystickInit:
        client.publish(cfg.MQTT_ID + cfg.MQTT_TOPIC + cfg.INIT_TOPIC, "Joystick init ok")
        print("Joystick init ok Nachricht veröffentlicht")
        GPIO.output(cfg.PIN_JOYSTICKINIT, GPIO.HIGH)
    else:
        client.publish(cfg.MQTT_ID + cfg.MQTT_TOPIC + cfg.INIT_TOPIC, "Joystick init failed")
        print("Joystick init failed Nachricht veröffentlicht")
        #gpio cfg.joystickinit pin low
        GPIO.output(cfg.PIN_JOYSTICKINIT, GPIO.LOW)


#main function
def main(): 
    try:

        # Verbindung zum MQTT-Broker herstellen
        print("Verbindung zum Broker herstellen", broker)
        client.connect(broker)

        client.on_connect = on_connect
        client.loop_start()
        print("Verbindung zum Broker hergestellt")

        ADC = ADS1263.ADS1263()
        
        # The faster the rate, the worse the stability
        # and the need to choose a suitable digital filter(REG_MODE1)
        if (ADC.ADS1263_init_ADC1('ADS1263_50SPS') == -1):
            exit()
        ADC.ADS1263_SetMode(0) # 0 is singleChannel, 1 is diffChannel

        #initialise joystick
        init_joystick()

        # MQTT-Nachricht veröffentlichen, dass der Joystick aktiv ist
        client.publish(cfg.MQTT_ID+cfg.MQTT_TOPIC+cfg.OS_TOPIC, "Joystick Task gestartet")
        print("Joystick Task alive Nachricht veröffentlicht")


        if(TEST_ADC1):       # ADC1 Test
            global bJoystickInit, channel_names
            channelList = [cfg.channel_mapping[name] for name in channel_names]  
            mqtt_JoystickpubTimer = time() + cfg.MQTT_JoystickTimer
            mqtt_JoystickRawpubTimer = time() + cfg.MQTT_JoystickRawTimer
            while(1):
                ADC_Value = ADC.ADS1263_GetAll(channelList)    # get ADC1 value
                # erzeuge liste für alle channel values
                VR150_liste_channel_values = []
                #'CS', 'HUB', 'CD', 'LICHT', 'KS', 'KD' ->  VR 153 cs, 152 cd, 151 ks, 150 kd, 154 hub, 155 licht
                # set ADC_Value to VR150_liste_channel_values - warum 2* ? - wahrscheinlich wegen 2.5V/5V Referenzspannung
                VR150_liste_channel_values = [scale_32bit_to_1023(ADC_Value[5]*2), scale_32bit_to_1023(ADC_Value[4]*2), 
                                              scale_32bit_to_1023(ADC_Value[2]*2), scale_32bit_to_1023(ADC_Value[0]*2),
                                              scale_32bit_to_1023(ADC_Value[1]*2), scale_32bit_to_1023(ADC_Value[3]*2)]

                for i, channel in enumerate(channelList):
                    if ADC_Value[i]>>31 ==1:
                        print("ADC1 IN %s = -%lf" %(str(channel_names[channel]), (cfg.ADC_REF*2 - ADC_Value[i] * cfg.ADC_REF / 0x80000000)))  
                    else:
                        print("ADC1 IN %s = %lf" %(str(channel_names[channel]), (ADC_Value[i] * cfg.ADC_REF / 0x7fffffff)))   # 32bit

                if time() > mqtt_JoystickpubTimer and bJoystickInit:
                    mqtt_JoystickpubTimer = time() + cfg.MQTT_JoystickTimer
                    client.publish(cfg.MQTT_ID+cfg.MQTT_TOPIC+cfg.JOYSTICKVAL_TOPIC+cfg.JOYSTICKMODBUSVR150_TOPIC, str(VR150_liste_channel_values[0:6]))
                    # for i, channel in enumerate(channelList):
                    #     client.publish(cfg.MQTT_ID+cfg.MQTT_TOPIC+cfg.JOYSTICKVAL_TOPIC+'/CH'+str(channel_names[channel]),str(scale_voltage_to_10bit(  (ADC_Value[i] * cfg.ADC_REF / 0x7fffffff),cfg.ADC_REF) ))
                if time() > mqtt_JoystickRawpubTimer and not bJoystickInit:
                    mqtt_JoystickRawpubTimer = time() + cfg.MQTT_JoystickRawTimer
                    for i, channel in enumerate(channelList):
                        client.publish(cfg.MQTT_ID+cfg.MQTT_TOPIC+cfg.JOYSTICKVAL_TOPIC+'/CH'+str(channel_names[channel]),'Init Err: v='+ str(round((ADC_Value[i] * cfg.ADC_REF / 0x7fffffff),3)) +' i: ' + str(scale_voltage_to_10bit(  (ADC_Value[i] * cfg.ADC_REF / 0x7fffffff),cfg.ADC_REF) 
                                        +' i: ' + str(scale_voltage_to_10bit(  (ADC_Value[i] * cfg.ADC_REF / 0x7fffffff),cfg.ADC_REF) ) +' diffi: ' + str(scale_voltage_to_10bit(  (ADC_Value[i] * cfg.ADC_REF / 0x7fffffff),cfg.ADC_REF)-512 )))
                for i, channel in enumerate(channelList):
                    print("\33[2A")
             
        elif(TEST_ADC2):
            if (ADC.ADS1263_init_ADC2('ADS1263_ADC2_400SPS') == -1):
                exit()
            while(1):
                ADC_Value = ADC.ADS1263_GetAll_ADC2()   # get ADC2 value
                for i in range(0, 10):
                    if(ADC_Value[i]>>23 ==1):
                        print("ADC2 IN%d = -%lf"%(i, (cfg.ADC_REF*2 - ADC_Value[i] * cfg.ADC_REF / 0x800000)))
                    else:
                        print("ADC2 IN%d = %lf"%(i, (ADC_Value[i] * cfg.ADC_REF / 0x7fffff)))     # 24bit
                print("\33[11A")

        elif(TEST_ADC1_RATE):    # rate test
            time_start = time()
            ADC_Value = []
            isSingleChannel = True
            if isSingleChannel:
                while(1):
                    ADC_Value.append(ADC.ADS1263_GetChannalValue(0))
                    if len(ADC_Value) == 5000:
                        time_end = time()
                        print(time_start, time_end)
                        print(time_end - time_start)
                        print('frequency = ', 5000 / (time_end - time_start))
                        break
            else:
                while(1):
                    ADC_Value.append(ADC.ADS1263_GetChannalValue(0))
                    if len(ADC_Value) == 5000:
                        time_end = time()
                        print(time_start, time_end)
                        print(time_end - time_start)
                        print('frequency = ', 5000 / (time_end - time_start))
                        break

        elif(TEST_RTD):     # RTD Test
            while(1):
                ADC_Value = ADC.ADS1263_RTD_Test()
                RES = ADC_Value / 2147483647.0 * 2.0 *2000.0       #2000.0 -- 2000R, 2.0 -- 2*i
                print("RES is %lf"%RES)
                TEMP = (RES/100.0 - 1.0) / 0.00385      #0.00385 -- pt100
                print("TEMP is %lf"%TEMP)
                print("\33[3A")
            
        ADC.ADS1263_Exit()

    except IOError as e:
        print(e)
    
    except KeyboardInterrupt:
        print("ctrl + c:")
        print("Program end")
        ADC.ADS1263_Exit()
        exit()


if __name__ == "__main__":
    main()
