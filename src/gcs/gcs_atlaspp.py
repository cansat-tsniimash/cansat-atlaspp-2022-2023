import sys
import argparse
import time
import struct
import datetime

from RF24 import RF24_PA_LOW, RF24_PA_HIGH, RF24_PA_MAX
from RF24 import RF24_1MBPS, RF24_250KBPS, RF24_2MBPS
from RF24 import RF24_CRC_16, RF24_CRC_8, RF24_CRC_DISABLED
from RF24 import RF24 as RF24_CLASS
from RF24 import RF24_CRC_DISABLED
from RF24 import RF24_CRC_8
from RF24 import RF24_CRC_16


radio2=RF24_CLASS(24, 1)
#radio2=RF24_CLASS(22, 0)


def generate_logfile_name():
    now = datetime.datetime.utcnow().replace(microsecond=0)
    isostring = now.isoformat()  # string 2021-04-27T23:17:31
    isostring = isostring.replace("-", "")  # string 20210427T23:17:31
    isostring = isostring.replace(":", "")  # string 20210427T231731, oi ?oi iaai
    return "atlaspp_gcs-" + isostring + ".bin"

gyro_calib = [0.5174269005847948, -3.421812865497076, -0.24684210526315856]

if __name__ == '__main__':
    static_payload_size = None

    radio2.begin()

    radio2.openReadingPipe(1, b'\x9a\x78\x56\x34\x12')

    radio2.setCRCLength(RF24_CRC_8)
    radio2.setAddressWidth(5)
    radio2.channel = 11
    radio2.setDataRate(RF24_250KBPS)
    radio2.setAutoAck(True)


    if static_payload_size is not None:
        radio2.disableDynamicPayloads()
        radio2.payloadSize = static_payload_size
    else:
        radio2.enableDynamicPayloads()

    #radio2.disableAckPayload()
 
    radio2.startListening()
    radio2.printDetails()

    filename = generate_logfile_name()
    f = open(filename, 'wb')
    fname_raw = filename + ".raw"
    fraw = open(fname_raw, 'wb')
    #summ = [0, 0, 0]
    #count =f 0
    while True:
        has_payload, pipe_number = radio2.available_pipe()
        #print(f'has_payload-{has_payload}, pipe_number={pipe_number}')

        if has_payload:
            payload_size = static_payload_size
            if payload_size is None:
                payload_size = radio2.getDynamicPayloadSize()

            data = radio2.read(payload_size)
            print('got data %s' % data)
            packet = data
            packet_size = len(packet)
            biter = struct.pack("<B", packet_size)
            unix = time.time()
            p_unix = struct.pack("<d", unix)
            record = p_unix + biter + packet
            f.write(record)
            f.flush()

            try:
                if data[0] == 0x21:
                    print("==== PACKET ORIENT & BMP ====")#ОК
                    unpack_data = struct.unpack("<B2H9hHIH", data[:31])
                    print(unpack_data)
                    print ("Time:", unpack_data[2])
                    print ("Number:", unpack_data[1])

                    print ("Gyro:", unpack_data[6:9])
                    print ("Acceler:", unpack_data[3:6])
                    print ("Mag:", unpack_data[9:12])
                    print ("Temperature BMP:", unpack_data[12])
                    print ("Pressure BMP:", unpack_data[13],'\n\n\n')
                    
                    #summ[0] += ([x/1000 for x in unpack_data[9:12]])[0]
                    #summ[1] += ([x/1000 for x in unpack_data[9:12]])[1]
                    #summ[2] += ([x/1000 for x in unpack_data[9:12]])[2]
                    #count += 1
                    #print([x/count for x in summ])
                elif data[0] == 0x20:
                    print("==== LUX & STATE ====")
                    unpack_data = struct.unpack("<B5H", data[:11])
                    print ("Time:", unpack_data[2])
                    print ("Number:", unpack_data[1])

                    print ("Lux:", unpack_data[3])
                    print ("State:", unpack_data[4],'\n\n\n')
                    
                  
                elif data[0] == 0x06:
                    unpack_data = struct.unpack("<B2Hh3fbH", data[:22])
                    print("==== GPS ====")

                    print ("Time:", unpack_data[2])
                    print ("Number:", unpack_data[1])

                    print ("Temperature DS18B20:", unpack_data[3]/10)
                    print ("Latitude:", unpack_data[4])
                    print ("Lontitude:", unpack_data[5])
                    print ("Altitude:", unpack_data[6])
                    print ("FIX GPS:", unpack_data[7],'\n\n\n')

                elif data[0] == 0x08:
                    unpack_data = struct.unpack("<B2H2IH", data[:15])
                    print("==== PACKET TIME ====")
                    print ("Time:", unpack_data[2])
                    print ("Number:", unpack_data[1])

                    print ("GPS_time_S:", unpack_data[3])
                    print ("GPS_time_US:", unpack_data[4],'\n\n\n')
                    
                else:
                    print('got data %s' % data)
            except Exception as e:
                print(e)
            #print(data)
            #print(data[0])
            #print('got data %s' % data)
            fraw.write(data)
            fraw.flush()
        else:
            #print('got no data')
            pass
        time.sleep(0.1)
