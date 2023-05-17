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

def crc16(data : bytearray, offset=0, length=-1):
    if length < 0:
        length = len(data)
    
    if data is None or offset < 0 or offset > len(data)- 1 and offset+length > len(data):
        return 0

    crc = 0xFFFF
    for i in range(0, length):
        crc ^= data[offset + i] << 8

    for j in range(0,8):
        if (crc & 0x8000) > 0:
            crc =(crc << 1) ^ 0x1021
        else:
            crc = crc << 1

    return crc & 0xFFFF


def generate_logfile_name():
    now = datetime.datetime.utcnow().replace(microsecond=0)
    isostring = now.isoformat()  # string 2021-04-27T23:17:31
    isostring = isostring.replace("-", "")  # string 20210427T23:17:31
    isostring = isostring.replace(":", "")  # string 20210427T231731, oi ?oi iaai
    return "log/BETA_gcs-" + isostring + ".bin"

gyro_calib = [0.5174269005847948, -3.421812865497076, -0.24684210526315856]

if __name__ == '__main__':
    static_payload_size = None

    radio2.begin()

    radio2.openReadingPipe(1, b'\x9a\x78\x56\x34\x12')

    radio2.setCRCLength(RF24_CRC_8)
    radio2.setAddressWidth(5)
    radio2.channel = 77
    radio2.setDataRate(RF24_250KBPS)
    radio2.setAutoAck(True)


    if static_payload_size is not None:
        radio2.disableDynamicPayloads()
        radio2.payloadSize = static_payload_size
    else:
        radio2.enableDynamicPayloads()

    radio2.enableAckPayload()
    radio2.enableDynamicAck()
    radio2.setCRCLength(RF24_CRC_DISABLED)
 
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
            # print('got data %s' % data)
            packet = data
            packet_size = len(packet)
            biter = struct.pack("<B", packet_size)
            unix = time.time()
            p_unix = struct.pack("<d", unix)
            record = p_unix + biter + packet
            f.write(record)
            f.flush()

            try:
                if data[0] == 0x01:

                    print("+++++++++++++++BETA+++++++++++++++")#ОК
                    unpack_data = struct.unpack("<B2H3f2IbH", data[:28])
                    print(unpack_data)
                    print ("num:", unpack_data[1])
                    print ("time_s:", unpack_data[2])
                    print ("latitude:", unpack_data[3])
                    print ("lontitude:", unpack_data[4])
                    print ("altitude:", unpack_data[5])
                    print ("gps_time_s:", unpack_data[6])
                    print ("gps_time_us:", unpack_data[7])
                    print ("FIX:", unpack_data[8])
                   
                else:
                    pass#print('got data %s' % data)
            except Exception as e:
                print(e)
            #print(data)
            #print(data[0])
            fraw.write(data)
            fraw.flush()
        else:
            #print('got no data')
            pass
        time.sleep(0.1)