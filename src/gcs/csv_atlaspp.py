import struct


FILEPATH = "packet.bin"
CSV_ORIENT_FILEPATH = FILEPATH + '-orient' + '.csv'
CSV_STATEPHOTOREZ_FILEPATH = FILEPATH + '-statephoto280' + '.csv'
CSV_GPS_FILEPATH = FILEPATH + '-gps' + '.csv'
CSV_TIMES_FILEPATH = FILEPATH + '-time' + '.csv'
CSV_BETA_FILEPATH = FILEPATH + '-BETA'+ 'csv'

# Формат лога:
# 4 байта, флоат - время
# 1 байт, беззнаковое целое - размер пакета
# пакет длинной с указанным размером


stream = open(FILEPATH, mode="rb")

def crc16(data : bytearray, offset=0, length=-1):
    if length < 0:
        length = len(data)
    
    if data is None or offset < 0 or offset > len(data)- 1 and offset+length > len(data):
        return 0

    crc = 0xFFFF
    for i in range(0, length):
        crc ^= data[offset + i] << 8
        for j in range(0, 8):
            if (crc & 0x8000) > 0:
                crc =(crc << 1) ^ 0x1021
            else:
                crc = crc << 1
        crc = crc & 0xFFFF

    return crc & 0xFFFF


def read_packet(stream):
	packet_size_raw = stream.read(1)
	if not packet_size_raw:
		return None

	packet_size, = struct.unpack(">B", packet_size_raw)

	packet = stream.read(packet_size)

	time_raw = stream.read(8)
	time, = struct.unpack("<d", time_raw)

	return time, packet





class OrientParser:
	pack_len = 21
	def __init__(self):
		self.csv_orient = open(CSV_ORIENT_FILEPATH, "w")
		line_name = '%s;%s;%s;%s;%s;%s;%s;%s;%s;%s;%s;%s;%s;' % ("flag", "num" , "time_s" ,"acc_x", "acc_y", "acc_z", "gyro_x", "gyro_y", "gyro_z", "mag_x", "mag_y", "mag_z", "crc")
		print(line_name, file = self.csv_orient)

	def parse(self, data: bytes):
		unpacked = struct.unpack("<B2H9hHIH", data[:self.pack_len])

		flag = unpacked[0]
		num = unpacked[1]
		time_s = unpacked[2]
		acc_x = unpacked[3] / 1000
		acc_y = unpacked[4] / 1000
		acc_z = unpacked[5] / 1000
		gyro_x = unpacked[6] / 1000
		gyro_y = unpacked[7] / 1000
		gyro_z = unpacked[8] / 1000
		mag_x = unpacked[9] / 1000
		mag_y = unpacked[10] / 1000
		mag_z = unpacked[11] / 1000
		
		crc = unpacked[14]

		if (crc != crc16(data, length=(self.pack_len-2))):
			return -1

		line_orient = "%s;%s;%s;%s;%s;%s;%s;%s;%s;%s;%s;%s;%s" % (flag, num, time_s ,acc_x, acc_y, acc_z, gyro_x, gyro_y, gyro_z, mag_x, mag_y, mag_z, crc)
		print(line_orient, file = self.csv_orient)
		print(line_orient)
		return 0
		

class StatePhotorez:
	pack_len = 19
	def __init__(self):
		self.csv_bme = open(CSV_STATEPHOTOREZ_FILEPATH, "w")
		line_name = "%s;%s;%s;%s;%s;%s" % ("flag", "num" , "time_s" , "photorez", "", "bmp_pres", "status", "crc")
		print(line_name, file = self.csv_bme)
	def parse(self, data: bytes):
		unpacked = struct.unpack("<BHIHI3H", data[:self.pack_len])

		flag = unpacked[0]		
		num = unpacked[1]
		time_s = unpacked[2]
		photorez = unpacked[5]
		bmp_temp = unpacked[3]
		bmp_pres = unpacked[4]
		status = unpacked[6]
		crc = unpacked[7]

		if (crc != crc16(data, length=(self.pack_len-2))):
			return -1


		line_bme280 = "%s;%s;%s;%s;%s;%s;%s;%s" % (flag, num, time_s, photorez, bmp_temp, bmp_pres ,status, crc)
		print(line_bme280, file = self.csv_bme)
		print(line_bme280)
		return 0

class GpsParser:
	pack_len = 22
	def __init__(self):
		self.csv_dosim = open(CSV_GPS_FILEPATH, "w")
		line_name = '%s;%s;%s;%s;%s;%s;%s;%s;%s' % ("flag", "num", "time_s", "ds_temp", "lat", "lon", "alt", "fix" , "crc")
		print(line_name, file = self.csv_dosim)
	def parse(self, data: bytes):
		unpacked = struct.unpack("<B2Hh3fbH", data[:self.pack_len])	

		flag = unpacked[0]
		num = unpacked[1]
		time_s = unpacked[2]
		ds_temp = unpacked[3] / 10
		lat = unpacked[4]
		lon = unpacked[5]
		alt = unpacked[6]
		fix = unpacked[7]
		crc = unpacked[8]

		if (crc != crc16(data, length=(self.pack_len-2))):
			return -1


		line_dosim = "%s;%s;%s;%s;%s;%s;%s;%s;%s" % (flag, num, time_s , ds_temp, lat, lon, alt, fix, crc)
		print(line_dosim, file = self.csv_dosim)
		print(line_dosim)
		return 0

class TimesParser:
	pack_len = 15
	def __init__(self):
		self.csv_gps = open(CSV_TIMES_FILEPATH, "w")
		line_name = '%s;%s;%s;%s;%s;%s' % ("flag", "num", "time_s", "gps_time_s", "gps_time_us" ,"crc")
		print(line_name, file = self.csv_gps)
	def parse(self, data: bytes):
		unpacked = struct.unpack("<B2H2IH", data[:self.pack_len])

		flag = unpacked[0]
		num = unpacked[1]
		time_s = unpacked[2]
		gps_time_s = unpacked[3]
		gps_time_us = unpacked[3]
		
		crc = unpacked[5]


		if (crc != crc16(data, length=(self.pack_len-2))):
			return -1

		line_gps = "%s;%s;%s;%s;%s;%s" % (flag, num, time_s, gps_time_s, gps_time_us, crc)
		print(line_gps, file = self.csv_gps)
		print(line_gps)
		return 0
	

class BetaParser:
	pack_len = 28
	def __init__(self):
		self.csv_gps = open(CSV_BETA_FILEPATH, "w")
		line_name = '%s;%s;%s;%s;%s;%s;%s;%s;%s;%s' % ("flag", "num", "time_s", "lat", "lon" ,"alt", "gps_time_s", "gps_time_us", "fix", "crc")
		print(line_name, file = self.csv_gps)
	def parse(self, data: bytes):
		unpacked = struct.unpack("<B2H3f2IbH", data[:self.pack_len])

		flag = unpacked[0]
		num = unpacked[1]
		time_s = unpacked[2]
		lat = unpacked[3]
		lon = unpacked[4]
		alt = unpacked[5]
		gps_time_s = unpacked[6]
		gps_time_us = unpacked[7]
		fix = unpacked[8]

		crc = unpacked[9]
		


		if (crc != crc16(data, length=(self.pack_len-2))):
			return -1

		line_beta = "%s;%s;%s;%s;%s;%s;%s;%s;%s;%s" % (flag, num, time_s, lat, lon, alt, gps_time_s, gps_time_us, fix, crc)
		print(line_beta, file = self.csv_gps)
		print(line_beta)
		return 0


orient = OrientParser()
statephoto = StatePhotorez()
gps = GpsParser()
times = TimesParser()
beta = BetaParser()

buf = []

while True:
	#data = read_packet(stream)
	buf.extend(list(stream.read(1)))
	if len(buf) < 1:
		break

	#time, packet = data
	flag = buf[0]
	#print(f"flag={flag}, data={packet}, len(data)={len(packet)}")

	try:
		if flag == 0x21:
			buf.extend(list(stream.read(orient.pack_len - 1)))
			if orient.parse(bytes(buf)) != 0:
				buf = buf[1:]
			else:
				buf = buf[orient.pack_len:]


		elif flag == 0x20:
			buf.extend(list(stream.read(statephoto.pack_len - 1)))
			if statephoto.parse(bytes(buf)) != 0:
				buf = buf[1:]
			else:
				buf = buf[statephoto.pack_len:]


		elif flag == 0x06:
			buf.extend(list(stream.read(gps.pack_len - 1)))
			if gps.parse(bytes(buf)) != 0:
				buf = buf[1:]
			else:
				buf = buf[gps.pack_len:]


		elif flag == 0x08:
			buf.extend(list(stream.read(times.pack_len - 1)))
			if times.parse(bytes(buf)) != 0:
				buf = buf[1:]
			else:
				buf = buf[times.pack_len:]


		elif flag == 0x01:
			buf.extend(list(stream.read(beta.pack_len - 1)))
			if beta.parse(bytes(buf)) != 0:
				buf = buf[1:]
			else:
				buf = buf[beta.pack_len:]


		else:
			print("НЕИЗВЕСТНЫЙ ФЛАГ %d" % int(flag))
			buf = buf[1:]
			print(buf[0])
			
	except Exception as e:
		raise e
		print("НЕ МОГУ РАЗОБРАТЬ ПАКЕТ С ФЛАГОМ %x: %s" % (int(flag), e))


	# unpacked_bme = struct.unpack("<B", packet[:117])
	# unpacked_dosim = struct.unpack("<B", packet[:99])
	# unpacked_gps = struct.unpack("<B", packet[:66])
	# unpacked_state = struct.unpack("<B", packet[:71])

	#print(unpacked)
	#print("readed bytes %s of data %s at time %s" % (len(packet), packet, time))

#print("flag;BMP_temperature;LSM_acc_x;LSM_acc_y;LSM_acc_z;LSM_gyro_x;LSM_gyro_y;LSM_gyro_z;num;time_from_start;BMP_pressure;crc;time", file=csv_stream)
