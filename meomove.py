import RPi.GPIO as GPIO
import time
import picamera
import smbus
import math
import astral
#import sys
import os
import astropy.units as u #warning, weird unit declarations *u.(unit)
from astropy.coordinates import SkyCoord, EarthLocation, AltAz
import datetime
import Adafruit_DHT
from __future__ import division
import logging

#WatchList
watch_list = [
'sirius',
'canopus',
'arcturus',
'vega',
'capella',
'rigel',
'procyon',
'achernar',
'betelgeuse',
'polaris',
'acrux',
'altair',
'aldebaran',
'antares',
'spica',
'pollux',
'fomalhaut',
'deneb',
'regulus',
'castor',
'gacrux',
'shaula',
'eta carinae',
'alpha centauri',
'proxima centauri'
]



# BMP085 default address.
BMP085_I2CADDR           = 0x77

# Operating Modes
BMP085_ULTRALOWPOWER     = 0
BMP085_STANDARD          = 1
BMP085_HIGHRES           = 2
BMP085_ULTRAHIGHRES      = 3

# BMP085 Registers
BMP085_CAL_AC1           = 0xAA  # R   Calibration data (16 bits)
BMP085_CAL_AC2           = 0xAC  # R   Calibration data (16 bits)
BMP085_CAL_AC3           = 0xAE  # R   Calibration data (16 bits)
BMP085_CAL_AC4           = 0xB0  # R   Calibration data (16 bits)
BMP085_CAL_AC5           = 0xB2  # R   Calibration data (16 bits)
BMP085_CAL_AC6           = 0xB4  # R   Calibration data (16 bits)
BMP085_CAL_B1            = 0xB6  # R   Calibration data (16 bits)
BMP085_CAL_B2            = 0xB8  # R   Calibration data (16 bits)
BMP085_CAL_MB            = 0xBA  # R   Calibration data (16 bits)
BMP085_CAL_MC            = 0xBC  # R   Calibration data (16 bits)
BMP085_CAL_MD            = 0xBE  # R   Calibration data (16 bits)
BMP085_CONTROL           = 0xF4
BMP085_TEMPDATA          = 0xF6
BMP085_PRESSUREDATA      = 0xF6

# Commands
BMP085_READTEMPCMD       = 0x2E
BMP085_READPRESSURECMD   = 0x34

class BMP085(object):
	def __init__(self, mode=BMP085_STANDARD, address=BMP085_I2CADDR, i2c=None, **kwargs):
		self._logger = logging.getLogger('Adafruit_BMP.BMP085')
		# Check that mode is valid.
		if mode not in [BMP085_ULTRALOWPOWER, BMP085_STANDARD, BMP085_HIGHRES, BMP085_ULTRAHIGHRES]:
			raise ValueError('Unexpected mode value {0}.  Set mode to one of BMP085_ULTRALOWPOWER, BMP085_STANDARD, BMP085_HIGHRES, or BMP085_ULTRAHIGHRES'.format(mode))
		self._mode = mode
		# Create I2C device.
		if i2c is None:
			import Adafruit_GPIO.I2C as I2C
			i2c = I2C
		self._device = i2c.get_i2c_device(address, **kwargs)
		# Load calibration values.
		self._load_calibration()

	def _load_calibration(self):
		self.cal_AC1 = self._device.readS16BE(BMP085_CAL_AC1)   # INT16
		self.cal_AC2 = self._device.readS16BE(BMP085_CAL_AC2)   # INT16
		self.cal_AC3 = self._device.readS16BE(BMP085_CAL_AC3)   # INT16
		self.cal_AC4 = self._device.readU16BE(BMP085_CAL_AC4)   # UINT16
		self.cal_AC5 = self._device.readU16BE(BMP085_CAL_AC5)   # UINT16
		self.cal_AC6 = self._device.readU16BE(BMP085_CAL_AC6)   # UINT16
		self.cal_B1 = self._device.readS16BE(BMP085_CAL_B1)     # INT16
		self.cal_B2 = self._device.readS16BE(BMP085_CAL_B2)     # INT16
		self.cal_MB = self._device.readS16BE(BMP085_CAL_MB)     # INT16
		self.cal_MC = self._device.readS16BE(BMP085_CAL_MC)     # INT16
		self.cal_MD = self._device.readS16BE(BMP085_CAL_MD)     # INT16
		self._logger.debug('AC1 = {0:6d}'.format(self.cal_AC1))
		self._logger.debug('AC2 = {0:6d}'.format(self.cal_AC2))
		self._logger.debug('AC3 = {0:6d}'.format(self.cal_AC3))
		self._logger.debug('AC4 = {0:6d}'.format(self.cal_AC4))
		self._logger.debug('AC5 = {0:6d}'.format(self.cal_AC5))
		self._logger.debug('AC6 = {0:6d}'.format(self.cal_AC6))
		self._logger.debug('B1 = {0:6d}'.format(self.cal_B1))
		self._logger.debug('B2 = {0:6d}'.format(self.cal_B2))
		self._logger.debug('MB = {0:6d}'.format(self.cal_MB))
		self._logger.debug('MC = {0:6d}'.format(self.cal_MC))
		self._logger.debug('MD = {0:6d}'.format(self.cal_MD))

	def _load_datasheet_calibration(self):
		# Set calibration from values in the datasheet example.  Useful for debugging the
		# temp and pressure calculation accuracy.
		self.cal_AC1 = 408
		self.cal_AC2 = -72
		self.cal_AC3 = -14383
		self.cal_AC4 = 32741
		self.cal_AC5 = 32757
		self.cal_AC6 = 23153
		self.cal_B1 = 6190
		self.cal_B2 = 4
		self.cal_MB = -32767
		self.cal_MC = -8711
		self.cal_MD = 2868

	def read_raw_temp(self):
		"""Reads the raw (uncompensated) temperature from the sensor."""
		self._device.write8(BMP085_CONTROL, BMP085_READTEMPCMD)
		time.sleep(0.005)  # Wait 5ms
		raw = self._device.readU16BE(BMP085_TEMPDATA)
		self._logger.debug('Raw temp 0x{0:X} ({1})'.format(raw & 0xFFFF, raw))
		return raw

	def read_raw_pressure(self):
		"""Reads the raw (uncompensated) pressure level from the sensor."""
		self._device.write8(BMP085_CONTROL, BMP085_READPRESSURECMD + (self._mode << 6))
		if self._mode == BMP085_ULTRALOWPOWER:
			time.sleep(0.005)
		elif self._mode == BMP085_HIGHRES:
			time.sleep(0.014)
		elif self._mode == BMP085_ULTRAHIGHRES:
			time.sleep(0.026)
		else:
			time.sleep(0.008)
		msb = self._device.readU8(BMP085_PRESSUREDATA)
		lsb = self._device.readU8(BMP085_PRESSUREDATA+1)
		xlsb = self._device.readU8(BMP085_PRESSUREDATA+2)
		raw = ((msb << 16) + (lsb << 8) + xlsb) >> (8 - self._mode)
		self._logger.debug('Raw pressure 0x{0:04X} ({1})'.format(raw & 0xFFFF, raw))
		return raw

	def read_temperature(self):
		"""Gets the compensated temperature in degrees celsius."""
		UT = self.read_raw_temp()
		# Datasheet value for debugging:
		#UT = 27898
		# Calculations below are taken straight from section 3.5 of the datasheet.
		X1 = ((UT - self.cal_AC6) * self.cal_AC5) >> 15
		X2 = (self.cal_MC << 11) // (X1 + self.cal_MD)
		B5 = X1 + X2
		temp = ((B5 + 8) >> 4) / 10.0
		self._logger.debug('Calibrated temperature {0} C'.format(temp))
		return temp

	def read_pressure(self):
		"""Gets the compensated pressure in Pascals."""
		UT = self.read_raw_temp()
		UP = self.read_raw_pressure()
		# Datasheet values for debugging:
		#UT = 27898
		#UP = 23843
		# Calculations below are taken straight from section 3.5 of the datasheet.
		# Calculate true temperature coefficient B5.
		X1 = ((UT - self.cal_AC6) * self.cal_AC5) >> 15
		X2 = (self.cal_MC << 11) // (X1 + self.cal_MD)
		B5 = X1 + X2
		self._logger.debug('B5 = {0}'.format(B5))
		# Pressure Calculations
		B6 = B5 - 4000
		self._logger.debug('B6 = {0}'.format(B6))
		X1 = (self.cal_B2 * (B6 * B6) >> 12) >> 11
		X2 = (self.cal_AC2 * B6) >> 11
		X3 = X1 + X2
		B3 = (((self.cal_AC1 * 4 + X3) << self._mode) + 2) // 4
		self._logger.debug('B3 = {0}'.format(B3))
		X1 = (self.cal_AC3 * B6) >> 13
		X2 = (self.cal_B1 * ((B6 * B6) >> 12)) >> 16
		X3 = ((X1 + X2) + 2) >> 2
		B4 = (self.cal_AC4 * (X3 + 32768)) >> 15
		self._logger.debug('B4 = {0}'.format(B4))
		B7 = (UP - B3) * (50000 >> self._mode)
		self._logger.debug('B7 = {0}'.format(B7))
		if B7 < 0x80000000:
			p = (B7 * 2) // B4
		else:
			p = (B7 // B4) * 2
		X1 = (p >> 8) * (p >> 8)
		X1 = (X1 * 3038) >> 16
		X2 = (-7357 * p) >> 16
		p = p + ((X1 + X2 + 3791) >> 4)
		self._logger.debug('Pressure {0} Pa'.format(p))
		return p

	def read_altitude(self, sealevel_pa=101325.0):
		"""Calculates the altitude in meters."""
		# Calculation taken straight from section 3.6 of the datasheet.
		pressure = float(self.read_pressure())
		altitude = 44330.0 * (1.0 - pow(pressure / sealevel_pa, (1.0/5.255)))
		self._logger.debug('Altitude {0} m'.format(altitude))
		return altitude

	def read_sealevel_pressure(self, altitude_m=0.0):
		"""Calculates the pressure at sealevel when given a known altitude in
		meters. Returns a value in Pascals."""
		pressure = float(self.read_pressure())
		p0 = pressure / pow(1.0 - altitude_m/44330.0, 5.255)
		self._logger.debug('Sealevel pressure {0} Pa'.format(p0))
		return p0


class TiltSensor:
	address = None
	def __init__(self):
		self.bus = smbus.SMBus(1)
		self.address = 0x53

		self.bus.write_byte_data(self.address,0x2D,0x08) #power ctl,measure
		self.bus.write_byte_data(self.address,0x2C,0x0B) #BW_RATE,BW_RATE_100HZ
		
		value = self.bus.read_byte_data(self.address,0x31) #DATA_FORMAT
		value &= ~0x0F;
		value |= 0x00; #range_flag
		value |= 0x08;
		self.bus.write_byte_data(self.address,0x31, value) #DATA_FORMAT

	def get_deg_up(self):
		bytes = self.bus.read_i2c_block_data(self.address,0x32, 6) #AXES_DATA

		x = bytes[0] | (bytes[1] << 8)
		if(x & (1 << 16 - 1)):
			x = x - (1<<16)
			
		x = x * 0.004 #* 9.80665 scale multiplier * Earths Gravity
		x = round(x, 4)
		y_deg = (x * 90)
		
		return (y_deg)
		

class CompassSensor:

	__scales = {
		0.88: [0, 0.73],
		1.30: [1, 0.92],
		1.90: [2, 1.22],
		2.50: [3, 1.52],
		4.00: [4, 2.27],
		4.70: [5, 2.56],
		5.60: [6, 3.03],
		8.10: [7, 4.35],
	}

	def __init__(self, port=1, address=0x1E, gauss=1.3, declination=(0,0)):
		self.bus = smbus.SMBus(port)
		self.address = address

		(degrees, minutes) = declination
		self.__declDegrees = degrees
		self.__declMinutes = minutes
		self.__declination = (degrees + minutes / 60) * math.pi / 180

		(reg, self.__scale) = self.__scales[gauss]
		self.bus.write_byte_data(self.address, 0x00, 0x70) # 8 Average, 15 Hz, normal measurement
		self.bus.write_byte_data(self.address, 0x01, reg << 5) # Scale
		self.bus.write_byte_data(self.address, 0x02, 0x00) # Continuous measurement

	def declination(self):
		return (self.__declDegrees, self.__declMinutes)

	def twos_complement(self, val, len):
		# Convert twos compliment to integer
		if (val & (1 << len - 1)):
			val = val - (1<<len)
		return val

	def __convert(self, data, offset): 
		val = self.twos_complement(data[offset] << 8 | data[offset+1], 16)
		if val == -4096: return None
		return round(val * self.__scale, 4)

	def axes(self):
		data = self.bus.read_i2c_block_data(self.address, 0x00)
		#print map(hex, data)
		x = self.__convert(data, 3)
		y = self.__convert(data, 7)
		z = self.__convert(data, 5)
		return (x,y,z)

	def heading(self):
		(x, y, z) = self.axes()
		headingRad = math.atan2(y, x)
		headingRad += self.__declination

		# Correct for reversed heading
		if headingRad < 0:
			headingRad += 2 * math.pi

		# Check for wrap and compensate
		elif headingRad > 2 * math.pi:
			headingRad -= 2 * math.pi

		# Convert to degrees from radians
		headingDeg = headingRad * 180 / math.pi
		return headingDeg

	def degrees(self, headingDeg):
		degrees = math.floor(headingDeg)
		minutes = round((headingDeg - degrees) * 60)
		return (degrees, minutes)

class Stepper:


	def __init__ (self, enable_pin, magpins = (0,0,0,0), delay = 2):
		self.enable_pin = enable_pin
		self.delay = (delay / 1000)
		
		(a1,a2,b1,b2) = magpins
		self.a1_pin = a1
		self.a2_pin = a2
		self.b1_pin = b1
		self.b2_pin = b2
		
	def upset(self):
		GPIO.setup(self.enable_pin, GPIO.OUT)
		GPIO.setup(self.a1_pin, GPIO.OUT)
		GPIO.setup(self.a2_pin, GPIO.OUT)
		GPIO.setup(self.b1_pin, GPIO.OUT)
		GPIO.setup(self.b2_pin, GPIO.OUT)
		GPIO.output(self.enable_pin, 1)
		time.sleep(.5)

	def right(self,steps):  #corrected for new gear set up
		for i in range(0, steps):
			self.setStep(1, 0, 1, 0)
			time.sleep(self.delay)
			self.setStep(0, 1, 1, 0)
			time.sleep(self.delay)
			self.setStep(0, 1, 0, 1)
			time.sleep(self.delay)
			self.setStep(1, 0, 0, 1)
			time.sleep(self.delay)

	def left(self,steps):  #corrected for new gear setup
		for i in range(0, steps):
			self.setStep(1, 0, 0, 1)
			time.sleep(self.delay)
			self.setStep(0, 1, 0, 1)
			time.sleep(self.delay)
			self.setStep(0, 1, 1, 0)
			time.sleep(self.delay)
			self.setStep(1, 0, 1, 0)
			time.sleep(self.delay)

	def setStep(self,w1, w2, w3, w4):
		GPIO.output(self.a1_pin, w1)
		GPIO.output(self.a2_pin, w2)
		GPIO.output(self.b1_pin, w3)
		GPIO.output(self.b2_pin, w4)


class Platform:

	def __init__(self):
		#instantiations
		self.compass = CompassSensor(gauss = 4.7, declination = (-4,15))
		self.rotation = Stepper(13, magpins = (27,17,22,26), delay = 3)
		self.oculous = Stepper(6,magpins = (5,10,11,9),delay = 3) 
		self.tilt = TiltSensor() 
		self.baro = BMP085()
		self.camera = picamera.PiCamera()
		
		#setups
		self.rotation.upset()
		self.oculous.upset()
		self.get_sun_state(True,True)
		self.camera.resolution = (1024,768)
		self.update_folder()
		
		#location info
		self.latitude = 40.3 
		self.longitude = -86.9
		self.earthcoord = EarthLocation(lat = self.latitude*u.deg,lon = self.longitude*u.deg,height = 167*u.m)
		
		#other sensor Info
		self.dht_pin = 23  # 3.3v power, I2C
		
		#environment Info
		self.sun_state = down
		self.day_time = None
		self.new_day = False
		self.day_switch = 0
		
		#fs Info
		self.pic_path = ('/home/pi/projects/MEO/pics')
		self.cwd = 0
		
		#temp Containers?
		self.tupes = []
		
	#local file System Stuff
	
	def daily_pic_folder(self):
		to_day = str(datetime.datetime.utcnow()) 
		daystamp = to_day[0:10]
		file_path = ('{1}/Pics - {0}'.format(daystamp,self.pic_path))
		try:
			os.system('mkdir {}'.format(file_path)
		except AlreadyExists:
			print("the folder/'s already there, Kid"
		print('Made Daily Folder for {}'.format(daystamp))
		
	def update_folder(self):
		to_day = str(datetime.datetime.utcnow()) 
		daystamp = to_day[0:10]
		self.cwd = daystamp
	
	def create_body_folders(self,self.tupes)
		for item in na_mes:
			try:
				os.system('mkdir {}/Pics - {}/{}'.format(self.pic_path,self.cwd,item[0]) 
			except AlreadyExists:
				continue
				
	#camera stuff
	def take_picture(self,body):
		self.camera.start_preview()
		tn = str(datetime.datetime.utcnow())
		timestamp = tn[11:19]
		time.sleep(.5)
		camera.capture('{}/Pics - {}/{}/{}.jpg'.format(self.pic_path,self.cwd,body,timestamp))
		
	#starstuff and environment
	def get_altaz_of(self,simbad_ref):
		(hum,temp) = Adafruit_DHT.read_retry(11,self.dht_pin)
		hpa = (self.baro.read_pressure()/100.00)
		target = SkyCoord.from_name(simbad_ref)
		time_now = datetime.datetime.utcnow() 
		t_altaz = target.transform_to(AltAz(obstime = time_now, location = self.earthcoord, relative_humidity = hum*u.pct,pressure = hpa*u.hPa,temperature = temp*u.deg_C))
		return (t_altaz.az,t_altaz.alt)
		
	def get_sun_state(self, p = False, dl = False, r = False):
		t_now = str(datetime.datetime.utcnow())
		h_now = int(t_now[11:13]
		m_now = int(t_now[14:16]
		if h_now < self.day_switch:
			self.daily_pic_folder()
			self.new_day = True
		self.day_switch = h_now
		tmins_now = ((h_now * 60) + m_now)
		lcn = astral.Location()
		lcn.name = 'Mikey\'s'
		lcn.region = 'Laffy'
		lcn.latitude = self.latitude
		lcn.longitude = self.longitude
		lcn.timezone = 'US/Eastern'
		lcn.elevation = 600
		sun_info = lcn.sun(date = datetime.date(int(t_now[0:4]),int(t_now[5:7]),int(t_now[8:10])), local = True)
		sunrise = str(sun_info['sunrise'])
		sunrise = sunrise[11:16]
		sunrise_tmins = (int(sunrise[11:13])*60 + int(sunrise[14:16])
		sunset = str(sun_info['sunset'])
		sunset = sunset[11:16]
		sunset_tmins = (int(sunset[11:13])*60 + int(sunset[14:16])
		if (sunrise_tmins + 15) <= tmins_now < (sunset_tmins - 15):
			self.sun_state = above
		elif (sunrise_tmins - 15) <= tmins_now < (sunrise_tmins + 15):
			self.sun_state = up
		elif (sunset_tmins - 15) <= tmins_now < (sunset_tmins + 15):
			self.sun_state = down
		else:
			self.sun_state = view
		if p or self.new_day:
			print('Today, the sun rises at {}, and sets at {}'.format(sunrise, sunset))
			if self.new_day:
				self.new_day = False
		if dl:
			self.day_time = sunset_tmins - sunrise_tmins
		if r:
			return (sunset_tmins - tmins_now)
			
	def build_list(self):
		d = []
		names = []
		tmp_az,tmp_alt = 0,0
		for item in watch_list:
			(tmp_az,tmp_alt) = self.get_altaz_of(item)
			if tmp_alt > 30*u.deg:
				form_alt = str(tmp_alt*u.deg)
				tmp_alt = float(form_alt[0:5])
				form_az = str(tmp_az*u.deg)
				tmp_az = float(form_az[0:6])
				d.append(tuple((item,round(tmp_alt,2),round(tmp_az,2))))
			else:
				pass
		
		for item in d:
			names.append(item[0])
		
		self.tupes = d		
		self.create_body_folders(names)
		return d
			
	#Platform methods for tilt detection using accelerometer and a stepper
	def keesy(self,edg):
		if 0 <= edg <= 90:
			des_edg = edg
			cur_edg = self.tilt.get_deg_up()
			if cur_edg != des_edg:
				if des_edg > cur_edg:
					read_edg = cur_edg
					while read_edg < des_edg:
						self.oculous.right(4) #adjusted for new gearing
						read_edg = self.tilt.get_deg_up()
				elif des_edg < cur_edg:
					read_edg = cur_edg
					while read_edg > des_edg:
						self.oculous.left(4) #adjusted for new gearing
						read_edg = self.tilt.get_deg_up()
				else:
					print('well what is it then kid?')
			else:
				print('You\'re on it, Kid!')
		else:
			print('Where you tryin to go, Kid?')

	#Platform methods for rotation using the compass and a stepper  
	def stop_range_terms(self,st_deg) 
	# st_deg is passed as 4 float, please
	stop_list = []
	stop_list.clear()
	if 0 <= st_deg <=360
		if 358 > st_deg	> 2:
			stop_list = [st_deg-2, st_deg+2]
	return stop_list


	def shave_mins(self,wtf): #more accurate, wtf passed as 4 float please
		s_degs,s_mins = self.heading()
		t_minutes = ((s_degs * 60) + s_mins)
		d_minutes = ((wtf * 60)//1)
		if d_minutes > t_minutes:
			u_minutes = d_minutes
			while u_minutes < t_minutes:
				self.rotation.right(1)
				(u_degs,u_mins) = self.heading()
				u_minutes = ((u_degs * 60) + u_mins)
		elif d_minutes < t_minutes:
			u_minutes = d_minutes
			while u_minutes > t_minutes:
				self.rotation.left(1)
				(u_degs,u_mins) = self.heading()
				u_minutes = ((u_degs * 60) + u_mins)


	def heading(self,p = False):
		"""reads and returns heading in degrees, minutes.  if True is passed will read, print, and return heading"""
		time.sleep(.1)
		(dghead,mins) = self.compass.degrees(self.compass.heading()) #only Call to heading
		if p:
			print(dghead,mins)
		return (dghead,mins)
	
	def seek(self,des_deg): #higher speed turning, passed as 4 float please
		chk_list = []
		chk_list.clear()
		chk_list = self.stop_range_terms(des_deg) #as 4 float!!!!
		(r_deg,r_mins) = self.heading()
		if 358 > des_deg > 2: 
			if r_deg > 180:
				inv_a = (r_deg - 180)
				if inv_a <= des_deg < r_deg: #on left easy side, go left
					m_deg = r_deg
					while not chk_list[0] > m_deg > chk_list[1]:
						self.rotation.left(20) # Fast Skip Number
						(m_deg,m_mins) = self.heading()
					self.shave_mins(des_deg)
				else:	#on left side, right to nonzero
					m_deg = r_deg	
					while not chk_list[0] > m_deg > chk_list[1]:
						self.rotation.right(20) # Fast Skip Number
						(m_deg,m_mins) = self.heading()
					self.shave_mins(des_deg)
			elif r_deg <= 180:
				inv_a = (r_deg + 180)
				if inv_a >= des_deg > r_deg: #on right easy side, go right
					m_deg = r_deg
					while not chk_list[0] > m_deg > chk_list[1]:
						self.rotation.right(20) # Fast Skip Number
						(m_deg,m_mins) = self.heading()
					self.shave_mins(des_deg)
				else:					#on right side, left to nozero range
					m_deg = r_deg
					while not chk_list[0] > m_deg > chk_list[1]:
						self.rotation.left(20) # Fast Skip Number
						(m_deg,m_mins) = self.heading()
					self.shave_mins(des_deg)
			else:
				print('Magnetostorm??..read_deg is out of possiblerange')
				
		elif des_deg >= 358:
			save_des_deg = des_deg
			self.seek(355)
			self.shave_mins(save_des_deg)
			
		elif des_deg <= 2:
			save_des_deg = des_deg
			self.seek(5)
			self.shave_mins(save_des_deg)
			
		else:
			print('Your banana has more than 360 or less than 0')

	#Platform Class methods to tie positional, rotational, and tilt
	def target(self,x,y):
		self.seek(x)
		self.keesy(y)
		print (self.heading(),self.get_deg_up())

	def watch(self,simbad_ref = 'None',running = True, update_rate = 90):
		(bearing, altitude) = self.get_altaz_of(simbad_ref)
		if altitude > 1:
			self.target(bearing,altitude)
			while running:
				time.sleep(update_rate)
				(ubearing,ualt) = self.get_altaz_of(simbad_ref)
				if ualt < 1:
					print(simbad_ref,' has gone down  :-(')
					break
				self.target(ubearing,ualt)
		else:
			print(simbad_ref,' is not up for viewing  :-(')
			

def main():
	meo = Platform():
	running = True
	while running:
		while meo.sun_state = up: #sun rising, ending the viewing day
			meo.update_folder() #update to save files in new folder
			(dg,mn) = meo.heading() #read heading
			du = meo.tilt.get_deg_up() #read tilt angle
			meo.get_sun_state(dl = True) #calculate daylight
			if 170 < dg < 190 and du < 8: #if heading is southerly and tilt is low
				time.sleep(meo.day_time * 60)# attempt sleep through day, only attempt
				meo.get_sun_state()
			else:
				meo.target(181,5)
			
		while meo.sun_state = above: #if its daytime
			(dg,mn) = meo.heading() #ensure scope is southerly and low
			du = meo.tilt.get_deg_up()
			if 170 < dg < 190 and du < 8:
				time.sleep(meo.get_sun_state(r = True) * 60) #sleep for the time of remaining daylight
				meo.get_sun_state()
			else:
				meo.target(181,5)
	
		while meo.sun_state = down: #setting sun, 
			(dg,mn) = meo.heading() 
			du = meo.tilt.get_deg_up()
			if 170 < dg < 190 and du < 8:#ensure safe scope
				time.sleep(300)			 #and sleep five minutes
				meo.get_sun_state()
			else:
				meo.target(181,5)

		while meo.sun_state = view: #viewing time!
			spots = meo.build_list() #build up-list
			meo.create_body_folders() # create folders in current folder-day, bodies which are in names 								skipping folders which have already been created for the day
			for item in spots:			   #iterate through tuples containing names, and altaz info
				print('Aquiring: {}'.format(item[0]))
				meo.target(item[2],item[1]) #look at
				#focus
				meo.take_picture(item[0]) #take and save picture
			meo.get_sun_state()	#ensure still viewing
			
if __name__ == ('__main__'):
	main()
#meo = Platform()
#meo.watch(simbad_ref = 'sirius') # will track, adjusting ~90 seconds for default
#meo.watch(simbad_ref = 'polaris', running = False) # looks at ref, no update
#meo.watch(simbad_ref = 'betelgeuse', update_rate = 60) #looks at ref, updates ~60 seconds


