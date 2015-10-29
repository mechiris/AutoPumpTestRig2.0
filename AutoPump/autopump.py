from multiprocessing import Process, Pipe
from Adafruit_PWM_Servo_Driver import PWM
import datetime, time
import subprocess as sbp
import SimpleCV as scv
import logging
import os

class AutoPump():
	def setServoPulse(self,channel, pulse):
		pulseLength = 1000000                   # 1,000,000 us per second
		pulseLength /= 60                       # 60 Hz
		pulseLength /= 4096                     # 12 bits of resolution
		pulse *= 1000
		pulse /= pulseLength
		pwm.setPWM(channel, 0, pulse)

	def RunMotor(self,conn, breathTime, servoMin, servoMax):
		#Initialise the PWM device using the default address
		pwm = PWM(0x40) #for debug: pwm = PWM(0x40, debug=True)

		logging.info('Initializing servo motion')

		breathcounter = 0
		while(True):
			pwm.setPWM(0, 0, servoMin)
			time.sleep(breathTime)
			pwm.setPWM(0, 0, servoMax)
			time.sleep(breathTime)
			breathcounter += 1

			#send breath counter update
			conn.send(breathcounter)
		conn.close()

	def processVision(self, threshold=30, heightToMl=3, saveImages=False, imagesDir="/tmp/"):

		logging.info('Initializing machine vision')
		#take a picture 
		#this goes to disk because the uv4l direct to SCV solution doesn't work on jessie: http://www.linux-projects.org/modules/sections/index.php?op=viewarticle&artid=14
		sbp.call("raspistill -o image.bmp", shell=True)
		img = scv.Image('image.bmp')

		logging.info('Image acquired.  Processing machine vision')

		#find the ball
		red_channel = img.splitChannels()[0]
		blobs = red_channel.findBlobs(threshold)
		if blobs and len(blobs)>0:
			ballheight = blobs[0].y
		else:
			logging.warning('Machine vision failure: ball not detected')
			ballheight = -1 
		#compute the mls 
		mls = ballheight * heightToMl

		#if save is activated
		if saveImages:
			outputFile = imagesDir + '/{}.bmp'.format(datetime.datetime.now().strftime("%Y_%m_%d-%H-%M-%S"))
			logging.info('Saving image to {}'.format(outputFile))
			img.save(outputFile)

	def start(self):

		parent_conn, child_conn = Pipe()
		p = Process(target=self.RunMotor, args=(child_conn,self.breathTime,self.servoMin,self.servoMax))
		p.start()

		if self.saveImages:
			if not os.path.exists(self.imagesDir):
				os.makedirs(self.imagesDir)

		while (True):
			self.processVision(self.threshhold, self.heightToMl, self.saveImages, self.imagesDir)
			breathcount = parent_conn.recv()
			time.sleep(self.sampleRate)

		p.join()


	def __init__(self):
		### Configureable parameters ###
		self.outputFile = 'run_{}.csv'.format(datetime.datetime.now().strftime("%Y_%m_%d-%H-%M-%S"))

		# Breathing Motion
		self.servoMin = 375  # Min pulse length out of 4096
		self.servoMax = 325  # Max pulse length out of 4096
		self.breathTime = 0.25 # Time in seconds between inhale/exhale

		# Machine Vision
		self.sampleRate = 60 #how long to wait in seconds between 
		self.threshhold = 30 #threshold for ball detection 
		self.heightToMl = 10 #vertical axis pixel height to mL conversion factor
		self.saveImages = False
		self.imagesDir = 'imageoutput'

		if not os.path.exists('/var/log/autopump'):
			os.makedirs('/var/log/autopump')
		logging.basicConfig(filename='/var/log/autopump/autopump.log',level=logging.INFO)



	# if __name__ == '__main__':
		
	# 	if not os.path.exists('/var/log/autopump'):
	# 		os.makedirs('/var/log/autopump')
	# 	logging.basicConfig(filename='/var/log/autopump/autopump.log',level=logging.INFO)

	# 	parent_conn, child_conn = Pipe()
	# 	p = Process(target=RunMotor, args=(child_conn,breathTime,servoMin,servoMax))
	# 	p.start()

	# 	if saveImages:
	# 		if not os.path.exists(imagesDir):
	# 			os.makedirs(imagesDir)

	# 	while (True):
	# 		processVision(threshhold, heightToMl, saveImages, imagesDir)
	# 		breathcount = parent_conn.recv()
	# 		time.sleep(sampleRate)

			
	# 	p.join()


