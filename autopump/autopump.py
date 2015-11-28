from multiprocessing import Process, Pipe, Value
from Adafruit_PWM_Servo_Driver import PWM
import RPi.GPIO as GPIO
import datetime, time
import numpy as np
import subprocess as sbp
import SimpleCV as scv
import logging
import os
import glob
import json
import csv
import copy 

class AutoPump():
	def setServoPulse(self,channel, pulse):
		pulseLength = 1000000                   # 1,000,000 us per second
		pulseLength /= 60                       # 60 Hz
		pulseLength /= 4096                     # 12 bits of resolution
		pulse *= 1000
		pulse /= pulseLength
		pwm.setPWM(channel, 0, pulse)

	def RunMotor(self,counter, runMotor):
		#Initialise the PWM device using the default address
		pwm = PWM(0x40) #for debug: pwm = PWM(0x40, debug=True)

		logging.info('Initializing servo motion')
                with counter.get_lock():
                        counter.value =0
		while(runMotor.value):
			pwm.setPWM(0, 0, self.servoMin)
			time.sleep(self.breathTime)
			pwm.setPWM(0, 0, self.servoMax)
			time.sleep(self.breathTime)
                        with counter.get_lock():
                                counter.value += 1
                logging.info('Stopping motor')
		pwm.setPWM(0,0,self.servoSlack)


	def measureFluidLevel(self):
		logging.info('Capturing image')
		self.captureImage()
		time.sleep(15) #give image time to write to disk to avoid race condition.  This is sloppy
		logging.info('Processing vision')
		self.processVision()
		#if save is activated
		if self.saveImages:
			outputFile = imagesDir + '/{}.jpg'.format(datetime.datetime.now().strftime("%Y_%m_%d-%H-%M-%S"))
			logging.info('Saving image to {}'.format(outputFile))
			self.img.save(outputFile)


	def captureImage(self):
		logging.info('Initializing machine vision')
		#take a picture 
		#this goes to disk because the uv4l direct to SCV solution doesn't work on jessie: http://www.linux-projects.org/modules/sections/index.php?op=viewarticle&artid=14
		sbp.call("raspistill -o /tmp/image.jpg", shell=True)
		logging.info('Image acquired')


	def processVision(self, imgfile='/tmp/image.jpg'):
		logging.info('Processing machine vision')
		self.img = scv.Image(imgfile)
		self.img = self.img.rotate270()
		cylinder = self.img.crop(self.cylinderROI) #extents for image 
		bluechan = cylinder.getNumpy()[:,:,0]
		bluechan = bluechan.mean(axis=1)
		smoothb = self.smooth(bluechan,window_len=50)
		try:
			ballheight = np.nonzero(np.diff(smoothb) < self.threshold)[0][0] #get first y value that exceeds thrshold 
		except:
			logging.warning('Machine vision failure: ball not detected')
			ballheight = -1
		
		mls = np.polyval(np.array(self.heightToMl),ballheight)
                self.ballHeight = ballheight
                self.mls = mls
                return [ballheight,mls]


	def smooth(self, x,window_len=11,window='hanning'):
	        if x.ndim != 1:
	                raise ValueError, "smooth only accepts 1 dimension arrays."
	        if x.size < window_len:
	                raise ValueError, "Input vector needs to be bigger than window size."
	        if window_len<3:
	                return x
	        if not window in ['flat', 'hanning', 'hamming', 'bartlett', 'blackman']:
	                raise ValueError, "Window is on of 'flat', 'hanning', 'hamming', 'bartlett', 'blackman'"
	        s=np.r_[2*x[0]-x[window_len-1::-1],x,2*x[-1]-x[-1:-window_len:-1]]
	        if window == 'flat': #moving average
	                w=numpy.ones(window_len,'d')
	        else:  
	                w=eval('np.'+window+'(window_len)')
	        y=np.convolve(w/w.sum(),s,mode='same')
	        return y[window_len:-window_len+1]

    def voidPumpingChamber(self):
        GPIO.output(self.returnMotorPin,GPIO.HIGH)
        time.sleep(self.returnPumpTime)
        GOIP.output(self.returnMotorPin,GPIO.LOW)


	def start(self):

		self.parent_conn, self.child_conn = Pipe()
		p = Process(target=self.RunMotor, args=(self.breathCounter,self.motor))
		p.start()

		if self.saveImages:
			if not os.path.exists(self.imagesDir):
				os.makedirs(self.imagesDir)

		while self.runCounter != 0:
			self.processSingleRun()

			# reset for the next run
			self.breathCounter = Value('d', -1)
			self.initializeRunVariables()

			if self.runCounter > 0:
				self.runCounter -= 1


		self.motor.value = 0
		p.join()
        print('Run: {} completed'.format(self.outputFile))


    def processSingleRun(self):
    	while (True):
			self.measureFluidLevel()

			time.sleep(self.sampleRate)
			logging.info('BreathCounter: {}, BallHeight: {}, mls: {}'.format(self.breathCounter.value,self.ballHeight, self.mls))
			print('BreathCounter:{}, BallHeight: {}, mls: {}'.format(self.breathCounter.value, self.ballHeight, self.mls))
			self.saveData()
			if self.mls > self.maxFluidLevel:
				logging.info('Fluid level {} exceeds max fluid level of {}.  Shutting down.'.format(self.mls, self.maxFluidLevel))
				break
        logging.info('Run: {} completed'.format(self.outputFile))


	def saveData(self):
		if not os.path.isfile(self.outputFile):
			with open(self.outputFile, 'wb') as csvfile:
				csvwriter = csv.writer(csvfile, delimiter=' ')
				csvwriter.writerow(['breathCounter','breathTimeHumanHours','ballHeight','mlsTotal','mlsPumped','timestamp'])
				self.saveConfig()
                                self.baseFluidLevel = self.mls

		breathTimeHumanHours = self.breathCounter.value / ( self.humanBPM * 60 ) #breathing time in hours

		with open(self.outputFile, 'a') as csvfile:
			csvwriter = csv.writer(csvfile, delimiter=' ')
			csvwriter.writerow([self.breathCounter.value,breathTimeHumanHours,self.ballHeight, self.mls, self.mls-self.baseFluidLevel, datetime.datetime.now().strftime("%Y_%m_%d-%H-%M-%S")])


	def generateCalibrationValues(self,imgdir):
	    #returns calibration curve from an image directory.  Assumes imgs are named XXml.jpg where XX is the volumne in ml
	    imgs = glob.glob(imgdir + '/*.*')
	    vols = list()
	    out = list()
	    for imgfile in imgs:
	        self.img = scv.Image(imgfile)
	        out.append(self.processVision())
	        vols.append(imgfile.split('ml')[0].split('/')[-1])
	    out = np.array(out)
	    return [out,np.array(vols).astype('int64')]

	def generateCalibrationFit(self,imgdir):
		[xvals,yvals] = self.generateCalibrationValues(imgdir)
		fit = np.polyfit(xvals[:,0],yvals,1)
		logging.info('Fit performed: {}'.format(fit))
		return fit

	def saveConfig(self):
            outjson = copy.copy(self.__dict__)
            #prune non-native types from configuration json dump
            badkeys = ['motor','parent_conn','child_conn','img']
            for var in badkeys:
                del outjson[var]
            outjson['breathCounter'] = outjson['breathCounter'].value
            with open(self.configFile, 'w') as f:
                json.dump(outjson,f)

    def runReturnPump(self):
    	logging.info('Running return motor for {} seconds'.format(self.returnPumpTime))
		GPIO.output(self.returnMotorPin,GPIO.HIGH)
		time.sleep(self.returnPumpTime)
		GPIO.output(self.returnMotorPin,GPIO.LOW)
		logging.info('Return motor complete')



    def initializeRunVariables(self):
    	#
    	curtime = datetime.datetime.now().strftime("%Y_%m_%d-%H-%M-%S")
		self.outputFile = 'run_{}.csv'.format(curtime)
		self.configFile = 'run_{}.cfg'.format(curtime)

		### init holding variables
		self.breathCounter = Value('d', -1)
		self.ballHeight = -1
		self.mls = -1
		self.motor = Value('d',1)
		self.runCounter = -1

	def __init__(self):
		### Configureable parameters ###
		# Breathing Motion
		self.servoMin = 275  # Min pulse length out of 4096
		self.servoMax = 325  # Max pulse length out of 4096
		self.servoSlack = 400
		self.breathTime = 0.25 # Time in seconds between inhale/exhale

		# Machine Vision
		self.sampleRate = 10 # How long to wait in seconds between 
		self.threshold = -0.25 # Threshold for ball detection 
		self.heightToMl = list([ -2.56861600e-01,   4.55618036e+02]) # Vertical axis pixel height to mL conversion factor
		self.saveImages = False
		self.cylinderROI = [150,850,350,2000]
		self.imagesDir = 'imageoutput'
		self.maxFluidLevel = 450 # If we exceed this in MLs, shut things down.

		# Breath Normalization Parameters
		self.humanBPM = 15 # For doing testrig to in-vivo calculation

		# Return motor
		self.returnMotorPin = 17 # Raspberry pi pin to trigger mosfet relay to run the return pump
		self.returnPumpTime = 60 # Time in seconds to run the return motor to void the cylinder

		#### Initialize system
		GPIO.setmode(GPIO.BOARD)
		GPIO.setup(returnMotorPin, GPIO.OUT)
		self.initializeRunVariables()

		if not os.path.exists('/var/log/autopump'):
			os.makedirs('/var/log/autopump')
		logging.basicConfig(filename='/var/log/autopump/autopump.log',level=logging.INFO)

