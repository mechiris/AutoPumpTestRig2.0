from multiprocessing import Process, Pipe
from Adafruit_PWM_Servo_Driver import PWM
import datetime, time
import subprocess as sbp
import SimpleCV as scv

### Configureable parameters ###
outputFile = 'run_{}.csv'.format(datetime.datetime.now().strftime("%Y_%m_%d-%H-%M-%S"))

# Breathing Motion
servoMin = 200  # Min pulse length out of 4096
servoMax = 300  # Max pulse length out of 4096
breathTime = 0.5 # Time in seconds to take a breath

# Machine Vision
sampleRate = 60 #how long to wait in seconds between 
threshhold = 30 #threshold for ball detection 
heightToMl = 10 #vertical axis pixel height to mL conversion factor
saveImages = False
imagesDir = 'imageoutput'

def setServoPulse(channel, pulse):
	pulseLength = 1000000                   # 1,000,000 us per second
	pulseLength /= 60                       # 60 Hz
	print "%d us per period" % pulseLength
	pulseLength /= 4096                     # 12 bits of resolution
	print "%d us per bit" % pulseLength
	pulse *= 1000
	pulse /= pulseLength
	pwm.setPWM(channel, 0, pulse)

def RunMotor(conn, breathTime, servoMin, servoMax):
	#Initialise the PWM device using the default address
	pwm = PWM(0x40) #for debug: pwm = PWM(0x40, debug=True)


	breathcounter = 0
	while(True):
		pwm.setPWM(0, 0, servoMin)
		time.sleep(waitTime)
		pwm.setPWM(0, 0, servoMax)
		time.sleep(waitTime)
		breathcounter += 1

		#send breath counter update
		conn.send(breathcounter)
	conn.close()

def processVision(threshold=30, heightToMl, saveImages, imagesDir):
	#take a picture 
	#this goes to disk because the uv4l direct to SCV solution doesn't work on jessie: http://www.linux-projects.org/modules/sections/index.php?op=viewarticle&artid=14
	sbp.call(“raspistill -n -t 0 -o image.bmp, shell=True)
	img = Image(“image.bmp”)

	#find the ball
	red_channel = img.splitChannels[0]
	blobs = red_chanel.findBlobs(threshold)
	ballheight = blobs[0].y

	#compute the mls 
	mls = ballheight * heightToMl

	#if save is activated 

if __name__ == '__main__':
	parent_conn, child_conn = Pipe()
	p = Process(target=RunMotor, args=(child_conn,breathTime,servoMin,ServoMax))
	p.start()

	if saveImages:
		if not os.path.exists(imagesDir):
			os.makedirs(imagesDir)

	while (True):
		processVision(threshhold, heightToMl, saveImages, imagesDir)
		breathcount = parent_conn.recv()
		
	p.join()


