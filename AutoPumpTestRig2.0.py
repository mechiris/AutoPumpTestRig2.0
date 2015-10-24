from multiprocessing import Process, Pipe
from Adafruit_PWM_Servo_Driver import PWM
import datetime, time



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

def processVision(threshold=30, heightToMl, saveImages):
	#take a picture

	#find the ball

	#compute 

if __name__ == '__main__':
	parent_conn, child_conn = Pipe()
	p = Process(target=RunMotor, args=(child_conn,breathTime,servoMin,ServoMax))
	p.start()

	while (True):
		processVision(threshhold, heightToMl, saveImages)
		breathcount = parent_conn.recv()
		
	p.join()


