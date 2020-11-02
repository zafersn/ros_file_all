#!/usr/bin/env python
# license removed for brevity

'''
aracin
h_left_wheel yonunde donmesi	-	
h_right_wheel yonunde donmesi 	+ yonde twist datasi gonderir.

ayni sekilde aracin geri gitmesi -
ileri gitmesi + yonde twist datasi dondurur.
'''
import rospy
from std_msgs.msg import String
from geometry_msgs.msg  import Twist
from sensor_msgs.msg import Joy

import serial

class Pi_CAR_JOYSTICK(object):
	def __init__(self):
		self.pub = rospy.Publisher('joy', Joy, queue_size=1)
		#self.closestP=rospy.Publisher("/closest_point", PointStamped, queue_size=1)
		self.ser = serial.Serial('/dev/ttyUSB0', 115200)
		self.mylist=[]
		self.vel_msg=Twist()
		self.joy_msg=Joy()

		self.maxLnrPWM=100		#lnr=Linear	maxLinearPWM degeri
		self.minLnrPWM=-100
		self.oldLnrMin=-1
		self.oldLnrMax=1

		self.maxAngPWM=100		#Ang=Angular	maxAngularPWM degeri
		self.minAngPWM=-100
		self.oldAngMin=-2.2
		self.oldAngMax=2.2
		self.oldTimeN=0
		self.oldTimeS=0

		self.countCalbrData=0
		self.arrycount=0
		self.goData=[]
		self.joyData1=[]
		self.joyLeft_x=0
		self.joyLeft_y=0
		self.joyRight_x=0	
		self.joyRight_y=0
		self.returnData=[];
		self.isCalb=True
		self.isFirst=True
	
	def calculatePWM(self,x,in_min,in_max,out_min,out_max):
		return  (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min
		 
	def SerialRead(self):

	
		read=self.ser.readline()
		#print(read )

		indexStart=read.find("!", 0,1)
		indexEnd=read.find("!",14,len(read))
		if (indexStart is not -1 and indexEnd is not -1):
			getData=read.split(':')
			#print(getData)
			joyL_X=0
			joyL_Y=0
			joyR_X=0
			joyR_Y=0
			'''Joystick Axes  '''
			if(self.isFirst == True):
				self.joyLeft_x=self.calibreJOY(float(getData[0][1:]))
				self.joyLeft_y=self.calibreJOY(float(getData[1]))
				self.joyRight_x=self.calibreJOY(float(getData[2]))
				self.joyRight_y=self.calibreJOY(float(getData[3]))
				#print("True")
			
			elif(self.isFirst == False):
				'''	
				LX=self.calibreJOY(float(getData[0][1:]))
				LY=self.calibreJOY(float(getData[1]))
				RX=self.calibreJOY(float(getData[2]))
				RY=self.calibreJOY(float(getData[3]))
				print (" s: ")
				print("%s %s %s %s" %(LX,LY,RX,RY))

				'''
				
				

				LX=float(getData[0][1:])
				LY=float(getData[1])
				RX=float(getData[2])
				RY=float(getData[3])
			#	print( " d: ")
			#	print("%f %f %f %f" %(self.joyLeft_x[0],self.joyLeft_y[1],self.joyRight_x[2],self.joyRight_y[3]))
				
				
				joyL_X=self.joyLeft_x-LX
				if(joyL_X<100 and joyL_X>-100):
					joyL_X=0					

				else:

					joyL_X=self.calculatePWM(joyL_X,-4050,2515,-15000,15000)
					
				joyL_Y=self.joyLeft_y-LY
				if(joyL_Y<100 and joyL_Y>-100):
					joyL_Y=0
				else:
					joyL_Y=self.calculatePWM(joyL_Y,-3050,3356,-15000,15000)

				print("  %s          %s  " %(joyL_X,joyL_Y))
				#joyL_X=self.calculatePWM(6000,-6000,2515,-4050,joyL_X)
				#joyL_Y=self.calculatePWM(6000,-6000,3356,-3050,joyL_Y)
			



				joyR_X=self.joyRight_x-RX
				joyR_Y=self.joyRight_y-RY
				#print("False")

			'''Joystick Button'''
			joyLeft_btn=getData[4]
			joyRight_btn=getData[5]
				
			#print("%s %s %s %s" %(joyL_X,joyL_Y,joyR_X,joyR_Y))
			#print("2222:   %s          %s  " %(joyL_X,joyL_Y))
			#self.joy_msg.axes=float(joyLeft_x[1:])/10000,float(joyLeft_y)/10000,float(joyRight_x)/10000,float(joyRight_y)/10000
			self.joy_msg.axes=joyL_X/10000,joyL_Y/10000
			#self.joy_msg.axes=-0.0038758506998419762, -0.0038453321903944016, -0.0, -0.999969482421875, 0.0, 0.0
			self.joy_msg.buttons=float(joyLeft_btn),float(joyRight_btn)
	 		self.pub.publish(self.joy_msg)

		
	def calibreJOY(self,rawData):
			
				
			if(self.countCalbrData==21):
				#print(self.joyData1[self.arrycount])
				self.joyData1[self.arrycount].sort()
				self.goData.insert(self.arrycount,[])
				#print(self.joyData1[self.arrycount])
				#print(" :  ")
				#print(self.arrycount)
				for x in range(19):
				#	print(x)
					self.goData[self.arrycount].insert(x,self.joyData1[self.arrycount][x+1])
				#print (self.goData[self.arrycount])
				self.returnData .insert(self.arrycount, reduce(lambda x,y: x+y, self.goData[self.arrycount])/19)
				#print (self.returnData[self.arrycount])
				self.arrycount=self.arrycount+1
				if (self.arrycount==4):
					self.countCalbrData=0
					for x in range (4):
						self.joyData1[x]=[]
						self.goData[x]=[]
					self.isFirst=False
					self.isCalb=True
				#	print (self.isFirst)


					self.arrycount=0
					self.isCalb=False	
				#	print ("************************************")
					return self.returnData[3]
				#print (self.returnData)
				#print ("************************************")
				
				return self.returnData[self.arrycount-1]
			
			if (self.isCalb):
				self.joyData1.insert(self.arrycount,[])
			self.joyData1[self.arrycount].insert(self.countCalbrData,rawData)
			#print(self.arrycount)
			#print(self.countCalbrData)
			self.arrycount=self.arrycount+1
		
			if (self.arrycount==4):
					self.countCalbrData=self.countCalbrData+1
					self.arrycount=0
					self.isCalb=False
			if (self.isFirst ==False):
				if(self.arrycount==0):
					return self.returnData[3];
				else:
					return self.returnData[self.arrycount-1];
			else:
				return 0
			
			#return self.returnData
	
	def clearTwist(self,newTimeN,newTimeS):
		if ((newTimeN)-(self.oldTimeN)>90000000)or((newTimeS)-(self.oldTimeS)>1):
			self.vel_msg.linear.x=0
			self.vel_msg.linear.y=0
			self.vel_msg.linear.z=0

			self.vel_msg.angular.x = 0
			self.vel_msg.angular.y = 0
			self.vel_msg.angular.z = 0  
		   


		
if __name__ == '__main__':
    try:
	rospy.init_node("pi_car_joy_board")
	r=rospy.Rate(50)
	lr=Pi_CAR_JOYSTICK()
	isOp=lr.ser.isOpen()
	if isOp:
		print("serialOpen")

	while not rospy.is_shutdown():
		lr.SerialRead()
		r.sleep()
        
    except rospy.ROSInterruptException:
        pass 


