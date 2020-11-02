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
import serial

class stm32CP(object):
	def __init__(self):
		self.blink=rospy.Subscriber("/jackal_velocity_controller/cmd_vel", Twist, self.blink_callback)
		#self.closestP=rospy.Publisher("/closest_point", PointStamped, queue_size=1)
		self.ser = serial.Serial('/dev/ttyUSB0',57600)
		self.mylist=[]
		self.vel_msg=Twist()

		self.maxLnrPWM=100		#lnr=Linear	maxLinearPWM degeri
		self.minLnrPWM=-100
		self.oldLnrMin=-1
		self.oldLnrMax=1

		self.maxAngPWM=100		#Ang=Angular	maxAngularPWM degeri
		self.minAngPWM=-100
		self.oldAngMin=-2.2
		self.oldAngMax=2.2
		self.oldTimeN=0;
		self.oldTimeS=0;


	def blink_callback (self, msg):
		self.vel_msg.linear.x=msg.linear.x
		self.vel_msg.linear.y=msg.linear.y
		self.vel_msg.linear.z=msg.linear.z

		self.vel_msg.angular.x = msg.angular.x
        	self.vel_msg.angular.y = msg.angular.y
	        self.vel_msg.angular.z = msg.angular.z
		
		#now = rospy.Time.now().tosec()
		self.oldTimeN = rospy.get_rostime().nsecs
		self.oldTimeS = rospy.get_rostime().secs
		#seconds = t.to_sec() #floating point
		#rospy.loginfo("Current time %i", self.oldTime)
		#rospy.loginfo("Current time %i %i", self.oldTime.secs, self.oldTime.nsecs)

		#self.ser.write(str(msg.data))

	def calculatePWM(self,NewMax,NewMin,OldMax,OldMin,OldValue):
		NewValue = (((OldValue - OldMin) * (NewMax - NewMin)) / (OldMax - OldMin)) + NewMin
		return NewValue
			
	def SerialWrite(self):

		#rospy.loginfo(rospy.get_caller_id() + "I heard %s %s", self.vel_msg.linear.x, self.vel_msg.angular.z)
		pwmLnr=self.calculatePWM(self.maxLnrPWM,self.minLnrPWM,self.oldLnrMax,self.oldLnrMin,self.vel_msg.linear.x)
		pwmAng=self.calculatePWM(self.maxAngPWM,self.minAngPWM,self.oldAngMax,self.oldAngMin,self.vel_msg.angular.z)
   		#rospy.loginfo(rospy.get_caller_id() + "I heard %s %s", pwmLnr,pwmAng)
		newTimeN=rospy.get_rostime().nsecs
		newTimeS=rospy.get_rostime().secs
		self.clearTwist(newTimeN,newTimeS)
		
		self.yonVer(int(pwmLnr),int(pwmAng))
		
		
	def yonVer(self,Lnr,Ang):
		h_left_pwm = 0 
		h_right_pwm = 0
		if(Lnr>0 ):
		    h_left_pwm  = Lnr
		    h_right_pwm = Lnr
		  
		    if(Ang<0):
		   	h_left_pwm-=(Ang*-1)
		    elif(Ang>0):
			h_right_pwm-=(Ang)

		elif(Lnr<0 ):
		    h_left_pwm  = Lnr
		    h_right_pwm = Lnr
		  
		    if(Ang<0):
		   	h_left_pwm+=(Ang*-1)
		    elif(Ang>0):
			h_right_pwm+=(Ang)
		elif(Lnr == 0):
			if(Ang<0):
		   		h_left_pwm-=(Ang*-1)
				h_right_pwm+=(Ang*-1)
			elif(Ang>0):
				h_left_pwm+=(Ang)
				h_right_pwm-=(Ang)
		if(h_left_pwm>=0):
			a=("+%03d"%h_left_pwm)
		if(h_right_pwm>=0):
			b=("+%03d"%h_right_pwm)
		if(h_left_pwm<0):
			a=("%04d"%h_left_pwm)
		if(h_right_pwm<0):
			b=("%04d"%h_right_pwm)
		mData="#"+str(a)+":"+str(b)+"!\0"	
		rospy.loginfo(rospy.get_caller_id() + "I heard %s", mData)
		self.ser.write(str(mData))
		
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
	rospy.init_node("stm32_c_p")
	r=rospy.Rate(50)
	lr=stm32CP()
	isOp=lr.ser.isOpen()
	if isOp:
		print("serialOpen")

	while not rospy.is_shutdown():
		lr.SerialWrite()
		r.sleep()
        
    except rospy.ROSInterruptException:
        pass


