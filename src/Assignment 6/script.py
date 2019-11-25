import rospy
import time
import math
import sys
from nav_msgs.msg import Odometry
from autominy_msgs.msg import SpeedCommand, NormalizedSteeringCommand, SteeringFeedback, Tick
from std_msgs.msg import Float32


accTickDistance = 0.0
accTick = 0.0
gpsDistance = 0.0
positionX = 0.0
positionY = 0.0
positions = []
speed = 0.0



def gpsCall(odometry_data):
    global positionX, positionY
    positionX = odometry_data.pose.pose.position.x
    positionY = odometry_data.pose.pose.position.y
    # rospy.loginfo("Position x: "+str(positionX)+" , Position y: "+str(positionY))
    positions.append((positionX,positionY))


def tickCall(tick):
    global accTick
    accTick = tick.value 
    # rospy.loginfo("Tick: "+str(accTick) )


def driveForwardDistance():
    global positions, accTickDistance,  gpsDistance
    i=1

    steering_cmd = NormalizedSteeringCommand()
    steering_cmd.value = 0.0 #left 1.0 , right -1.0 
    pub_steering.publish(steering_cmd)
    speed = SpeedCommand()
    speed.value = 0.2 #moeglichst langsam
    pub_speed.publish(speed)
    time.sleep(4)

    while i< len(positions):
        gpsDistance += math.sqrt(math.pow(positions[i][0]-positions[i-1][0],2)+math.pow(positions[i][1]-positions[i-1][1],2))
	accTickDistance += accTick
	
	# print(gpsDistance)
	i= i+1

    rospy.loginfo("cumulativ gpsDistance (cm): "+str(gpsDistance))
    rospy.loginfo("last - first position gps distance (cm) : "+str( math.sqrt(math.pow(positions[len(positions)-1][0]-positions[0][0],2)+math.pow(positions[len(positions)-1][1]-positions[0][1],2))))
    rospy.loginfo("Ticks: "+str(accTickDistance))
    rospy.loginfo("Distance per Tick (cumulativ gpsDistance (cm)/ ticks): "+str(gpsDistance/accTickDistance))
    speed.value = 0.0
    pub_speed.publish(speed)

def driveCircleDistance(direction):
    global positions, accTickDistance,  gpsDistance
    i=1

    steering_cmd = NormalizedSteeringCommand()

    if(direction == "left"):
	steering_cmd.value = 1.0
    elif(direction == "right"):
	steering_cmd.value = -1.0
    else:
	steering_cmd.value = 0.0

    pub_steering.publish(steering_cmd)

    speed = SpeedCommand()
    speed.value = 0.2 #moeglichst langsam
    pub_speed.publish(speed)
    time.sleep(4)

    while i< len(positions):
        gpsDistance += math.sqrt(math.pow(positions[i][0]-positions[i-1][0],2)+math.pow(positions[i][1]-positions[i-1][1],2))
	accTickDistance += accTick
	
	# print(gpsDistance)
	i= i+1

    rospy.loginfo("cumulativ gpsDistance (cm): "+str(gpsDistance))
    rospy.loginfo("last - first position gps distance (cm) : "+str( math.sqrt(math.pow(positions[len(positions)-1][0]-positions[0][0],2)+math.pow(positions[len(positions)-1][1]-positions[0][1],2))))
    rospy.loginfo("Ticks : "+str(accTickDistance))
    rospy.loginfo("Distance per Tick (cumulativ gpsDistance (cm)/ ticks): "+str(gpsDistance/accTickDistance))
    speed.value = 0.0
    pub_speed.publish(speed)


def clearSystem():
    accTickDistance = 0.0
    accTick = 0.0
    gpsDistance = 0.0
    positionX = 0.0
    positionY = 0.0
    positions = []

pub_speed = rospy.Publisher("/actuators/speed", SpeedCommand, queue_size=80)
pub_steering = rospy.Publisher("/actuators/steering_normalized", NormalizedSteeringCommand, queue_size=80)
pub_steeringAngle = rospy.Publisher("/kreise", Float32, queue_size=80)
pub_tick = rospy.Publisher("/sensors/arduino/ticks", Tick, queue_size=80)
sub_GPS = rospy.Subscriber("/communication/127/localization", Odometry, gpsCall, queue_size=5)

rospy.init_node("kreise")
rospy.Subscriber("/sensors/arduino/ticks",Tick,tickCall)


def main():

    driveForwardDistance()     
    #driveCircleDistance("left")
    #driveCircleDistance("right")
    clearSystem()
    sys.exit()





if __name__ == '__main__':
    main()


rospy.spin()
