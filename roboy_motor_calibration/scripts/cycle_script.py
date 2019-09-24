import rospy
import roboy_middleware_msgs.msg

rospy.init_node("cycle_script")

pos = 0

def MotorStatus(msg):
    global pos
    pos = msg.position[11]
    # rospy.loginfo_throttle(1,pos)

pub = rospy.Publisher('roboy/middleware/MotorCommand', roboy_middleware_msgs.msg.MotorCommand, queue_size=1)
rospy.Subscriber("roboy/middleware/MotorStatus", roboy_middleware_msgs.msg.MotorStatus, MotorStatus)
up = True

cycle = 0

while not rospy.is_shutdown():
    msg = roboy_middleware_msgs.msg.MotorCommand()
    msg.id = 3
    msg.motors = [11]
    if pos>518145 and up:
        up = False
    if pos<100000 and not up:
        rospy.loginfo_throttle(1,"cycle %d"%(cycle))
        cycle = cycle + 1
        up = True
    if up:
        msg.set_points = [1000]
    else:
        msg.set_points = [20]
    pub.publish(msg)
