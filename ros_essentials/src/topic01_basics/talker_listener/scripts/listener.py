import rospy
from std_msgs.msg import String

def chatter_callback(message):
    rospy.loginto(rospy.get_caller_id() + "I heard %s", message.data)
    # Outputs into termial

    # print("I heard %s", message.data)

def listener():
    rospy.init_node('listener', anonymous = True)

    rospy.Subscriber("chatter", String, chatter_callback)
    # Creates subscriber object
    # chatter_callback is the callback function

    rospy.spin()
    # Start listening

if __name__ == '__main__':
    listener()
