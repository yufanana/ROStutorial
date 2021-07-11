import rospy
from std_msgs.msg import String
# std_msgs.msg is the package

def talker():
    pub = rospy.Publisher('chatter', String, queue_size=10)
    # Creates publisher object
    # 'chatter' is topic, String is topic type, queue_size is like a buffer/queue

    rospy.init_node('talker', anonymous = True)
    # Initialise rosnode
    # 'talker' is name of the node, anonymous = True ensures that nodes have unique names/ID

    rate = rospy.Rate(1) # in Hz

    i = 0       # counter
    while not rospy.is_shutdown():
        hello_str = "hello world %s" % i
        rospy.loginfo(hello_str)
        # Outputs into terminal
        pub.publish(hello_str)
        rate.sleep()        # sleep duration(s) = 1/rate
        i += 1
 
if __name__ == '__main__':
    try:
        talker()
    except rospy.ROSInterruptException:
        pass
