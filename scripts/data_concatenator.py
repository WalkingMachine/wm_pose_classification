

from sara_msgs.msg import Poses
import numpy as np
from std_msgs.msg import UInt8MultiArray
import rospy

#human_pose = open('HumanPoses',mode='w')
#human_writer=csv.writer(human_pose,delimiter=',')
label=[0,0]
concat = np.zeros([17, 5, 0])

def callbackPose(msg):

    global label
    global concat
    for pose in msg.poses:
        l = np.zeros([17, 5,1])
        for part in pose.parts:
            l[part.id,0]=part.position.x
            l[part.id,1]=part.position.y
            l[part.id,2]=part.position.z
            l[part.id, 3] = int(label[0])
            l[part.id, 4] = int(label[1])
        concat = np.concatenate((concat,l),axis=2)
        #print(concat)
    np.save('HumanPoses',concat)

def callbackLabel(msg):
    global label
    label=msg.data
    print(label)

if __name__=='__main__':
    rospy.init_node('converter', anonymous=True)
    topic1 = rospy.Subscriber('/pose_detection/poses', Poses, callbackPose)
    topic2 = rospy.Subscriber('/pose_detection/label', UInt8MultiArray, callbackLabel)
    rospy.spin()

