#!/usr/bin/env python
# Software License Agreement (BSD License)
#
# Copyright (c) 2008, Willow Garage, Inc.
# All rights reserved.
#
# Redistribution and use in source and binary forms, with or without
# modification, are permitted provided that the following conditions
# are met:
#
#  * Redistributions of source code must retain the above copyright
#    notice, this list of conditions and the following disclaimer.
#  * Redistributions in binary form must reproduce the above
#    copyright notice, this list of conditions and the following
#    disclaimer in the documentation and/or other materials provided
#    with the distribution.
#  * Neither the name of Willow Garage, Inc. nor the names of its
#    contributors may be used to endorse or promote products derived
#    from this software without specific prior written permission.
#
# THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
# "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
# LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
# FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
# COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
# INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
# BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
# LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
# CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
# LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
# ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
# POSSIBILITY OF SUCH DAMAGE.
#
# Revision $Id$

## Simple talker demo that published std_msgs/Strings messages
## to the 'chatter' topic


# Nombre de points : 17 [0-16]
            # 0 : nez
            # 1 : oeil gauche
            # 2 : oeil droit
            # 3 : oreille gauche
            # 4 : oreille droite
            # 5 : epaule gauche
            # 6 : epaule droite
            # 7 : coude gauche
            # 8 : coude droit
            # 9 : main gauche
            # 10 : main droite
            # 11 : hanche gauche
            # 12 : hanche droite
            # 13 : genoux gauche
            # 14 : genoux droit
            # 15 : pied gauche
            # 16 : pied droit


import rospy
from sara_msgs.msg import Pose
import math

def getAngleBetweenThreePoints(a, b, c):
    if a == None or b == None or c == None:
        return 999
    else:
        u = [c.x - b.x, c.y - b.y, c.y - b.y]
        v = [a.x - a.x, a.y - a.y, a.y - a.y]
        scal = u[0]*v[0] + u[1]*v[1] + u[2]*v[2]
        magnitude_u = math.sqrt(u[0]*u[0] + u[1]*u[1] + u[2]*u[2])
        magnitude_v = math.sqrt(v[0]*v[0] + v[1]*v[1] + v[2]*v[2])
        return math.acos( scal / ( magnitude_u * magnitude_v ) )

def callback(data):
    rospy.loginfo(rospy.get_caller_id() + 'I heard %s', data)

    #data.poses.parts[x].id/position

    #fill the dictionnary
    points = {}
    for point in data.poses.parts:
        points[str(point.id)] = point.position

    data.poses.right_arm_up = False
    data.poses.right_arm_down = False
    data.poses.right_arm_point = False
    data.poses.left_arm_up = False
    data.poses.left_arm_down = False
    data.poses.left_arm_point = False

    ## RIGHT ARM UP
    if points.has_key('10'):  # main droite
        if points.has_key('1'):  # oeil gauche
            if points.get('10').z > points.get('1').z:
                data.poses.right_arm_up = True
        if points.has_key('2'):  # oeil droit
            if points.get('10').z > points.get('2').z:
                data.poses.right_arm_up = True
        if points.has_key('0'):  # nez
            if points.get('10').z > points.get('0').z:
                data.poses.right_arm_up = True
        if points.has_key('3'):  # oreille gauche
            if points.get('10').z > points.get('3').z:
                data.poses.right_arm_up = True
        if points.has_key('4'):  # oreille droite
            if points.get('10').z > points.get('4').z:
                data.poses.right_arm_up = True

    ## LEFT ARM UP
    if points.has_key('9'):  # main gauche
        if points.has_key('1'):  # oeil gauche
            if points.get('9').z > points.get('1').z:
                data.poses.left_arm_up = True
        if points.has_key('2'):  # oeil droit
            if points.get('9').z > points.get('2').z:
                data.poses.left_arm_up = True
        if points.has_key('0'):  # nez
            if points.get('9').z > points.get('0').z:
                data.poses.left_arm_up = True
        if points.has_key('3'):  # oreille gauche
            if points.get('9').z > points.get('3').z:
                data.poses.left_arm_up = True
        if points.has_key('4'):  # oreille droite
            if points.get('9').z > points.get('4').z:
                data.poses.left_arm_up = True

    ## LEFT ARM POINT
    data.poses.left_arm_point = False
    pointLeft = False
    if points.has_key('5') and points.has_key('7') and points.has_key('9'):  # epaule coude et main gauche
        if getAngleBetweenThreePoints(points.get('5').position, points.get('7').position, points.get('9').position) > 2.0944:  # 120 degres
            data.poses.left_arm_point = True
            pointLeft = True

    ## RIGHT ARM POINT
    data.poses.right_arm_point = False
    pointRight = False
    if points.has_key('6') and points.has_key('8') and points.has_key('10'):  # epaule coude et main droite
        if getAngleBetweenThreePoints(points.get('6').position, points.get('8').position, points.get('10').position) > 2.0944:  # 120 degres
            data.poses.right_arm_point = True
            pointRight = True

    ## LEFT ARM DOWN
    data.poses.left_arm_down = False
    if points.has_key('5') and points.has_key('7') and points.has_key('11'):  # hanche epaule et coude gauche
        if getAngleBetweenThreePoints(points.get('11').position, points.get('5').position, points.get('7').position) < 0.5:  # 28 degres
            if pointLeft:
                data.poses.left_arm_down = True

    ## RIGHT ARM DOWN
    data.poses.right_arm_down = False
    if points.has_key('6') and points.has_key('8') and points.has_key('12'):  # hanche epaule et coude droit
        if getAngleBetweenThreePoints(points.get('12').position, points.get('6').position, points.get('8').position) < 0.5:  # 28 degres
            if pointRight:
                data.poses.right_arm_down = True

    pub.publish(data)


def pose_classification():
    #ecoute au topic de la pose classifiee
    rospy.init_node('pose_classification', anonymous=True)

    rospy.Subscriber('/pose_detection/poses', Pose, callback)
    #souscrit a un topic pour ecouter la pose detectee


    rospy.spin()
    #Pour continuer la boucle sans en sortir

if __name__ == '__main__':
    try:

        pub = rospy.Publisher('/pose_detection/classified', Pose, queue_size=10)
        pose_classification()
    except rospy.ROSInterruptException:
        pass
