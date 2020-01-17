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
from sara_msgs.msg import Poses
import numpy as np
import sklearn as svm
from sklearn import datasets
from sklearn.model_selection import train_test_split
import math
import matplotlib.pyplot as plt
from sklearn.svm import SVC
from sklearn import preprocessing
from sklearn import utils
from mpl_toolkits import mplot3d

#def callback(data):
 #   rospy.loginfo(rospy.get_caller_id() + "I heard %s", data.data)

bankdata=np.load('HumanPoses.npy')[:,:,:]

#moyenne de chaque donne
def calculateZero(data):
    dataBase=np.zeros([17,3,49])
    for d in range(data.shape[2]):
    #moyenne des points pas egales a zero, met dans la liste
        zero=np.sum(data[:,:,d],axis=0)/np.count_nonzero(data[:,:,d])
        for coor in range(data.shape[0]):
            dataBase[coor,:,d]= data[coor, :, d] - zero
    return dataBase

#separe en X et label
bankdataPose=bankdata[:,0:3,:]
bankdataPosture=bankdata[:,-1,:]
DataPose=calculateZero(bankdataPose)

classifier = SVC(kernel = 'rbf', random_state=0, gamma=100)
nx, ny, nsamples = np.shape(DataPose)
d2X=np.reshape(DataPose,[nsamples,nx*ny])

lab_enc=preprocessing.LabelEncoder()
l=[]
for i in range(49):
    l.append(np.max(bankdataPosture[3,i]))

encodedY=lab_enc.fit_transform(l)

classifier.fit(d2X,encodedY)
x=classifier.predict(d2X)
print(x)
#ax=plt.axes(projection='3d')
#ax.scatter3D(bankdata[:,0,1],bankdata[:,1,1],bankdata[:,2,1],cmap='Greens',marker="s")
#ax.scatter3D(DataPose[:,0,1],DataPose[:,1,1],DataPose[:,2,1],cmap='Blue')
#plt.show()
