import Adafruit_PCA9685
import numpy as np
from threading import Event

#sys.path.append(".")
import p_fkdh as fk

#ROS
import rospy
from uavlab411.msg import drawing_point_msg

xt,yt,zt=0,0,0
j1,j2,j3,j4=0,0,0,0
x4,y4,z4,Tm=0,0,0,0

#Initial PCA9685
pwm = Adafruit_PCA9685.PCA9685(address=0x40)
pwm.set_pwm_freq(int(60))

def initState():
    global j1, j2, j3, j4
     
    #Initial Joint Angles
    j1 = 90
    j2 = 60
    j3 = -90
    j4 = -j2 - j3
    drawfk(j1, j2, j3)

def SetPos(Channel, Pos):
    pulse = int((650 - 150)/180*Pos + 150)
    pwm.set_pwm(Channel, 0, pulse)

def callback(drawing_point):
    global xt, yt, zt

    xt = int(drawing_point.x / 4.95)  # Width of Image / 40 
    yt = int(drawing_point.z)
    zt = int(drawing_point.y / 8.5)  # Height of Image / 30
        
    pinj_ik()

def drawing_point_recv():
    rospy.init_node('drawing_point_recv', anonymous=True)
    rospy.Subscriber("uavlab411/drawing_point", drawing_point_msg, callback)
    rospy.spin()

def pinj_ik():
    global j1, j2, j3, j4

    x_start=x4
    y_start=y4
    z_start=z4

    ptarget=np.vstack([xt, yt, zt])
    pstart=np.vstack([x_start,y_start,z_start])
    delta=fk.divelo(ptarget, pstart)
    step=5
    EucXYZ=delta[4]
    if abs(EucXYZ)>5:
        pembagi_step=abs(EucXYZ)/step
        dXYZ=delta[0:3]/pembagi_step
    else:
        dXYZ=delta[0:3]
    if abs(EucXYZ)<=0.01:
        print("Warning!!!")          
    else:
        Jac=fk.jacobian(Tm)
        Jac_Inv=fk.PinvJac(Jac)
        dTheta=Jac_Inv@dXYZ
        dTheta1=np.rad2deg(dTheta[0])
        dTheta2=np.rad2deg(dTheta[1])
        dTheta3=np.rad2deg(dTheta[2])
        j1=j1+dTheta1
        j2=j2+dTheta2
        j3=j3+dTheta3
        j4=-j2-j3

        J0 = int(j1[0])
        J1 = int(180 - j2[0])
        J2 = int(90 - j3[0])
        J3 = 80
        J4 = int(360 - J1 - J2)

        SetPos(0, J0)
        SetPos(1, 180 - J1)
        SetPos(2, J2)
        SetPos(3, J3)
        SetPos(4, J4 + 10)
        print(J0, ' ', J1, ' ', J2, ' ', J3, ' ', J4)

        drawfk(j1,j2,j3)
        pinj_ik()

def drawfk(a,b,c): 
    global x4, y4, z4, Tm

    j=fk.dh_par(a,b,c)
    Tm=fk.dh_kine(j)
    ee=fk.el_xyzpos(Tm)
    p0,p1,p2,p3,p4,p5=fk.el_pos2base(Tm)
    X1,Y1,Z1=ee[0,0:3],ee[1,0:3],ee[2,0:3]
    X2,Y2,Z2=ee[0,2:4],ee[1,2:4],ee[2,2:4]
    X3,Y3,Z3=ee[0,3:5],ee[1,3:5],ee[2,3:5]
    X4,Y4,Z4=ee[0,4:6],ee[1,4:6],ee[2,4:6]
        
    x4=X4[1]
    y4=Y4[1]
    z4=Z4[1]


if __name__ == '__main__':
    initState()
    drawing_point_recv()
