from YorinaoDriver import *
from myutil import *
import keyboard
import time


Yorinao = YorinaoDriver(["20853892304E", "20513892304E", "205439964D4D"])
Yorinao.EncoderCalibrations()
Yorinao.Init_JointPos()

JointAngle = Yorinao.Get_JointAngles()
for i in range(6):
	print("Joint[{}] = {}".format(i, JointAngle[i] * myConv.RAD2DEG))

Yorinao.MotionTest()

Yorinao.JointModule[0].ControlByKeyboard()


JointAngle = [30.0 * myConv.DEG2RAD, 
			  30.0 * myConv.DEG2RAD, 
			  120.0 * myConv.DEG2RAD, 
			  0.0 * myConv.DEG2RAD, 
			  60.0 * myConv.DEG2RAD, 
			  0.0 * myConv.DEG2RAD]

print("Joint Angle:")
for angle in JointAngle:
	print("{}".format(angle * myConv.RAD2DEG))
print("==============")

Transform = Yorinao.ForwardKinematics(JointAngle)
np.set_printoptions(precision=3, suppress=True)
print("Transform: \n{}".format(Transform))
print("==============")

Transform[1, 3] += - 0.10
JointAngle = Yorinao.InverseKinematics(Transform)
print("Joint Angle:")
for angle in JointAngle:
	print("{:6.3f}".format(angle * myConv.RAD2DEG))
