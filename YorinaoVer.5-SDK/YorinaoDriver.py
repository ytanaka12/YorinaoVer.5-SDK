from JointModuleDriver import *
from myutil import *
import math
from math import *
import numpy as np


class YorinaoDriver:
	#1st ODrive "20853892304E"
	#2nd ODrive "20513892304E"
	#3rd ODrive "205439964D4D"

	__ODriveSerialNumber = ["20853892304E", "20513892304E", "205439964D4D"]	#[1st, 2nd, 3rd]
	JointModule = []

	JointAngle = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
	__a_2 = 0.372	#Link Param
	__a_3 = 0.0	#Link Param
	__d_3 = 0.0	#Link Param
	__d_4 = 0.376	#Link Param
	
	offset_angle = [0.0] * 6
	offset_angle[0] = +  0.0 * myConv.DEG2RAD
	offset_angle[1] = -  5.0 * myConv.DEG2RAD
	offset_angle[2] = +  51.0 * myConv.DEG2RAD
	offset_angle[3] = +  0.0 * myConv.DEG2RAD
	offset_angle[4] = +  0.0 * myConv.DEG2RAD
	offset_angle[5] = +  0.0 * myConv.DEG2RAD


	def __init__(self, odrive_serial_number = ["20853892304E", "20513892304E", "205439964D4D"]):
		self.__ODriveSerialNumber = odrive_serial_number
		self.JointModule.append(JointModuleDriver(self.__ODriveSerialNumber[0], init_type = 2, with_reducer=True))
		self.JointModule.append(JointModuleDriver(self.__ODriveSerialNumber[1], init_type = 1, with_reducer=True))
		self.JointModule.append(JointModuleDriver(self.__ODriveSerialNumber[2], init_type = 1, with_reducer=False))
		return


	def __del__(self):
		for i in range(3):
			self.JointModule[i].Set_AxisState_IDLE()
		return


	def EncoderCalibrations(self):
		for i in range(3):
			self.JointModule[i].EncoderCalibration()
		return


	def Init_JointPos(self):
		for i in range(3):
			self.JointModule[i].Init_JointPos()
		return


	def Module2Robot(self, angle_on_module = [0.0] * 6):
		angle_on_robot = [0.0] * 6
		for i in range(6):
			angle_on_robot[i] = angle_on_module[i] + self.offset_angle[i]
		return angle_on_robot


	def Robot2Module(self, angle_on_robot = [0.0] * 6):
		angle_on_module = [0.0] * 6
		for i in range(6):
			angle_on_module[i] = angle_on_robot[i] - self.offset_angle[i]
		return angle_on_module


	def Set_JointAngles(self, joint_angle = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0]):
		des_angle = self.Robot2Module(joint_angle)
		self.JointModule[0].Set_JointAngles(des_angle[0], des_angle[1])
		self.JointModule[1].Set_JointAngles(des_angle[3], des_angle[2])
		self.JointModule[2].Set_JointAngles(des_angle[5], des_angle[4])
		return


	def Set_Pose(self, pose):
		des_angle = self.InverseKinematics(pose)
		self.Set_JointAngles(des_angle)
		return


	def Get_JointAngles(self):
		angle = []
		joint_angle_1, joint_angle_2 = self.JointModule[0].Get_CurrentJointAngles()
		joint_angle_4, joint_angle_3 = self.JointModule[1].Get_CurrentJointAngles()
		joint_angle_6, joint_angle_5 = self.JointModule[2].Get_CurrentJointAngles()
		angle.append(joint_angle_1)
		angle.append(joint_angle_2)
		angle.append(joint_angle_3)
		angle.append(joint_angle_4)
		angle.append(joint_angle_5)
		angle.append(joint_angle_6)
		joint_angle = self.Module2Robot(angle)
		return joint_angle
	

	def Get_CurrentPose(self):
		cur_angle = self.Get_JointAngles()
		cur_pose = self.ForwardKinematics(cur_angle)
		return cur_pose


	def MoveJoint(self, target_joint_angle = [0.0] * 6, time_to_move = 10.0):
		cur_joint_angle = self.Get_JointAngles()
		trajectory = []
		for i in range(6):
			traj = TrajGen_HigherOrderPolynomials(cur_joint_angle[i], target_joint_angle[i], time_to_move)
			trajectory.append(traj)

		des_joint_angle = cur_joint_angle
		tk = TimeKeeper(0.01)
		start_time = time.time()
		while True:
			t = time.time() - start_time
			if time_to_move < t:
				break

			for i in range(6):
				des_joint_angle[i] = trajectory[i].Get_Trajectory(t)

			self.Set_JointAngles(des_joint_angle)
			tk.SleepToKeep()
		return


	def MoveLinearPose(self, target_hTransMat, time_to_move = 10.0):
		cur_pose = self.Get_CurrentPose()
		traj_x = TrajGen_HigherOrderPolynomials(cur_pose[0, 3], target_hTransMat[0, 3], time_to_move)
		traj_y = TrajGen_HigherOrderPolynomials(cur_pose[1, 3], target_hTransMat[1, 3], time_to_move)
		traj_z = TrajGen_HigherOrderPolynomials(cur_pose[2, 3], target_hTransMat[2, 3], time_to_move)

		des_pose = cur_pose
		tk = TimeKeeper(0.01)
		start_time = time.time()
		while True:
			t = time.time() - start_time
			if time_to_move < t:
				break

			x = traj_x.Get_Trajectory(t)
			y = traj_y.Get_Trajectory(t)
			z = traj_z.Get_Trajectory(t)

			des_pose[0, 3] = x
			des_pose[1, 3] = y
			des_pose[2, 3] = z

			self.Set_Pose(des_pose)
			tk.SleepToKeep()
		return


	def MotionTest(self):
		target_joint_angle = [0.0] * 6
		target_joint_angle[0] = 5.0 * myConv.DEG2RAD
		target_joint_angle[1] = 0.0 * myConv.DEG2RAD
		target_joint_angle[2] = 0.0 * myConv.DEG2RAD
		target_joint_angle[3] = 0.0 * myConv.DEG2RAD
		target_joint_angle[4] = 0.0 * myConv.DEG2RAD
		target_joint_angle[5] = 0.0 * myConv.DEG2RAD
		self.MoveJoint(target_joint_angle, time_to_move=5.0)

		target_joint_angle[0] = 30.0 * myConv.DEG2RAD
		target_joint_angle[1] = 30.0 * myConv.DEG2RAD
		target_joint_angle[2] = 120.0 * myConv.DEG2RAD
		target_joint_angle[3] = 0.0 * myConv.DEG2RAD
		target_joint_angle[4] = 60.0 * myConv.DEG2RAD
		target_joint_angle[5] = 0.0 * myConv.DEG2RAD
		self.MoveJoint(target_joint_angle, time_to_move=5.0)

		des_pose = self.Get_CurrentPose()
		des_pose[1, 3] += - 0.20
		self.MoveLinearPose(des_pose, 0.8)

		des_pose[0, 3] += 0.10
		self.MoveLinearPose(des_pose, 0.8)

		des_pose[0, 3] -= 0.10
		self.MoveLinearPose(des_pose, 0.8)

		des_pose[2, 3] += 0.10
		self.MoveLinearPose(des_pose, 0.8)

		des_pose[1, 3] += + 0.20
		self.MoveLinearPose(des_pose, 0.8)

		des_pose[1, 3] += - 0.20
		self.MoveLinearPose(des_pose, 3.0)

		target_joint_angle[0] = 0.0 * myConv.DEG2RAD
		target_joint_angle[1] = 0.0 * myConv.DEG2RAD
		target_joint_angle[2] = 120.0 * myConv.DEG2RAD
		target_joint_angle[3] = 0.0 * myConv.DEG2RAD
		target_joint_angle[4] = 60.0 * myConv.DEG2RAD
		target_joint_angle[5] = 0.0 * myConv.DEG2RAD
		self.MoveJoint(target_joint_angle, time_to_move=5.0)

		return


	##
	#@brief calculates Forward Kinematics of Yorinao Robot Arm.
	#@param joint_angle = []
	def ForwardKinematics(self, joint_angle = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0]):
		joint_angle[1] += - 90.0 * myConv.DEG2RAD
		joint_angle[2] += - 90.0 * myConv.DEG2RAD

		Trans = []
		s1 = sin(joint_angle[0])
		c1 = cos(joint_angle[0])
		T_0 = np.matrix([
			[c1, -s1, 0.0, 0.0],
			[s1,  c1, 0.0, 0.0],
			[0.0, 0.0, 1.0, 0.0],
			[0.0, 0.0, 0.0, 1.0]
			])
		Trans.append(T_0)

		s2 = sin(joint_angle[1])
		c2 = cos(joint_angle[1])
		T_1 = np.matrix([
			[c2, - s2, 0.0, 0.0],
			[0.0, 0.0, 1.0, 0.0],
			[-s2, -c2, 0.0, 0.0],
			[0.0, 0.0, 0.0, 1.0]
			])
		Trans.append(T_1)

		s3 = sin(joint_angle[2])
		c3 = cos(joint_angle[2])
		T_2 = np.matrix([
			[c3, -s3, 0.0, self.__a_2],
			[s3,  c3, 0.0, 0.0],
			[0.0, 0.0, 1.0, self.__d_3],
			[0.0, 0.0, 0.0, 1.0]
			])
		Trans.append(T_2)

		s4 = sin(joint_angle[3])
		c4 = cos(joint_angle[3])
		T_3 = np.matrix([
			[c4, - s4, 0.0, self.__a_3],
			[0.0, 0.0, 1.0, self.__d_4],
			[-s4, -c4, 0.0, 0.0],
			[0.0, 0.0, 0.0, 1.0]
			])
		Trans.append(T_3)

		s5 = sin(joint_angle[4])
		c5 = cos(joint_angle[4])
		T_4 = np.matrix([
			[c5, -s5, 0.0, 0.0],
			[0.0, 0.0, -1.0, 0.0],
			[s5,  c5, 0.0, 0.0],
			[0.0, 0.0, 0.0, 1.0]
			])
		Trans.append(T_4)

		s6 = sin(joint_angle[5])
		c6 = cos(joint_angle[5])
		T_5 = np.matrix([
			[c6, - s6, 0.0, 0.0],
			[0.0, 0.0, 1.0, 0.0],
			[-s6, -c6, 0.0, 0.0],
			[0.0, 0.0, 0.0, 1.0]
			])
		Trans.append(T_5)

		transform = np.eye(4)
		for T in Trans:
			transform = np.dot(transform, T)

		return transform


	##
	#@brief calculates Forward Kinematics of Yorinao Robot Arm.
	#@param h_transform = (numpy.matrix(4x4))
	def InverseKinematics(self, h_transform):
		r = h_transform
		px = h_transform[0, 3]
		py = h_transform[1, 3]
		pz = h_transform[2, 3]

		a_1 = atan2(py, px)
		a_2 = atan2(self.__d_3, + sqrt(px ** 2.0 + py ** 2.0 - self.__d_3 ** 2.0))
		#a_2 = atan2(self.__d_3, - sqrt(px ** 2.0 + py ** 2.0 - self.__d_3 ** 2.0))
		theta_1 = a_1 - a_2
		s1 = sin(theta_1)
		c1 = cos(theta_1)

		K = (px ** 2.0 + py ** 2.0 + pz ** 2.0 - self.__a_2 ** 2.0 - self.__a_3 ** 2.0 - self.__d_3 ** 2.0 - self.__d_4 ** 2.0) / (2.0 * self.__a_2)
		b_1 = atan2(self.__a_3, self.__d_4)
		b_2 = atan2(K, + sqrt(self.__a_3 ** 2.0 + self.__d_4 ** 2.0 - K ** 2.0))
		#b_2 = atan2(K, - sqrt(self.__a_3 ** 2.0 + self.__d_4 ** 2.0 - K ** 2.0))
		theta_3 = b_1 - b_2
		s3 = sin(theta_3)
		c3 = cos(theta_3)

		c_1 = (- self.__a_3 - self.__a_2 * c3) * pz
		c_2 = (c1 * px + s1 * py) * (self.__d_4 - self.__a_2 * s3)
		c_3 = (self.__a_2 * s3 - self.__d_4) * pz
		c_4 = (self.__a_3 + self.__a_2 * c3) * (c1 * px + s1 * py)
		theta_23 = atan2(c_1 - c_2, c_3 + c_4)
		s23 = sin(theta_23)
		c23 = cos(theta_23)
		theta_2 = theta_23 - theta_3
		s2 = sin(theta_2)
		c2 = cos(theta_2)
		#print("theta_23: {}".format(theta_23 * myConv.RAD2DEG))

		d_1 = - r[0, 2] * s1 + r[1, 2] * c1
		d_2 = - r[0, 2] * c1 * c23
		d_3 = - r[1, 2] * s1 * c23
		d_4 = + r[2, 2] * s23
		theta_4 = atan2(d_1, d_2 + d_3 + d_4)
		s4 = sin(theta_4)
		c4 = cos(theta_4)

		e_1 = r[0, 2] * (c1 * c23 * c4 + s1 * s4)
		e_2 = r[1, 2] * (s1 * c23 * c4 - c1 * s4)
		e_3 = r[2, 2] * (s23 * c4)
		e_4 = r[0, 2] * (-c1 * s23)
		e_5 = r[1, 2] * (-s1 * s23)
		e_6 = r[2, 2] * (-c23)
		s5 = -(e_1 + e_2 - e_3)
		c5 = e_4 + e_5 + e_6
		theta_5 = atan2( s5, c5 )

		f_1 = r[0, 0] * (c1 * c23 * s4 - s1 * c4)
		f_2 = r[1, 0] * (s1 * c23 * s4 + c1 * c4)
		f_3 = r[2, 0] * (s23 * s4)
		f_4 = c1 * c23 * c4 + s1 * s4
		f_5 = c1 * s23 * s5
		f_6 = s1 * c23 * c4 - c1 * s4
		f_7 = s1 * s23 * s5
		f_8 = s23 * c4 * c5 + c23 * s5
		s6 = - f_1 - f_2 + f_3
		c6 = r[0, 0] * ( f_4 * c5 - f_5 ) + r[1, 0] * ( f_6 * c5 - f_7 ) - r[2, 0] * f_8
		theta_6 = atan2(s6, c6)

		joint_angle = [theta_1, theta_2, theta_3, theta_4, theta_5, theta_6]

		joint_angle[1] += 90.0 * myConv.DEG2RAD
		joint_angle[2] += 90.0 * myConv.DEG2RAD

		return joint_angle





class TrajGen_HigherOrderPolynomials:
	theta_0 = None
	theta_f = None
	theta_0_dot = 0.0
	theta_f_dot = 0.0
	theta_0_dotdot = 0.0
	theta_f_dotdot = 0.0
	t_f = None

	a0 = None
	a1 = None
	a2 = None
	a3 = None
	a4 = None
	a5 = None


	def __init__(self, start_pos = 0.0, end_pos = 0.0, time_to_move = 10.0):
		self.Set_Conditions(start_pos, end_pos, time_to_move)
		return


	def Set_Conditions(self, start_pos = 0.0, end_pos = 0.0, time_to_move = 10.0):
		self.theta_0 = start_pos
		self.theta_f = end_pos
		if time_to_move < 0.1:
			self.t_f = 0.1
		else:
			self.t_f = time_to_move

		self.a0 = self.theta_0
		self.a1 = self.theta_0_dot
		self.a2 = self.theta_0_dotdot / 2.0
		self.a3 = (20.0 * self.theta_f - 20.0 * self.theta_0 - (8.0 * self.theta_f_dot + 12.0 * self.theta_0_dot) * self.t_f - (3.0 * self.theta_0_dotdot - self.theta_f_dotdot) * (self.t_f ** 2.0)) / (2.0 * (self.t_f ** 3.0))
		self.a4 = (30.0 * self.theta_0 - 30.0 * self.theta_f + (14.0 * self.theta_f_dot + 16.0 * self.theta_0_dot) * self.t_f + (3.0 * self.theta_0_dotdot - 2.0 * self.theta_f_dotdot) * (self.t_f ** 2.0)) / (2.0 * (self.t_f ** 4.0))
		self.a5 = (12.0 * self.theta_f - 12.0 * self.theta_0 - (6.0 * self.theta_f_dot + 6.0 * self.theta_0_dot) * self.t_f - (self.theta_0_dotdot - self.theta_f_dotdot) * (self.t_f ** 2.0)) / (2.0 * (self.t_f ** 5.0))
		return


	def Get_Trajectory(self, time = 0.0):
		t = time
		if self.t_f < t:
			t = self.t_f

		des_pos = self.a0 
		des_pos += self.a1 * t
		des_pos += self.a2 * (t ** 2.0)
		des_pos += self.a3 * (t ** 3.0)
		des_pos += self.a4 * (t ** 4.0)
		des_pos += self.a5 * (t ** 5.0)

		return des_pos


	def Get_t_f(self):
		return self.t_f
# end class
