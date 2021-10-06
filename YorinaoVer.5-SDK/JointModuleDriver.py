import odrive
from odrive.enums import *
import time
import math
import keyboard
from myutil import *

##
# @class JointModuleDriver
# @brief drives a joint module of Yorinao
# @details 
class JointModuleDriver:
    __CPR = 4096
    __GEAR_MOTOR_SIDE = 20
    __GEAR_JOINT_SIDE = 57
    __GEAR_RATIO_TIMING_BELT = float(__GEAR_MOTOR_SIDE) / float(__GEAR_JOINT_SIDE)
    __REDUCER_RATIO = 1.0 / 6.0 # Planetary Gear Box
    __MOTOR_ANGLE_TO_MOTOR_PULSE = float(__CPR) / (2.0 * math.pi)
    __MOTOR_PULSE_TO_MOTOR_ANGLE = (2.0 * math.pi) / float(__CPR)

    #__RAD2COUNT = float(__CPR) / __GEAR_RATIO_TIMING_BELT / __REDUCER_RATIO / (2.0 * math.pi)
    __RADIUS_IN = 20
    __RADIUS_OUT = 30
    __MAX_JOINT_ANGLE = 30.0 * myConv.DEG2RAD
    isServoOn = True
    my_drive = None    # ODrive Instance
    ODriveSerialNumber = None

    __OffsetMotorPulse_1 = 0.0
    __OffsetMotorPulse_2 = 0.0
    __OffsetJointAngle_1 = 0.0 * myConv.DEG2RAD
    __OffsetJointAngle_2 = None

    __OffsetJointAngle_2_Type_1 = 60.0 * myConv.DEG2RAD
    __OffsetJointAngle_2_Type_2 = 30.0 * myConv.DEG2RAD

    __InitType = None


    def __init__(self, odrive_serial_number = "20853892304E", init_type = 1, with_reducer = True):
        self.ODriveSerialNumber = odrive_serial_number
        self.__InitType = init_type

        if with_reducer == False:
            self.__REDUCER_RATIO = 1.0

        if self.__InitType == 1:
            self.__OffsetJointAngle_2 = self.__OffsetJointAngle_2_Type_1
        elif self.__InitType == 2:
            self.__OffsetJointAngle_2 = self.__OffsetJointAngle_2_Type_2
        else:
            self.__InitType = 1
            self.__OffsetJointAngle_2 = self.__OffsetJointAngle_2_Type_1

        self.ConnectToODrive()
        return


    def __del__(self):
        self.Set_AxisState_IDLE()
        return



    def ConnectToODrive(self):
        print("Connecting an odrive, Serial Number: {0} ...".format(self.ODriveSerialNumber))
        self.my_drive = odrive.find_any(serial_number = self.ODriveSerialNumber)

        self.__OffsetMotorPulse_1 = self.my_drive.axis0.encoder.pos_estimate
        self.__OffsetMotorPulse_2 = self.my_drive.axis1.encoder.pos_estimate
        return


    def EncoderCalibration(self):
        # Calibrate motor and wait for it to finish
        if self.my_drive.axis0.encoder.is_ready == False or self.my_drive.axis1.encoder.is_ready == False:
            print("starting calibration...")
            self.my_drive.axis0.requested_state = AXIS_STATE_ENCODER_OFFSET_CALIBRATION
            self.my_drive.axis1.requested_state = AXIS_STATE_ENCODER_OFFSET_CALIBRATION
            while self.my_drive.axis0.current_state != AXIS_STATE_IDLE:
                time.sleep(0.1)
            if self.my_drive.axis0.encoder.is_ready == False or self.my_drive.axis1.encoder.is_ready == False:
                print("Calibration fail\n")
        else:
            print("already calibrated\n")

        self.my_drive.axis0.requested_state = AXIS_STATE_CLOSED_LOOP_CONTROL
        self.my_drive.axis1.requested_state = AXIS_STATE_CLOSED_LOOP_CONTROL
        return


    def ForwardKinematics(self, motor_pulse_1 = 0.0, motor_pulse_2 = 0.0):
        motor_pulse_1 -= self.__OffsetMotorPulse_1
        motor_angle_1 = motor_pulse_1 * self.__MOTOR_PULSE_TO_MOTOR_ANGLE
        motor_angle_1 *= self.__REDUCER_RATIO 
        motor_angle_1 *= self.__GEAR_RATIO_TIMING_BELT

        motor_pulse_2 -= self.__OffsetMotorPulse_2
        motor_pulse_2 *= -1.0
        motor_angle_2 = motor_pulse_2 * self.__MOTOR_PULSE_TO_MOTOR_ANGLE
        motor_angle_2 *= self.__REDUCER_RATIO 
        motor_angle_2 *= self.__GEAR_RATIO_TIMING_BELT

        joint_angle_1 = - self.__RADIUS_IN / (2.0 * self.__RADIUS_OUT) * (motor_angle_1 - motor_angle_2)
        joint_angle_1 *= -1.0
        joint_angle_1 += self.__OffsetJointAngle_1

        joint_angle_2 = (motor_angle_1 + motor_angle_2) / 2.0
        joint_angle_2 += self.__OffsetJointAngle_2
        return joint_angle_1, joint_angle_2


    def InverseKinematics(self, joint_angle_1, joint_angle_2):
        joint_angle_1 *= -1.0
        joint_angle_1 -= self.__OffsetJointAngle_1
        joint_angle_2 -= self.__OffsetJointAngle_2

        motor_pulse_1 = joint_angle_2 - float(self.__RADIUS_OUT) / float(self.__RADIUS_IN) * joint_angle_1
        motor_pulse_1 /= self.__REDUCER_RATIO
        motor_pulse_1 /= self.__GEAR_RATIO_TIMING_BELT
        motor_pulse_1 *= self.__MOTOR_ANGLE_TO_MOTOR_PULSE
        motor_pulse_1 += self.__OffsetMotorPulse_1

        motor_pulse_2 = float(self.__RADIUS_OUT) / float(self.__RADIUS_IN) * joint_angle_1 + joint_angle_2
        motor_pulse_2 /= self.__REDUCER_RATIO
        motor_pulse_2 /= self.__GEAR_RATIO_TIMING_BELT
        motor_pulse_2 *= self.__MOTOR_ANGLE_TO_MOTOR_PULSE
        motor_pulse_2 *= -1.0
        motor_pulse_2 += self.__OffsetMotorPulse_2

        return motor_pulse_1, motor_pulse_2


    def ControlByKeyboard(self):
        cur_joint_angle_1, cur_joint_angle_2 = self.Get_CurrentJointAngles()
        des_joint_angle_1 = cur_joint_angle_1
        des_joint_angle_2 = cur_joint_angle_2

        move_speed = 0.001
        while True:
            if keyboard.is_pressed('esc') == True:
                break
            if self.isServoOn == True:
                if keyboard.is_pressed('up') == True:
                    des_joint_angle_2 += move_speed
                elif keyboard.is_pressed('down') == True:
                    des_joint_angle_2 -= move_speed
                elif keyboard.is_pressed('left') == True:
                    des_joint_angle_1 += move_speed
                elif keyboard.is_pressed('right') == True:
                    des_joint_angle_1 -= move_speed

                self.Set_JointAngles(des_joint_angle_1, des_joint_angle_2)


                cur_joint_angle_1, cur_joint_angle_2 = self.ForwardKinematics(
                    self.my_drive.axis0.encoder.pos_estimate,
                    self.my_drive.axis1.encoder.pos_estimate
                    )
                print("Current Joint Angle: {} / {}".format(
                    cur_joint_angle_1 * myConv.RAD2DEG, 
                    cur_joint_angle_2 * myConv.RAD2DEG
                    ))
                #adc_voltage_3 = self.my_drive.get_adc_voltage(3)
                #adc_voltage_5 = self.my_drive.get_adc_voltage(5)
                ##print("sensor: {:3.1f} / {:3.1f}".format(adc_voltage_3, adc_voltage_5))
                #cur_torq_1 = self.my_drive.axis0.motor.current_control.Iq_measured
                ##print("sensor: {:3.1f}".format(cur_torq_1))
            #end of while
        time.sleep(0.5)
        return


    def Init_JointPos(self):
        cur_joint_angle_1, cur_joint_angle_2 = self.Get_CurrentJointAngles()
        des_joint_angle_1 = cur_joint_angle_1
        des_joint_angle_2 = cur_joint_angle_2

        move_speed = 0.0002

        # Search for Joint 2
        if self.__InitType == 1:
            if 2.2 < self.my_drive.get_adc_voltage(5):
                while True:
                    if self.my_drive.get_adc_voltage(5) < 2.2:
                        break
                    des_joint_angle_2 += move_speed
                    self.Set_JointAngles(des_joint_angle_1, des_joint_angle_2)
            while True:
                if keyboard.is_pressed('esc') == True:
                    break
                if keyboard.is_pressed('up') == True:
                    des_joint_angle_2 += move_speed * 2.0
                elif keyboard.is_pressed('down') == True:
                    des_joint_angle_2 -= move_speed * 2.0
                else:
                    if 2.2 < self.my_drive.get_adc_voltage(5):
                        break
                    des_joint_angle_2 -= move_speed

                self.Set_JointAngles(des_joint_angle_1, des_joint_angle_2)
                #end of while
        elif self.__InitType == 2:
            if 2.2 < self.my_drive.get_adc_voltage(5):
                while True:
                    if self.my_drive.get_adc_voltage(5) < 2.2:
                        break
                    des_joint_angle_2 -= move_speed
                    self.Set_JointAngles(des_joint_angle_1, des_joint_angle_2)
            while True:
                if keyboard.is_pressed('esc') == True:
                    break
                if keyboard.is_pressed('up') == True:
                    des_joint_angle_2 += move_speed * 2.0
                elif keyboard.is_pressed('down') == True:
                    des_joint_angle_2 -= move_speed * 2.0
                else:
                    if 2.2 < self.my_drive.get_adc_voltage(5):
                        break
                    des_joint_angle_2 += move_speed

                self.Set_JointAngles(des_joint_angle_1, des_joint_angle_2)
                #end of while
        else:
            return


        # Search for Joint 1
        if 2.2 < self.my_drive.get_adc_voltage(3):
            while True:
                if self.my_drive.get_adc_voltage(3) < 2.2:
                    break
                des_joint_angle_1 -= move_speed
                self.Set_JointAngles(des_joint_angle_1, des_joint_angle_2)

        while True:
            if keyboard.is_pressed('esc') == True:
                break
            if keyboard.is_pressed('left') == True:
                des_joint_angle_1 += move_speed * 2.0
            elif keyboard.is_pressed('right') == True:
                des_joint_angle_1 -= move_speed * 2.0
            else:
                if 2.2 < self.my_drive.get_adc_voltage(3):
                    break
                des_joint_angle_1 += move_speed

            self.Set_JointAngles(des_joint_angle_1, des_joint_angle_2)
            #end of while

        self.__OffsetMotorPulse_1 = self.my_drive.axis0.encoder.pos_estimate
        self.__OffsetMotorPulse_2 = self.my_drive.axis1.encoder.pos_estimate
        return



    def __Set_MotorAngles(self, des_motor_angle_1, des_motor_angle_2):
        self.my_drive.axis0.controller.pos_setpoint = des_motor_angle_1
        self.my_drive.axis1.controller.pos_setpoint = des_motor_angle_2
        return


    def Set_AxisState_IDLE(self):
        self.my_drive.axis0.requested_state = AXIS_STATE_IDLE
        self.my_drive.axis1.requested_state = AXIS_STATE_IDLE
        return


    def Set_JointAngle(self, joint_number = 1, des_angle = 0.0):
        cur_angle_1, cur_angle_2 = self.Get_CurrentJointAngles()
        if joint_number == 1:
            self.Set_JointAngles(des_angle, cur_angle_2)
        elif joint_number == 2:
            self.Set_JointAngles(cur_angle_1, des_angle)
        else:
            self.Set_JointAngles(cur_angle_1, cur_angle_2)
        return


    def Set_JointAngles(self, des_angle_1, des_angle_2):
        motor_pulse_1, motor_pulse_2 = self.InverseKinematics(des_angle_1, des_angle_2)
        self.__Set_MotorAngles(motor_pulse_1, motor_pulse_2)
        return


    def Get_CurrentJointAngles(self):
        cur_joint_angle_1, cur_joint_angle_2 = self.ForwardKinematics(
            self.my_drive.axis0.encoder.pos_estimate,
            self.my_drive.axis1.encoder.pos_estimate
        )
        return cur_joint_angle_1, cur_joint_angle_2


    def Get_ODriveSerialNumber(self):
        return ODriveSerialNumber


    def Get_ODriveInstance(self):
        return self.my_drive

    def MoveToJoint(self, joint_number = 1, des_joint_angle = 0.0, time_to_move = 10.0):
        cur_joint_angle_1, cur_joint_angle_2 = self.Get_CurrentJointAngles()

        theta_0 = None
        if joint_number == 1:
            theta_0 = cur_joint_angle_1
        elif joint_number == 2:
            theta_0 = cur_joint_angle_2
        else:
            return
        theta_f = des_joint_angle
        theta_0_dot = 0.0
        theta_f_dot = 0.0
        theta_0_dotdot = 0.0
        theta_f_dotdot = 0.0
        t_f = time_to_move

        a0 = theta_0
        a1 = theta_0_dot
        a2 = theta_0_dotdot / 2.0
        a3 = (20.0 * theta_f - 20.0 * theta_0 - (8.0 * theta_f_dot + 12.0 * theta_0_dot) * t_f - (3.0 * theta_0_dotdot - theta_f_dotdot) * (t_f ** 2.0)) / (2.0 * (t_f ** 3.0))
        a4 = (30.0 * theta_0 - 30.0 * theta_f + (14.0 * theta_f_dot + 16.0 * theta_0_dot) * t_f + (3.0 * theta_0_dotdot - 2.0 * theta_f_dotdot) * (t_f ** 2.0)) / (2.0 * (t_f ** 4.0))
        a5 = (12.0 * theta_f - 12.0 * theta_0 - (6.0 * theta_f_dot + 6.0 * theta_0_dot) * t_f - (theta_0_dotdot - theta_f_dotdot) * (t_f ** 2.0)) / (2.0 * (t_f ** 5.0))

        tk = TimeKeeper(0.01)
        start_time = time.time()
        while True:
            t = time.time() - start_time
            if t_f < t:
                break

            des_angle = a0 
            des_angle += a1 * t
            des_angle += a2 * (t ** 2.0)
            des_angle += a3 * (t ** 3.0)
            des_angle += a4 * (t ** 4.0)
            des_angle += a5 * (t ** 5.0)

            print("time: {:.3f} / des angle: {:.3f}".format(t, des_angle * myConv.RAD2DEG))

            tk.SleepToKeep()
            if joint_number == 1:
                self.Set_JointAngles(des_angle, cur_joint_angle_2)
            elif joint_number == 2:
                self.Set_JointAngles(cur_joint_angle_1, des_angle)
            else:
                return
        return

## To read a value, simply read the property
#print("Bus voltage is " + str(my_drive.vbus_voltage) + "V")

## Or to change a value, just assign to the property
#my_drive.axis0.controller.pos_setpoint = 3.14
#print("Position setpoint is " + str(my_drive.axis0.controller.pos_setpoint))

## And this is how function calls are done:
#for i in [1,2,3,4]:
#    print('voltage on GPIO{} is {} Volt'.format(i, my_drive.get_adc_voltage(i)))



