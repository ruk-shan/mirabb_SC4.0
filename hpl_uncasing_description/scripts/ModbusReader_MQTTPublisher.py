#!/usr/bin/env python3

# reads UR robot joints_angles, controller_status, joint_current, joint_temp, robot_current 
# over modbus and puslish to mqtt broker 
# set robot IP and broker IP 

from pyModbusTCP.client import ModbusClient
import paho.mqtt.client as mqtt 
import math
import random
import json

class ReadJoints:
    def __init__(self):
        self.client_id = f'python-mqtt-{random.randint(0, 1000)}'
        #modbus
        try:
            self.modbus_conn = ModbusClient(host="172.16.179.128", port=502, auto_open=True) #host ip is ROBOT's ip
            self.modbus_conn_status = True
            print (f"modbus_conn succsessful")
        except:
            self.modbus_conn_status = False
            print (f"modbus_conn failed")
        
        #MQTT
        try:
            self.client = mqtt.Client("P1") #create new instance
            self.client.connect("192.168.68.116",11883)
            self.mosquitto_conn_status = True
            print (f"mosquitto_conn succsessful")
            self.client.loop_start() #start the loop
        except:
            self.mosquitto_conn_status = False
            print (f"mosquitto_conn failed")

    def mrad_to_deg(self,joint_val):
        val_deg = joint_val * 180/(1000 *math.pi) 
        return val_deg

    def angle_filter(self,joint_val_mrad,counter_val):
        """fillter joint value from nagative to positive"""
        # if counter_val!= 0:
        #     joint_val_mrad = joint_val_mrad - (2*math.pi *1000) 
        return joint_val_mrad

    #read UR modbus rgisters
    def read_reg_UR(self):
        while True:

            # UR Robot status --------------------------------------
            self.reg_robot_status = self.modbus_conn.read_holding_registers(260,6) # reading robot status
            self.reg_robot_status = json.dumps({"isPowerOnRobot":self.reg_robot_status[0] ,
                                                    "isSecurityStopped":self.reg_robot_status[1],
                                                    "isEmergencyStopped":self.reg_robot_status[2],
                                                    "isTeachButtonPressed":self.reg_robot_status[3],
                                                    "isPowerPuttonPressed":self.reg_robot_status[4],
                                                    "isSafetySignalSuchThatWeShouldStop":self.reg_robot_status[5]})
            self.client.publish("uncasing_station/UR10/robot_status",self.reg_robot_status)
            # print (f"self.reg_robot_status: {self.reg_robot_status}")
            # -----------------------------------------------------

            # UR joint angles (in rad) --------------------------------------
            self.reg_joints = self.modbus_conn.read_holding_registers(270,6) # reading 6 joints in mrad
            # print (f"joints:{self.reg_joints}")
            self.reg_joint_counter = self.modbus_conn.read_holding_registers(320,6) # reading 6 joint counters 
            joint_01 = self.angle_filter(self.reg_joints[0],self.reg_joint_counter[0])/1000
            joint_02 = self.angle_filter(self.reg_joints[1],self.reg_joint_counter[1])/1000+1.5708
            joint_03 = self.angle_filter(self.reg_joints[2],self.reg_joint_counter[2])/1000
            joint_04 = self.angle_filter(self.reg_joints[3],self.reg_joint_counter[3])/1000+1.5708
            joint_05 = self.angle_filter(self.reg_joints[4],self.reg_joint_counter[4])/1000
            joint_06 = self.angle_filter(self.reg_joints[5],self.reg_joint_counter[5])/1000
            self.ur_joint_angles =  json.dumps({"joint_01":joint_01,
                                "joint_02":joint_02,
                                "joint_03":joint_03,
                                "joint_04":joint_04,
                                "joint_05":joint_05,
                                "joint_06":joint_06})
            self.client.publish("uncasing_station/UR10/joint_angles",self.ur_joint_angles,qos=0, retain=False) #publish over MQTT
            # -----------------------------------------------------

            # UR joint current (in mA)--------------------------------------
            self.reg_joints_current = self.modbus_conn.read_holding_registers(290,6) # reading current of joints
            self.reg_joints_current = json.dumps({"joint_01":self.reg_joints_current[0] ,
                                                    "joint_02":self.reg_joints_current[1],
                                                    "joint_03":self.reg_joints_current[2],
                                                    "joint_04":self.reg_joints_current[3],
                                                    "joint_05":self.reg_joints_current[4],
                                                    "joint_06":self.reg_joints_current[5]})
            self.client.publish("uncasing_station/UR10/joint_current",self.reg_joints_current)
            # print (f"self.reg_joints_current: {self.reg_joints_current}")
            # -----------------------------------------------------

            # UR robot current (in mA)--------------------------------------
            self.reg_robot_current = self.modbus_conn.read_holding_registers(450,1) # reading current of the robot
            self.client.publish("uncasing_station/UR10/robot_current",self.reg_robot_current[0])
            # print (f"self.reg_robot_current: {self.reg_robot_current[0]}")
            # -----------------------------------------------------

            # UR joint temperature (in C)--------------------------------------
            self.reg_joints_temp = self.modbus_conn.read_holding_registers(300,6) # reading temp of joints
            self.reg_joints_temp = json.dumps({"joint_01":self.reg_joints_temp[0] ,
                                                    "joint_02":self.reg_joints_temp[1],
                                                    "joint_03":self.reg_joints_temp[2],
                                                    "joint_04":self.reg_joints_temp[3],
                                                    "joint_05":self.reg_joints_temp[4],
                                                    "joint_06":self.reg_joints_temp[5]})
            self.client.publish("uncasing_station/UR10/joint_temp",self.reg_joints_temp)
            # print (f"self.reg_joints_temp: {self.reg_joints_temp}")
            # -----------------------------------------------------

            # UR TCP position ( in tenth of mm (in base frame))--------------------------------------
            self.reg_TCP_pos = self.modbus_conn.read_holding_registers(400,6) # reading temp of joints
            self.reg_TCP_pos = json.dumps({"TCP_x":self.reg_TCP_pos[0] ,
                                                    "TCP_y":self.reg_TCP_pos[1],
                                                    "TCP_z":self.reg_TCP_pos[2],
                                                    "TCP_rx":self.reg_TCP_pos[3],
                                                    "TCP_ry":self.reg_TCP_pos[4],
                                                    "TCP_rz":self.reg_TCP_pos[5]})
            self.client.publish("uncasing_station/UR10/TCP_pos",self.reg_TCP_pos)
            # print (f"self.reg_TCP_pos: {self.reg_TCP_pos}")
            # -----------------------------------------------------


    



    def main(self):
        self.read_reg_UR()

if __name__ == '__main__':
    """main"""
    try:
        JointClass = ReadJoints()
        JointClass.main()
    except:
        pass