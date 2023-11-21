#!/usr/bin/env python
#import logging
#include <stdio.h>
import rospy
import re
import socket
from sqlite3 import connect
from std_srvs.srv import Trigger, TriggerResponse
#from robotnik_msgs.srv import SetString, SetStringResponse

import threading


UR_INTERPRETER_SOCKET = 29999


class DashboardHelper:
#    log = logging.getLogger("interpreter.InterpreterHelper")

    def __init__(self, ip, port=UR_INTERPRETER_SOCKET):
        self.socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        self.ip = ip
        self.port = port
        self.status_msg = ""
        self.command_id = 0
        self.connected_flag = False
        self.timeout = 1
        self.lock = threading.Lock()
        self.connect()
        self.connect()
        self.timeout = 10
        self.connect()



    def connect(self):
        response = TriggerResponse()
        try:
            self.socket.settimeout(self.timeout)
            print(self.socket.connect((self.ip, self.port)))
            rospy.loginfo("Connected")
            response.success = True
            response.message = self.get_reply()
        except socket.error as exc:
            status = str(exc)
            if status.find("already connected") ==-1:
                rospy.logerr("")
                response.success = False
                response.message = "Connection failed"
            else:
                rospy.logwarn("Already connected")
                try:
                    if self.quit().success == True:
                        rospy.loginfo("Disconnected")
                        self.socket.connect((self.ip, self.port))
                        rospy.loginfo("Connected")
                        response.success = True
                        response.message = self.get_reply()
                    else:
                        response.success = False
                        response.message = "Disconnecting failed"
                except:        
                    response.success = False
                    response.message = "Disconnecting and connecting process failed"
        return response


    def quit(self):
        response = TriggerResponse()
        response = self.quit_dashboard()
        response = self.disconnect()
        return response


    def disconnect(self):
        response = TriggerResponse()
        try:
            self.socket.close()
            self.set_status("Disconnected")
            self.set_connected_status(False)
            self.socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
            response.success = True
            response.message = "Disconnected"
        except socket.error as exc:
            status = str(exc)
            self.set_status(status)
            response.success = False
            response.message = status
        return response


    def get_reply(self):
        msg = ""
        try:
            collected = b''
            collected = self.socket.recv(1024)
            msg = collected.decode("utf-8")
        except socket.error as exc:
            msg = str(exc)
        rospy.loginfo(msg)
        return msg


    def execute_command(self, command):
        """
        Send single line command to interpreter mode, and wait for reply
        :param command:
        :return: ack, or status id
        """
        response = TriggerResponse()
        command_raw=command
        if not command.endswith("\n"):
            command += "\n"
        try:
            self.socket.send(command.encode("utf-8"))
            response.message = self.get_reply()
            response.success = True
            return response
        except socket.error as exc:
            if exc.errno==32:
                rospy.loginfo("Reconnecting...")
                response = self.connect()
                if response.success:
                    rospy.loginfo("Sending command")
                    return self.execute_command(command_raw)
                else:
                    return response
                
            response.message = str(exc)
            response.success = False
            self.set_status(str(exc))
            self.set_command_id(-1)
        return response


    def load_program(self, program):
        if not program.endswith(".urp"):
            program += ".urp"
        command = "load " + program
        ret = self.execute_command(command)
        if ret.success:
            if ret.message.find("Loading") == -1:
                ret.success = False
        return ret
    
    def play(self):
        ret = self.execute_command("play")
        if ret.success:
            if ret.message.find("Starting") == -1:
                ret.success = False
        return ret
    
    def stop(self):
        ret = self.execute_command("stop")
        if ret.success:
            if ret.message.find("Stopped") == -1:
                ret.success = False
        return ret
    
    def pause(self):
        ret = self.execute_command("pause")
        if ret.success:
            if ret.message.find("Pausing") == -1:
                ret.success = False
        return ret
    
    def quit_dashboard(self):
        ret = self.execute_command("quit")
        if ret.success:
            if ret.message.find("Disconnected") == -1:
                ret.succes = False
        return ret
    
    def shutdown(self):
        ret = self.execute_command("shutdown")
        if ret.success:
            if ret.message.find("Shutting") == -1:
                ret.succes = False
        return ret

    def skip(self):
        return self.execute_command("skipbuffer")


    def abort_move(self):
        return self.execute_command("abort")


    def get_last_interpreted_id(self):
        return self.execute_command("statelastinterpreted")


    def get_last_executed_id(self):
        return self.execute_command("statelastexecuted")


    def get_last_cleared_id(self):
        return self.execute_command("statelastcleared")


    def get_unexecuted_count(self):
        return self.execute_command("stateunexecuted")


    def quit_dashboard(self):
        return self.execute_command("quit")


    def get_status(self):
        self.lock.acquire()       
        return_msg = self.status_msg
        self.lock.release()
        return return_msg


    def get_command_id(self):
        self.lock.acquire()       
        id_n = self.command_id
        self.lock.release()
        return id_n


    def get_connected_status(self):
        self.lock.acquire()       
        connected = self.connected_flag
        self.lock.release()
        return connected


    def set_status(self, msg):
        self.lock.acquire()
        self.status_msg = msg
        self.lock.release()


    def set_command_id(self, id_n):
        self.lock.acquire()
        self.command_id = id_n
        self.lock.release()


    def set_connected_status(self, connected):
        self.lock.acquire()
        self.connected_flag = connected
        self.lock.release()
