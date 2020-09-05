#!/usr/bin/env python
# -*- coding: utf-8 -*-

from AWSIoTPythonSDK.MQTTLib import AWSIoTMQTTClient

import rospy

import os
import random
import types
import json

class Mqtt:
    AllowedActions = ['both', 'publish', 'subscribe']

    def __init__(self, config, subscribe_cb_list=[], client_id_prefix="mqtt_remote_console_"):
        self.__iot_data = config
        self.__subscribe_cb_list = subscribe_cb_list
        self.__client_id = config["thingName"] + client_id_prefix + str(random.randint(0, 0x7fffffff))  # prefix なのに先頭に来ていないのは突っ込まないで...

        self.__init_mqtt_client()
        self.__init_mqtt_subscribers()


    def mqtt_publish(self, msg, topic):
        try:
            json_msg = json.dumps(msg)
        except TypeError as e:
            rospy.logwarn("[ERROR][remote_sonsole]Mqtt::mqtt_publish JSON 文字列に変換することができませんでした")
            rospy.logwarn(e)

        try:
            self.myAWSIoTMQTTClient.publish(topic, json_msg, 1)
        except Exception as e:
            rospy.logwarn("[ERROR][remote_sonsole]Mqtt::mqtt_publish got exception")
            rospy.logwarn(e)
    
    def mqtt_publish_str(self, msg, topic):
        if not type(msg) is types.StringType:
            rospy.logwarn("[ERROR][remote_sonsole]Mqtt::mqtt_publish_str メッセージ(msg)は文字列である必要があります")
            return
        self.myAWSIoTMQTTClient.publish(topic, msg, 1)
        


    def __init_mqtt_subscribers(self):
        self.myAWSIoTMQTTClient.connect()
        for cb in self.__subscribe_cb_list:
            if not "topic" in cb:
                rospy.logerr("[ERROR][remote_sonsole]Mqtt::__init_mqtt_subscribers topic が設定されていません")
                exit(1)
            if not type(cb["topic"]) is types.StringType:
                rospy.logerr("[ERROR][remote_sonsole]Mqtt::__init_mqtt_subscribers topic は文字列にする必要があります")
                exit(1)
            if not "cb" in cb:
                rospy.logerr("[ERROR][remote_sonsole]Mqtt::__init_mqtt_subscribers cb が設定されていません。cb とはコールバック関数のことです")
                exit(1)
            self.myAWSIoTMQTTClient.subscribe(cb["topic"], 1, cb["cb"])
            rospy.loginfo("[remote_sonsole]Mqtt::__init_mqtt_subscribers Topic: %s" % cb["topic"])


    def __init_mqtt_client(self):
        # Grab all required info from the parsed data
        folder_path = self.__iot_data['configFilePath']

        host = self.__iot_data['endpoint']
        rootCAPath = os.path.join(folder_path, self.__iot_data['rootCAFile'])
        certificatePath = os.path.join(folder_path, self.__iot_data['certFile'])
        privateKeyPath = os.path.join(
            folder_path, self.__iot_data['privateKeyFile'])
        useWebsocket = self.__iot_data['useWebsocket']
        self.mode = self.__iot_data['mqttMode']

        if self.mode not in Mqtt.AllowedActions:
            rospy.logwarn("[ERROR][remote_sonsole]Mqtt::_init_mqtt_client Unknown --mode option %s. Must be one of %s" %
                          (self.mode, str(Mqtt.AllowedActions)))
            exit(2)
        if useWebsocket and certificatePath and privateKeyPath:
            rospy.logwarn(
                "[ERROR][remote_sonsole]Mqtt::_init_mqtt_client X.509 cert authentication and WebSocket are mutual exclusive. Please pick one.")
            exit(2)
        if not useWebsocket and (not certificatePath or not privateKeyPath):
            rospy.logwarn("[ERROR][remote_sonsole]Mqtt::_init_mqtt_client Missing credentials for authentication.")
            exit(2)

        if useWebsocket:
            port = 443
        if not useWebsocket:
            port = 8883

        # Init AWSIoTMQTTClient
        self.myAWSIoTMQTTClient = None
        if useWebsocket:
            self.myAWSIoTMQTTClient = AWSIoTMQTTClient(
                self.__client_id, useWebsocket=True)
            self.myAWSIoTMQTTClient.configureEndpoint(host, port)
            self.myAWSIoTMQTTClient.configureCredentials(rootCAPath)
        else:
            self.myAWSIoTMQTTClient = AWSIoTMQTTClient(self.__client_id)
            self.myAWSIoTMQTTClient.configureEndpoint(host, port)
            self.myAWSIoTMQTTClient.configureCredentials(
                rootCAPath, privateKeyPath, certificatePath)

        # AWSIoTMQTTClient connection configuration
        self.myAWSIoTMQTTClient.configureAutoReconnectBackoffTime(1, 32, 20)
        # Infinite offline Publish queueing
        self.myAWSIoTMQTTClient.configureOfflinePublishQueueing(-1)
        self.myAWSIoTMQTTClient.configureDrainingFrequency(2)  # Draining: 2 Hz
        self.myAWSIoTMQTTClient.configureConnectDisconnectTimeout(10)  # 10 sec
        self.myAWSIoTMQTTClient.configureMQTTOperationTimeout(5)  # 5 sec
