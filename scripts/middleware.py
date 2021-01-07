#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import pika
import uuid
import rospy
import json
from std_msgs.msg import Float32MultiArray, MultiArrayLayout, MultiArrayDimension, Int32MultiArray
from std_msgs.msg import String


class RabbitConnection:
    def send(self, msg):
        print("Got it {}".format(msg))
        return {"linear":1, "angular":2, "arm1":3, "arm2":4, "front":5, "rear":6}


class RPCClient:
    def __init__(self):
        self.connection = pika.BlockingConnection(pika.ConnectionParameters(host='localhost'))
        self.channel = self.connection.channel()

        result = self.channel.queue_declare(queue='', exclusive=True)

        self.callback_queue = result.method.queue

        self.channel.basic_consume(
            queue=self.callback_queue,
            on_message_callback=self.on_response,
            auto_ack=True
        )
        self.responses = {}

    def on_response(self, ch, method, props, body):
        if props.correlation_id in self.responses.keys():
            self.responses[props.correlation_id] = body

    def call(self, m):
        corr_id = str(uuid.uuid4())
        self.responses[corr_id] = None
        self.channel.basic_publish(
            exchange='',
            routing_key='rpc_queue',
            properties=pika.BasicProperties(
                reply_to=self.callback_queue,
                correlation_id=corr_id,
            ),
            body=json.dumps(m)
        )
        while self.responses[corr_id] is None:
            self.connection.process_data_events()
        ret = self.responses[corr_id]
        del self.responses[corr_id]
        return json.loads(ret)

class Middleware:
    def __init__(self):
        rospy.init_node("middleware")
        self.conn = RPCClient()
        rospy.Subscriber('/jaguar/arm_cmd', Float32MultiArray, self.handle_arm, queue_size=1)
        rospy.Subscriber('/cmd_array', Float32MultiArray, self.handle_array, queue_size=1)
        rospy.Subscriber('/robot/command', String, self.handle_cmd, queue_size=1)
        self.pub_state = rospy.Publisher('/robot/state', Float32MultiArray)
        self.msg = Float32MultiArray()
        self.msg.layout.dim = [MultiArrayDimension(label="state", size=6, stride=6)]
        rospy.spin()

    def decorate_send(func):
        def inner(self, msg):
            m = func(self, msg)
            state = self.conn.call(m)
            self.distribute_robot_state(state)
        return inner

    @decorate_send
    def handle_cmd(self, msg):
        m = {
            "cmd": msg.data,
        }
        return m

    @decorate_send
    def handle_arm(self, msg):
        joint1, joint2 = msg.data
        m = {
            "arm1": joint1,
            "arm2": joint1,
        }
        return m

    @decorate_send
    def handle_array(self, msg):
        linear, angular, dfront, drear = msg.data
        m = {
            "linear": linear,
            "angular": angular,
            "front": linear,
            "rear": angular,
        }
        return m

    def distribute_robot_state(self, st):
        state = [st["linear"], st["angular"], st["front"], st["rear"], st["arm1"], st["arm2"]]
        self.msg.data = state
        self.pub_state.publish(self.msg)

if __name__ == "__main__":
    Middleware()
