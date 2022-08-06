#!/usr/bin/env python3

import rospy
from manipulator_gazebo.srv import *
from std_msgs.msg import Float64
from sensor_msgs.msg import JointState
import math
import time
import ipfshttpclient
import rospkg
import json
import subprocess
import re
from web3 import Web3, HTTPProvider

def _check_liability_contract(path, config):
    http_provider = HTTPProvider("https://proxy.devnet.neonlabs.org/solana")
    web3 = Web3(http_provider)
    with open(f"{path}/liability/abi/Liability.json") as f:
        abi = json.loads(f.read())    
    liability_address = config["liability_contract"]
    liability = web3.eth.contract(liability_address, abi=abi)
    model = liability.call().model()
    objective = liability.call().objective()
    return model, objective


class Kuka:
    def __init__(self):
        rospy.init_node('listener', anonymous=False)
        rospack = rospkg.RosPack()
        rospack.list()
        self.path = rospack.get_path('kuka_controller')
        with open(f"{self.path}/config/config", "r") as f:
            self.config = json.load(f)
        self.client = ipfshttpclient.connect()

    # Call service move_arm
    def move_arm_client(self, desired_xyz, duration):
        rospy.wait_for_service('move_arm')
        try:
            move_arm = rospy.ServiceProxy('move_arm', MoveArm)
            resp = move_arm(desired_xyz, duration)
            return resp
        except rospy.ServiceException as e:
            rospy.loginfo("Service call failed: %s"%e)

    # Write data to a file
    def listener(self, data):
        if self.write:
            times_prev = self.times
            self.times = int(time.time())
            if self.times != times_prev:
                #print('write')
                self.logs.write('\n')
                self.logs.write(str(data))

    
    def circle(self):
        t = 0
        self.logs = open(f'{self.path}/data.txt', 'w')
        self.move_arm_client([Float64(0.3), Float64(0.3), Float64(0.6)], Float64(2.0))
        self.times = 0
        self.write = True
        rospy.Subscriber('/manipulator/joint_states', JointState, self.listener)
        while t <= math.pi:
            x = 0.3*math.cos(t)
            z = 0.3*math.sin(t) + 0.6
            t += 0.2
            #print(x, z)
            self.move_arm_client([Float64(x), Float64(0.3), Float64(z)], Float64(0.05))
        self.write = False
        rospy.loginfo("Work done")
        self.logs.close()
        res = self.client.add(f'{self.path}/data.txt')
        rospy.loginfo(f"Data pinned to IPFS with hash {res['Hash']}")
        rospy.loginfo("Finalizing liability...")
        p = subprocess.Popen(["node", f"{self.path}/liability/result.js", self.path, res['Hash']], stdout=subprocess.PIPE)
        out = p.stdout.read()
        result_tx_hash = re.findall(r"0x\w*", str(out))
        rospy.loginfo(f"Finalized tx hash {result_tx_hash[0]}")

    def offer_sender(self):
        p = subprocess.Popen(["node", f"{self.path}/liability/msg.js", self.path], stdout=subprocess.PIPE)
        rospy.loginfo(f"Found new liability: {self.config['liability_contract']}")
        out = p.stdout.read()
        tx_hashes = re.findall(r"0x\w*", str(out))
        rospy.loginfo(f"Demand tx hash: {tx_hashes[0]}")
        rospy.loginfo(f"Offer tx hash: {tx_hashes[1]}")
        model, objective = _check_liability_contract(self.path, self.config)
        if model and objective:
            self.circle()
        else:
            rospy.loginfo("Model and objective are not valid! Abort.")


    def spin(self):
        rospy.loginfo(f"Sending offer")
        self.offer_sender()
    


if __name__ == "__main__":
    Kuka().spin()
        
