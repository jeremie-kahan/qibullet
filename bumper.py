#!/usr/bin/env python
# coding: utf-8

import time
import math
import pybullet
import threading
from qibullet.sensor import Sensor

RAY_MISS_COLOR = [0, 1, 0]
RAY_HIT_COLOR = [1, 0, 0]
NUM_RAY = 15
RAY_LENGTH = 0.2  # The theoretical length is 0.0
BUMPER_ANGLE = 0
BUMPER_POSITION = [
    [0.129, -0.227, -0.264],  # Front right bumper
    [0.129, 0.227, -0.264],  # Front left bumper
    [-0.251, 0.0, -0.264]  # Back bumper
]
ANGLE_LIST_POSITION = [
    math.radians(BUMPER_ANGLE/2) - 0.488692,  # Front right bumper
    math.radians(BUMPER_ANGLE/2) + 0.488692,  # Front left bumper
    math.radians(BUMPER_ANGLE/2) - 3.141593  # Back bumper
]

NUM_BUMPER = len(BUMPER_POSITION)
BUMPER_FRAMERATE = 6.25


class Bumper(Sensor):
    """
    Class representing a virtual bumper
    """

    def __init__(
            self,
            robot_model,
            bumper_id,
            physicsClientId=0,
            display=False):
        """
        Constructor

        Parameters:
            robot_model - The pybullet model of the robot.
            bumper_id - The id of the link (Link type)
            onto which the Bumpers' are attached.
            physicsClientId - The id of the simulated instance in which the
            bumpers are to be spawned
            display - boolean that allow the display of the bumper
        """
        Sensor.__init__(self, robot_model, physicsClientId)
        self.ray_from = []
        self.ray_to = []
        self.ray_ids = []
        self.bumper_value = [0] * NUM_RAY * NUM_BUMPER
        self.bumper_id = bumper_id
        self.display = display

    @classmethod
    def _getInstances(cls):
        """
        INTERNAL CLASSMETHOD, get all of the Bumper instances
        """
        dead = set()

        for ref in cls._instances:
            obj = ref()

            if obj is not None:
                yield obj
            else:
                dead.add(ref)

        cls._instances -= dead

    def isActive(self):
        """
        Check if the bumpers are subscribed or not (if the bumper thread process
        is active or not)

        Returns:
            boolean - True if the bumpers are subscribed, false otherwise
        """
        return self.module_process.isAlive()

    def subscribe(self):
        """
        Subscribe to the bumper scan (this will activate the bumper scan
        process).
        """
        # No need to subscribe to the bumper scan if the bumpers are activated
        if self.isActive():
            return

        self._module_termination = False
        self._initializeRays()
        self.module_process = threading.Thread(target=self._bumperScan)
        self.module_process.start()

    def unsubscribe(self):
        """
        Unsubscribe from the bumper scan (this will deactivate the bumper scan
        process)
        """
        if self.isActive():
            self._terminateModule()

    def showBumper(self, display):
        """
        Display debug lines that simulate the bumper
        """
        self.display = display

    def getFrontBumperValue(self):
        """
        Return a list of the front bumper value (clockwise)
        """
        return self.bumper_value[:NUM_RAY]

    def getRightBumperValue(self):
        """
        Return a list of the right bumper value (clockwise)
        """
        return self.bumper_value[NUM_RAY:2*NUM_RAY]

    def getLeftBumperValue(self):
        """
        Return a list of the left bumper value (clockwise)
        """
        return self.bumper_value[2*NUM_RAY:]

    def _initializeRays(self):
        """
        INTERNAL METHOD, initialize the bumper and all variables needed
        """
        for index in range(NUM_BUMPER):
            angle = ANGLE_LIST_POSITION[index]
            for i in range(NUM_RAY):
                self.ray_from.append([
                    BUMPER_POSITION[index][0],
                    BUMPER_POSITION[index][1],
                    BUMPER_POSITION[index][2]])
                self.ray_to.append(
                    [BUMPER_POSITION[index][0] + (RAY_LENGTH) *
                     math.cos(float(i) *
                     math.radians(-BUMPER_ANGLE)/NUM_RAY + angle),
                     BUMPER_POSITION[index][1] + (RAY_LENGTH) *
                     math.sin(float(i) *
                     math.radians(-BUMPER_ANGLE)/NUM_RAY + angle),
                     BUMPER_POSITION[index][2]])

    def _bumperScan(self):
        """
        INTERNAL METHOD, a loop that simulate the bumper and update the distance
        value of each bumper
        """
        lastBumperTime = time.time()

        while not self._module_termination:
            nowBumperTime = time.time()

            if (nowBumperTime-lastBumperTime > 1/BUMPER_FRAMERATE):
                results = pybullet.rayTestBatch(
                    self.ray_from,
                    self.ray_to,
                    parentObjectUniqueId=self.robot_model,
                    parentLinkIndex=self.bumper_id,
                    physicsClientId=self.physics_client)

                for i in range(NUM_RAY*len(ANGLE_LIST_POSITION)):
                    hitObjectUid = results[i][0]
                    hitFraction = results[i][2]
                    hitPosition = results[i][3]
                    self.bumper_value[i] = hitFraction * RAY_LENGTH

                    if self.display:
                        if not self.ray_ids:
                            self._createDebugLine()

                        if (hitFraction == 1.):
                            pybullet.addUserDebugLine(
                                self.ray_from[i],
                                self.ray_to[i],
                                RAY_MISS_COLOR,
                                replaceItemUniqueId=self.ray_ids[i],
                                parentObjectUniqueId=self.robot_model,
                                parentLinkIndex=self.bumper_id,
                                physicsClientId=self.physics_client)
                        else:
                            localHitTo = [self.ray_from[i][0]+hitFraction*(
                                        self.ray_to[i][0]-self.ray_from[i][0]),
                                         self.ray_from[i][1]+hitFraction*(
                                        self.ray_to[i][1]-self.ray_from[i][1]),
                                         self.ray_from[i][2]+hitFraction*(
                                        self.ray_to[i][2]-self.ray_from[i][2])]
                            pybullet.addUserDebugLine(
                                self.ray_from[i],
                                localHitTo,
                                RAY_HIT_COLOR,
                                replaceItemUniqueId=self.ray_ids[i],
                                parentObjectUniqueId=self.robot_model,
                                parentLinkIndex=self.bumper_id,
                                physicsClientId=self.physics_client)

                    else:
                        if self.ray_ids:
                            self._resetDebugLine()

                lastBumperTime = nowBumperTime

    def _createDebugLine(self):
        """
        INTERNAL METHOD, create all debug lines needed for simulating the
        bumpers
        """
        for i in range(NUM_RAY * NUM_BUMPER):
            self.ray_ids.append(pybullet.addUserDebugLine(
                self.ray_from[i],
                self.ray_to[i],
                RAY_MISS_COLOR,
                parentObjectUniqueId=self.robot_model,
                parentLinkIndex=self.bumper_id,
                physicsClientId=self.physics_client))

    def _resetDebugLine(self):
        """
        INTERNAL METHOD, remove all debug lines
        """
        for i in range(len(self.ray_ids)):
            pybullet.removeUserDebugItem(self.ray_ids[i], self.physics_client)

        self.ray_ids = []
