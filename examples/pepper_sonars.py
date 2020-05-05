#!/usr/bin/env python
# coding: utf-8

import pybullet
import pybullet_data
from qibullet import PepperVirtual
from qibullet import SimulationManager


def main():
    simulation_manager = SimulationManager()
    client = simulation_manager.launchSimulation(gui=True)
    pepper = simulation_manager.spawnPepper(client, spawn_ground_plane=True)

    pybullet.setAdditionalSearchPath(pybullet_data.getDataPath())

    pybullet.loadURDF(
        "duck_vhacd.urdf",
        basePosition=[1, -1, 0.5],
        globalScaling=10.0,
        physicsClientId=client)

    pepper.showSonar(True)
    pepper.subscribeSonar()
    pepper.goToPosture("Stand", 0.6)

    while True:
        sonar_list = pepper.getRightSonarValue()
        sonar_list.extend(pepper.getFrontSonarValue())
        sonar_list.extend(pepper.getLeftSonarValue())

        if all(sonar == 5.6 for sonar in sonar_list):
            print("Nothing detected")
        else:
            print("Detected")
            pass


if __name__ == "__main__":
    main()
