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

    pepper.showBumper(True)
    pepper.subscribeBumper()
    pepper.goToPosture("Stand", 0.6)

    while True:
        bumper_list = pepper.getRightBumperValue()
        bumper_list.extend(pepper.getFrontBumperValue())
        bumper_list.extend(pepper.getLeftBumperValue())

        if all(bumper == 5.6 for bumper in bumper_list):
            print("Nothing detected")
        else:
            print("Detected")
            pass


if __name__ == "__main__":
    main()
