import dronekit
from pymavlink import mavutil
import cv2
import numpy as np
import time
import subprocess

vehicle = dronekit.connect('192.168.170.152:8000')

model_svm = cv2.ml.SVM_load('model_svm_own.xml')

def upload_mission():
    cmds = vehicle.commands
    cmds.clear()

    waypoints = [
        (30.3509341, 76.3622506, 10),
        (30.3509641, 76.3626234, 10),
        (30.3510914, 76.3625993, 10),
        (30.3510463, 76.3622573, 10),
        (30.3509479, 76.3622586, 10)
    ]

    for i, (lat, lon, alt) in enumerate(waypoints):
        cmd = vehicle.message_factory.command_long_encode(
            0, 0,
            mavutil.mavlink.MAV_CMD_NAV_WAYPOINT,
            0, 0, 0, 0, 0,
            lat, lon, alt
        )
        cmds.add(cmd)

    cmds.upload()

cap = cv2.VideoCapture(1)

face_detector = cv2.CascadeClassifier('haarcascade_frontalface_default.xml')

upload_mission()

vehicle.arm()
vehicle.mode = dronekit.VehicleMode("AUTO")

while True:
    ret, frame = cap.read()
    gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
    faces = face_detector.detectMultiScale(gray, scaleFactor=1.1, minNeighbors=5)

    if len(faces) > 0:
        vehicle.mode = dronekit.VehicleMode("GUIDED")
        vehicle.commands.velocity_body.x = 0.0
        vehicle.commands.velocity_body.y = 0.0
        subprocess.run(["python", "face_rec.py"]
    else:
        vehicle.mode = dronekit.VehicleMode("AUTO")

    cv2.waitKey(10)

cap.release()

vehicle.armed = False
time.sleep(2)

vehicle.close()