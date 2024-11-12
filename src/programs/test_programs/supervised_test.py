from monke_hat import Car
from component_params import *

import numpy as np
from sklearn.model_selection import train_test_split
from sklearn.tree import DecisionTreeRegressor
from sklearn.metrics import mean_squared_error
import joblib

car = Car.Car(
    camera=camera,
    wheelBase=wheelBase,
    servo=servo,
    us_front=us4,
    us_left=us2,
    us_right=us5,
    us_spare1=us1,
    us_spare2=us3,
    rgb=rgb,
    pb=pb,
    mDrvr=mDrvr
)

# Initialize and train the Decision Tree Regressor
loaded_model = joblib.load('trained_model.joblib')
dt_regressor = loaded_model


start = False
curr_but_val = False
prev_but_val = False

car.compass.set_home()



while True:  

    front, left, right, compass = car.read_sensors()
    driving_direction = 0
    dist = 1

    data = [[front, left, right, compass, driving_direction, dist]]

    prev_but_val = curr_but_val
    curr_but_val = car.read_button()

    if curr_but_val and not prev_but_val:
        start = not start

    if start:

        car.LED.rgb(0,100,0)

        prediction = dt_regressor.predict(data)
        ang, speed = prediction[0]

        car.servo.write(ang)
        car.motor.speed(speed)

        print(f"front: {front} left: {left} right: {right} compass: {compass} Decision: {ang}, {speed}")

    else:

        car.LED.rgb(0,0,100)
        car.servo.write(0)
        car.motor.speed(0)