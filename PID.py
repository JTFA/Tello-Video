import numpy as np
import time


class PID:
    def __init__(self):
        self.Yaw_vel_P = 2.0
        self.Yaw_vel_I = 0.0
        self.Yaw_vel_D = 0.0

        self.Roll_vel_P = 2.0
        self.Roll_vel_I = 0.0
        self.Roll_vel_D = 0.0

        self.Pitch_vel_P = 2.0
        self.Pitch_vel_I = 0.0
        self.Pitch_vel_D = 0.0

        self.Throttle_vel_P = 2.0
        self.Throttle_vel_I = 0.0
        self.Throttle_vel_D = 0.0

        self.Pitch_error_last = 0.0
        self.Roll_error_last = 0.0
        self.Yaw_error_last = 0.0
        self.Throttle_error_last = 0.0

        self.Roll_error_last_time = time.time()*1000
        self.Yaw_error_last_time = time.time()*1000
        self.Pitch_error_last_time = time.time()*1000
        self.Throttle_error_last_time = time.time()*1000

    def get_vel_forward(self,error):
        '''
        :param error in meters
        :return: control value -100-100 in forward flight,
                positive means forward
        '''
        dt = time.time()*1000 - self.Pitch_error_last_time
        self.Pitch_error_last_time = time.time()*1000
        control = int(self.Pitch_vel_P*error +
                      self.Pitch_vel_I*(error + self.Pitch_error_last)*dt+
                      self.Pitch_vel_D*(error-self.Pitch_error_last)/dt)
        return int(control)

    @accepts(error=int)
    def get_vel_roll(self,error):
        '''
            :param error in meters
            :return: control value -100-100 in left flight,
                    positive means forward
        '''
        dt = time.time() * 1000 - self.Roll_error_last_time
        self.Roll_error_last_time = time.time() * 1000
        control = int(self.Roll_vel_P * error +
                      self.Roll_vel_I * (error + self.Roll_error_last) * dt +
                      self.Roll_vel_D * (error - self.Roll_error_last) / dt)
        return int(control)

    @accepts(error=int)
    def get_vel_yaw(self, error):
        '''
            :param error in meters
            :return: control value -100-100 in yaw flight,
                    positive means forward
        '''
        dt = time.time() * 1000 - self.Yaw_error_last_time
        self.Yaw_error_last_time = time.time() * 1000
        control = int(self.Yaw_vel_P * error +
                      self.Yaw_vel_I * (error + self.Yaw_error_last) * dt +
                      self.Yaw_vel_D * (error - self.Yaw_error_last) / dt)
        return int(control)

    @accepts(error=int)
    def get_vel_yaw(self, error):
        '''
            :param error in meters
            :return: control value -100-100 in Throttle flight,
                    positive means forward
        '''
        dt = time.time() * 1000 - self.Throttle_error_last_time
        self.Throttle_error_last_time = time.time() * 1000
        control = int(self.Throttle_vel_P * error +
                      self.Throttle_vel_I * (error + self.Throttle_error_last) * dt +
                      self.Throttle_vel_D * (error - self.Throttle_error_last) / dt)
        return int(control)
