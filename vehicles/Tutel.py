#!/usr/bin/env python  

__author__ = 'Raphael Leber'

from math import cos, sin, tan, pi, sqrt, atan2, fmod, fabs, copysign
import turtle  
import time

from vehicles.Vehicle import Vehicle
from PathTools import PathTools



class Tutel(Vehicle):

    def __init__(self, x=0, y=0, theta=0, Ks=16, Kv=1, T=0.35, dt = 0.003):
        self.x = x          # x position
        self.y = y          # y position
        self.theta = theta  # theta orientation
        self.Ks = Ks        # Steering proportionnal coefficient
        self.Kv = Kv        # Velocity proportionnal coefficient
        self.T = T          # Distance between left and right wheel
        self.dt = dt                # Time of one code cycle
        self.throttle_min = 400     # Min saturation throttle based of euclidan distance
        self.throttle_max = 1000    # Max saturation throttle based of euclidan distance




    def model(self, tL, tR):
        """  Tutel model
                           ___________________
                          |                   |
                          |                   |---> x           
            throttleR --->|                   |    
                          |       Tutel       |
                          |                   |---> y
                          |       Model       |                        
            throttleL --->|                   |   
                          |                   |---> theta      
                          |___________________|          
        """       

        track = self.T 
        dt = self.dt

        vL = min(tL, self.throttle_max)
        vR = min(tR, self.throttle_max)
   

        v = (vL + vR ) / 2      # Linear speed
        w = (vR - vL) / track   # Angular speed

        x_p = v * cos(self.theta)
        y_p = v * sin(self.theta)
        theta_p = w

        self.x = self.x + x_p*dt
        self.y = self.y + y_p*dt
        self.theta = self.theta + theta_p*dt

        return self.x, self.y, self.theta



    def toPoint(self, x, y, eps=0.5):
        """ Move to a point (x, y)

            Parameters:
            x (int): x coordinate to reach in the map
            y (int): y coordinate to reach in the map
            eps (float): epsilon below which we consider the goal is reached

            Returns:
            float: Returning throttle value (corresponding to euclidian distance vehicle-->goal) 

        """        
        Ks = self.Ks
        Kv = self.Kv          
        eucli_throttle = eps
        while eucli_throttle >= eps:

            eucli_throttle = sqrt( (x-self.x)**2 + (y-self.y)**2 )
            steering = atan2(  (y-self.y)  , (x-self.x) )

            v = max(min(eucli_throttle * Kv, self.throttle_max), self.throttle_min)
            s = sin(PathTools().shortestAngleDiff(steering, self.theta)) * Ks


            vL = v - s
            vR = v + s

            self.model(vL, vR)

            yield eucli_throttle

    def turn(self, phi, eps_angle=0.1):
        Ks = self.Ks
        Kv = self.Kv
        s = 1

        while fabs(s) > eps_angle:

            s = PathTools().shortestAngleDiff(phi, self.theta) 

            self.model(-s, s)

            yield s 


    
    def toPose(self, x, y, theta):
        pass





    