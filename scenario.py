#!/usr/bin/env python  

__author__ = 'Raphael Leber'

# from vehicles.Bicycle import Bicycle
from vehicles.Tutel import Tutel
from VehicleSimulator import *

# create simulator instance
vs = VehicleSimlulator()

# create vehicle instance
# bicycle = Bicycle()
tutel = Tutel()

# Choose one particular vehicle
# vs.selectVehicle(bicycle)
vs.selectVehicle(tutel)

# --- SCENARIO ---
vs.toPoint( *BLUE )
vs.toPoint( *GREEN )
vs.toPoint( *PURPLE )
vs.turn( pi/4 )

# End of Scenario
vs.frame.mainloop()

