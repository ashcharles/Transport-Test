# Desc: Charging station to work with the chatterbox model
# Author: Ash Charles
# Date: 26 Febuary 2009

define charge_station model
(  
  size [ 0.040 0.200 0.100 ]
  color "gray70"
  
  # base pad
  model(
	color "gray50"
	size [0.10 0.20 0.001]
	pose [-0.070 0 -0.1 0]
	obstacle_return 0
  )
  # IR beacons: Red=Port=Left; Right=Starboard=Green; Purple=Omni
  model(
	color "red"
	size [0.010 0.010 0.010]
	pose [-0.016 0.010 -0.015 -10 ]
	)
  model(
	color "green"
	size [0.010 0.010 0.010]
	pose [-0.016 -0.010 -0.015 10 ]
	)
  model(
	color "purple"
	size [0.030 0.030 0.030]
    pose [-0.020 0.000 -0.015 0.000]
    fiducial_return 6
    )

  # Positive Terminal of Charging Block 
  model( 
    pose [ -0.040 0.000 -0.100 0 ]
    color "yellow"
    size [ 0.020 0.100 0.005 ]
    joules -1  # provides infinite energy
    give_watts 1000 
	obstacle_return 0
   )
  
  # Negative terminal of the charging block
#  model( 
#    pose [ -0.040 -0.040 -0.100 0 ]
#    color "yellow"
#    size [ 0.020 0.020 0.005 ]
#   )
)
