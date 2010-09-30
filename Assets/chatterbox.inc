# Define a model based on the Autonomy Lab Chatterbox robot
#
# Author: Richard T Vaughan (rtv) 
# $Id: chatterbox.inc,v 1.5 2008-01-15 01:25:42 rtv Exp $

include "irobot.inc"
# optionally include a laser
include "sick.inc"

define chatterbox create
(
  # Create battery NiMh battery is 3000 mAh @ 14.4V
  joules_capacity 155520
  # Create runs constantly for 1.5 hrs with no accessories => 30 W
  # Gumstix Connex draws 60mA @ idle, 220mA @ max load (@4.5V) =>say 1W 
  # Baseboard draws maybe 200mA @ 5V => 1W
  # Tri colour LEDs: 5 LEDs at 20mA @ 2.0V => 2W
  watts 35
  odom_error [ 0.01 0.01 0.00 0.13 ]

  # charging connection
  model
  (
    pose [0.100 0.000 -0.090 0.000]
    take_watts 1000.0
    size [0.15 0.1 0.03]
    color "yellow"
    obstacle_return 0
  )

  # long range IRs
  ranger
  (
    pose [0.0 0.0 0.06 0.0]
    scount 6
    spose[0] [ 0.000  0.000   0]	# front forward
    spose[1] [ 0.000  0.080  30]	# front left side
    spose[2] [-0.054  0.086  90]	# left side
    spose[3] [-0.115  0.000 180]	# back backwards
    spose[4] [-0.054 -0.086 -90]	# right side
    spose[5] [ 0.000 -0.080 -30]	# front right side
    sview [ 0.2 1.5 20 ]
    ssize [0.01 0.03 ]
    watts 1.5 # 6 sensors at 5V AT 50mA max 
  )

# detect charging stations: need two to deal with different heights
#  fiducial
#  (
#    pose [0.000 0.000 -0.007 0.000]
#    range_min 0.0
#    range_max 3.0
#    range_max_id 3.0	
#    fov 30 
#    watts 10
#  )
  fiducial
  (
    pose [0.040 0.000 -0.005 0.000]
    range_min 0.0
    range_max 2.5
    range_max 2.5
    fov 60 
  )
  # optionally include a laser
  sicklaser()
)
