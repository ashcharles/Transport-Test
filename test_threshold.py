#!/usr/bin/python
import subprocess as sp
import numpy as np
import re

# Repeat simulation for different maps, different rangefinders
maps = ( 'open', 'clutter' )
rfs = ( 'ir', 'laser' )
repeats = 2	# number of trials at a particular threshold

srcworldpath = 'Assets/transport.world'
useworldpath = 'Assets/transport.world'

srcworld = open(srcworldpath, 'r')

for rf in rfs:
  for map in maps:
    for i in range(repeats):

      if map == 'open' and rf == 'laser':
        useworldpath = 'Assets/transport_ol.world'
      elif map == 'open' and rf == 'ir':
        useworldpath = 'Assets/transport_oi.world'
      elif map == 'clutter' and rf == 'laser':
        useworldpath = 'Assets/transport_cl.world'
      else:
        useworldpath = 'Assets/transport_ci.world'
      print( map, rf )
      # Now run Stage
      pStage = sp.Popen(['stage', '-g', useworldpath],
                        stdout=sp.PIPE, stderr=sp.PIPE)
      output = pStage.communicate()[0]
      match = re.search('Result: (\d+(\.\d*)?|\.\d+)', output)
      print( rf, map, i, match.group())
	
srcworld.close()
