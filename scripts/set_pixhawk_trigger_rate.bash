#!/bin/bash

fps=$1
interval=`bc -l <<< "scale=2; 1000.0/$fps"`
duration=5.0

sleep $2
echo "setting strobe interval to $interval, duration to $duration"
rosservice call /mavros/cmd/command "{broadcast: false, command: 214, confirmation: 0, param1: $interval, param2: $duration, param3: 0.0, param4: 0.0, param5: 0.0, param6: 0.0, param7: 0.0}"
rosservice call /mavros/cmd/command "{broadcast: false, command: 246, confirmation: 0, param1: 1, param2: 0.0, param3: 0.0, param4: 0.0, param5: 0.0, param6: 0.0, param7: 0.0}"
