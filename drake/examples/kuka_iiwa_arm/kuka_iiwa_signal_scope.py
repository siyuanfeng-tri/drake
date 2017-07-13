addPlot(timeWindow=5, yLimits=[-3.14, 3.14])
addSignals('IIWA_STATUS', msg.utime, msg.joint_position_measured, [1])
addSignals('IIWA_COMMAND', msg.utime, msg.joint_position, [1])
addSignals('IIWA_STATUS', msg.utime, msg.joint_position_commanded, [1])

