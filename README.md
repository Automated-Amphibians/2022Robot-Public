This code mostly works during autonomous and teleoperated period. The robot moves forward and backward both in autonomous and teleop pretty straight. But you have to rebuild 
and redeploy the robot everytime you want to switch between modes (autonomous/teleop). You still can't go forward/backward and right at the same time. This might be because the 
left encoder is not aligned properly and the leftMaster is hotter than the rest. This might also be because the left motors can't seem to spin much faster than the right side, 
so it can't turn right while moving foward or backward.
