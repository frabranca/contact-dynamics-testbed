import lcm
from math import *
from exlcm import command, state
import time
import select
import numpy as np

def my_handler(channel, data):
    msg = command.decode(data)
    # print("Received message on channel \"%s\"" % channel)
    # # Measured joint position. Unit: [rad]
    print("   tau_J_d      = %s" % str(msg.tau_J_d))
    # print("   K_P      = %s" % str(msg.K_P))    
    # print("   K_D      = %s" % str(msg.K_D)) 


# START LCM

# SUBSCRIBE TO CHANNEL

#DEFINE MESSAGE
#PUBLISH ON CHANNEL
lc = lcm.LCM()

# try:
timeout = 0.001
start_time = time.time()
while True:
	# SEND INITIAL STATE
	print(time.time())
	state().q       = np.array(command().tau_J_d) - (time.time())
	state().q_d     = np.array(command().tau_J_d) - (time.time())
	state().dq      = np.array(command().tau_J_d) - (time.time())
	state().dq_d    = np.array(command().tau_J_d) - (time.time())
	state().ddq_d   = np.array(command().tau_J_d) - (time.time())
	state().tau_J   = np.array(command().tau_J_d) - (time.time())
	state().tau_J_d = np.array(command().tau_J_d) - (time.time())
	state().dtau_J  = np.array(command().tau_J_d) - (time.time())

	lc.publish("STATE", state().encode())
	print("STATE SENT !!!")

	# SUBSCRIBE TO GET ORDERS
	subscription = lc.subscribe("COMMAND", my_handler)

	# IF A MESSAGE IS RECEIVED HANDLE THE MESSAGE
	rfds, wfds, efds = select.select([lc.fileno()], [], [], timeout)
	if rfds:
		print("MESSAGE RECEIVED !!!")
		lc.handle()
	else:
		print('waiting...')
	#time.sleep(2.)

# except KeyboardInterrupt:
#     pass