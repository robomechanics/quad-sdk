import time, sys
import numpy as np

# MB80Base is the "Mainboard 8.0 interface library".
sys.path.append('../../python')
from mb80base import MB80Base

# Start MAVLink interface
print("Starting...")
mb = MB80Base(local=False, verbose=True, useMocap=False)
mb.rxstart(savelog=False)

# Set fast upstream update rate (1000/freqHz)
mb.setRetry('UPST_LOOP_DELAY', 3)

"""Function to set all limb coord positions"""
def limbCmd(kp, kd, pos):

    hdr = 0 # uint64

    # Received om the mb in order (pos[3], Kp[3], Kd[3])x4
    data = np.zeros(58)

    # Populate with the provided data and send
    singleLimb = lambda pos3 : np.hstack((pos3, kp * np.ones(3), kd * np.ones(3)))
    data[:36] = np.hstack([singleLimb(pos[3*i:3*i+3]) for i in range(4)])
    mb.pctx.mav.debug_float_array_send(hdr, b'', mb.DEBUG_FLOAT_ARRAY_ID_USER, data=data)

# Start
time_start = time.time()
while True:
    try:
        # This blocks till there is new data in the queue. The rate can be set (to a limit) by UPST_LOOP_DELAY = 1000/freqHz
        res = mb.get()
		
        # Command repeating sit and stand positions to all 4 limbs.
        # limbCmd( P_gain, D_gain,
        #	[x, y, z,  # Front left limb
        #	 x, y, z,  # Back left limb
        #	 x, y, z,  # Front right limb
        #	 x, y, z]) # Back right limb
        # These are cartesian space positions of toes relative to x, y, z center of leg pod (intersection point of pod abduction and hip axis):
        # x: 0 (+ is toe forward)
        # y: -0.1 and 0.1 (+ is toe towards left) Note: If set to 0, it means toes will be directly underneath leg pod, so leg will aduct inwards when legs are retracted.
        # z: -0.1 to -0.5, sine wave over time (+ is toe up towards body, so -0.1 is sitting, -0.5 is robot standing at maximum Vision 60 robot height)
        time_elapsed = time.time() - time_start
        toeZ = -0.1 + 0.2 * (-1 + np.cos(2 * np.pi * 0.1 * time_elapsed))
        print("Toe z: " + str(round(toeZ, 2)) + 
              ", toe z received: " + str(round(res['user'][1], 2)) + 
              ", robot time: "     + str(round(res['user'][0] / 1000.0 / 1000.0, 1)) + 
              ", robot active: "   + str(round(res['user'][2])))

        limbCmd(2000, 50, [0,  0.1, toeZ,
                           0,  0.1, toeZ,
                           0, -0.1, toeZ,
                           0, -0.1, toeZ])

    except KeyboardInterrupt:
        mb.rxstop()
        break
