from __future__ import print_function
import time
import numpy as np
import argparse
from mb80v2 import MB80v2

parser = argparse.ArgumentParser(description='MB80 comm template')
parser.add_argument('-s', '--sim', default=False, action='store_true', help='communicate locally to the sim instead of to the mb8')
parser.add_argument('-m', '--mocap', default=False, action='store_true', help='Use mocap (ignored if in sim)')
parser.add_argument('-l', '--log', default=False, action='store_true', help='Save mb80 log')
args = parser.parse_args()

print("Starting...")
mb = MB80v2(sim=args.sim, verbose=True, log=args.log)#useMocap=args.mocap)

# Set upstream IP address to 192.168.168.5
mb.setRetry('_UPST_ADDRESS', 255)

# Set upstream loop delay
mb.setRetry('UPST_LOOP_DELAY', 5)

while True:
    try:
        # This blocks till there is new data and does not hog CPU
        res = mb.get()
        # print(res.keys())
        
    except KeyboardInterrupt:
        mb.rxstop()
        break
    except:
        raise

