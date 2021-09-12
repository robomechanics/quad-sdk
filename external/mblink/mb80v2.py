from __future__ import print_function
import numpy as np
import time
from grmblinkpy import MBLink

def savePKZ(f1, data, mb80log=True):
    """
    Save a data log to a file. This is called automatically on exit to save a primitive log.

    Saves as a compressed pickle pkz file with filename: f1 + 'mb80' + timestamp.
    """
    
    if data is None:
        return
    t = time.localtime()
    timestamp = time.strftime('%Y%m%d%H%M%S', t)
    import gzip, pickle
    # pickle.dump(data, open(fname+'.pkl', 'wb'))
    # New: compressed pickle
    fname = f1
    if mb80log:
        fname += 'mb80'
    fname += timestamp + '.pkz'
    zfile = gzip.GzipFile(fname, 'wb')
    pickle.dump(data, zfile)
    zfile.close()
    print('Saved log in', fname)
    return fname


class MB80v2(MBLink):
    diskList = []
    diskSentCount = 0
    def __init__(self, sim=False, verbose=True, log=False):
        MBLink.__init__(self) # must call this
        self.start(sim, verbose)
        self.rxstart()
        self.log = log
        self.floatArraySendCounter = 0
        if log:
            self.data = {}
    
    def _appendRow(self, row):
        # Append the new row
        for key in row:
            # initialize an array with key
            if key not in self.data.keys():
                self.data[key] = []
            self.data[key].append(row[key])
    
    def _finalizeLog(self):
        # first turn the lists of arrays to 2D arrays
        for key in self.data:
            self.data[key] = np.array(self.data[key])
        fname = savePKZ('logs/', self.data)
        self.data = {}
        return fname
    
    def get(self):
        res = super(MB80v2, self).get() # need the super(args) for python2
        res['t'] = res['y'][-1]
        res['y'] = res['y'][:-1]
        if self.log:
            self._appendRow(res)
        return res

    def rxstop(self):
        super(MB80v2, self).rxstop()
        if self.log:
            return self._finalizeLog()

    def sendFloatArrayUpdate(self, minDecimation=10, **kwargs):
        """Send steppable, free (see BiresGaitPlanner). Manages its own state in order to stagger and sequence the sending. Call this at the control rate"""
        def prepareGoalSteppables(steppable=[], goal=[0,0,0], **kwargs):
            if len(steppable) == 0:
                return None
            assert len(goal) == 3 # must be for SE2
            goal = np.array(goal)
            
            def steppableArray(availableFloats, steppable):
                PATCHSZ = 9 # 9 floats to send a patch: 8*xy + z
                Ntosend = min(len(steppable), availableFloats//PATCHSZ)
                return Ntosend, np.hstack([np.hstack((np.reshape(steppable[i][:,:2], 8, 'C'), [np.mean(steppable[i][:,2])])) for i in range(Ntosend)])
            Nsteppables, steppables = steppableArray(55, steppable)

            # create the full message
            data = np.hstack((goal, steppables, np.zeros(55 - len(steppables))))
            return [1, 0, Nsteppables, data]
        
        def prepareDiskList(obsts=[], **kwargs):
            if len(obsts) == 0:
                return None
            if len(self.diskList) == 0:
                # grab the arguments. for now assumes it does not change after this
                for obst in obsts:
                    # assume each is a ConvexObstacle
                    self.diskList = self.diskList + obst.xyrs
            # TODO: sequence
            XYRSIZE = 3
            Ntosend = min(len(self.diskList) - self.diskSentCount, 58//XYRSIZE)
            # came as a list of xyr, so just concatenate them together
            diskArr = np.hstack((self.diskList[self.diskSentCount:self.diskSentCount+Ntosend]))
            self.diskSentCount += Ntosend
            # if they have all been sent, go back to the beginning
            if self.diskSentCount >= len(self.diskList):
                self.diskSentCount = 0
            data = np.hstack((diskArr, np.zeros(58 - len(diskArr))))
            return [2, self.diskSentCount, self.diskSentCount + Ntosend, data]

        if self.floatArraySendCounter == 0:
            tosend = prepareGoalSteppables(**kwargs)
            if tosend is not None:
                self.sendToSim(*tosend)
        elif self.floatArraySendCounter == 5:
            tosend = prepareDiskList(**kwargs)
            if tosend is not None:
                self.sendToSim(*tosend)
        
        self.floatArraySendCounter = (self.floatArraySendCounter + 1) % minDecimation

    
    def test(self):
        self.sendBehavior(MBLink.STAND, 0)
        time.sleep(5)

        # Stand - send it once more at top of stand to enter lookaround mode, ready for WALK FIXME: send twice is correct?
        self.sendBehavior(MBLink.STAND, 0)
        time.sleep(2)

        # Enter walk mode
        self.sendBehavior(MBLink.WALK, 1)# WALK, see: http://ghostrobotics.gitlab.io/SDK/MAVLinkReference.html
        time.sleep(1)

        print(self.rxdata)

        # Walk forward
        self.sendSE2Twist([0.2, 0, 0])
        time.sleep(3)
        # Walk backwards
        self.sendSE2Twist([-0.2, 0.0, 0.0])
        time.sleep(3)
        # Turn left
        self.sendSE2Twist([0,0,0.4])
        time.sleep(3)
        # Turn right
        self.sendSE2Twist([0.0, 0.0, -0.4])
        time.sleep(3)

        # Sit
        self.sendBehavior(MBLink.SIT, 0)
        