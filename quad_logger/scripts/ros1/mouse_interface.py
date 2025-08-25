#!/usr/bin/python
import matplotlib.pyplot as plt
import numpy as np

plt.rcParams['lines.linewidth'] = 4
plt.rcParams['xtick.labelsize'] = 14
plt.rcParams['ytick.labelsize'] = 14

ax = {}

o_X_ = {}
o_Y_ = {}
o_xlim_ = {}
o_ylim_ = {}
scalex_ = {}
scaley_ = {}
coords = {} 
prev_button_ = None
shift_hold = False

def clear():
    global verticalLineX_
    global cur_xlim_
    global cur_ylim_
    verticalLineX_ = None

    cur_xlim_ = None
    cur_ylim_ = {}

clear()

def plot_data(data, variables_selected, topics_selected):
    global n_lines
    global verticalLineX
    fig = plt.figure(1)
    plt.clf()
    n_lines = len(topics_selected)
    n_plot = len(variables_selected)
    k = 1
    for var in variables_selected:
        ax[var] = plt.subplot(n_plot, 1 , k)
        ax[var].hold(True)
        ax[var].grid(True)
        plt.xlabel("time(s)", fontsize=20)
        if(var=="vx" or var=="vy" or var=="vz"):
            plt.ylabel(var+"(m/s)", fontsize=20)
        elif(var=="roll" or var=="pitch" or var=="yaw"):
            plt.ylabel(var+"(rad)", fontsize=20)
        else:
            plt.ylabel(var, fontsize=20)

        k += 1
        for topic in topics_selected:
            if var in data[topic]:
             ax[var].plot(data[topic]['t']-data[topic]['t'][0], data[topic][var], label=topic+"/"+var)
             ax[var].legend(bbox_to_anchor=(0.3, 0.9))
        if not verticalLineX_ == None:
             drawVerticalLine(ax[var])
        if not cur_xlim_ == None:
             drawZoomX(ax[var], cur_xlim_)
        if var in cur_ylim_:
             drawZoomY(ax[var], cur_ylim_[var])
    fig.canvas.mpl_connect('button_press_event', onClick)
    fig.canvas.mpl_connect('scroll_event', reZoom)
    fig.canvas.mpl_connect('key_press_event', turnOnKey)
    fig.canvas.mpl_connect('key_release_event', turnOffKey)
    fig.canvas.mpl_connect('motion_notify_event', dragZoom)
    fig.canvas.mpl_connect('button_press_event', dragZoom)
    fig.canvas.mpl_connect('button_release_event', dragZoom)

def save():
    #fig = plt.figure(1)
    plt.savefig('plot.png')

def turnOnKey(event):
    global shift_hold
    if event.key == 'shift':
        shift_hold = True
    else:
        shift_hold = False
def turnOffKey(event):
    global shift_hold
    if event.key == 'shift':
        shift_hold = False


def drawVerticalLine(axe):
      if len(axe.lines) > n_lines:
            axe.lines[-1].remove()
      global verticalLineX_
      axe.axvline( x = verticalLineX_, color='r')

def onClick(event):
   if event.button == 3:
    for axe in ax.values():
      if len(axe.lines) > n_lines:
            axe.lines[-1].remove()
      global verticalLineX_
      verticalLineX_ = event.xdata
      axe.axvline( x = verticalLineX_, color='r')
   event.canvas.draw()

def drawZoomX(axe, xlim):
    axe.set_xlim(xlim)

def drawZoomY(axe, ylim):
    axe.set_ylim(ylim)
  

def reZoom(event, base_factor = 1.5):
    global shift_hold
    global cur_xlim_
    global cur_ylim_
    #for axe in event.canvas.figure.axes:
    for axe_id, axe in ax.items():
      if hasattr(event, 'button') and event.inaxes:
        if event.button == 'up':
             scale = 1/base_factor
        elif event.button == 'down':
             scale = base_factor
        else:
             scale = 1
             return
        if shift_hold:
            coords = axe.transAxes.transform([(0,0), (1,1)])
            if checkAxes(event.x, event.y, coords):
             cur_ylim = axe.get_ylim()
             cur_yrange = (cur_ylim[1] - cur_ylim[0])*.5
             cur_y = event.ydata
 
             axe.set_ylim([cur_y - cur_yrange * scale,
                      cur_y + cur_yrange * scale])
        else:
             cur_xlim = axe.get_xlim()
             cur_xrange = (cur_xlim[1] - cur_xlim[0])*.5
             cur_x = event.xdata
    
             axe.set_xlim([cur_x - cur_xrange * scale,
                           cur_x + cur_xrange * scale])
        cur_xlim_ = axe.get_xlim()
        cur_ylim_[axe_id] = axe.get_ylim()
    
    event.canvas.draw()

def checkAxes(x, y, coords):
    if  x < coords[1][0] and x > coords[0][0] and \
        y < coords[1][1] and y > coords[0][1]:
        return True
    else:
        return False

def dragZoom(event):
    
    if event.inaxes and event.button == 1:
        global prev_button_
        global o_X_
        global o_Y_
        global o_xlim_
        global o_ylim_
        global scalex_
        global scaley_
        global cur_xlim_
        global cur_ylim_
        for key, axe in ax.items():
            if not event.button == prev_button_:
                o_xlim_[key] = axe.get_xlim()
                o_ylim_[key] = axe.get_ylim()
           
                coords[key] = axe.transAxes.transform([(0,0), (1,1)])
                scalex_[key] = (o_xlim_[key][1] - o_xlim_[key][0]) / (coords[key][1][0] - coords[key][0][0])
                scaley_[key] = (o_ylim_[key][1] - o_ylim_[key][0]) / (coords[key][1][1] - coords[key][0][1])
 
                o_X_[key] = event.x
                o_Y_[key] = event.y
                continue
            cur_X = event.x
            cur_Y = event.y
            dx = (cur_X - o_X_[key]) * scalex_[key]
            axe.set_xlim([o_xlim_[key][0] - dx, o_xlim_[key][1] - dx])
            if  checkAxes(cur_X, cur_Y, coords[key]):
                dy = (cur_Y - o_Y_[key]) * scaley_[key]
                axe.set_ylim([o_ylim_[key][0] - dy, o_ylim_[key][1] - dy])
            cur_xlim_ = axe.get_xlim()
            cur_ylim_[key] = axe.get_ylim()
        event.canvas.draw()
    prev_button_ = event.button

if __name__ == "__main__":
    plot_data({'1':{'1':[1,2,3], '2':[1,2,3], 't':[1,2,3]}, '2':{'1':[3,2,1], '2':[3,2,1], 't':[1,2,3]}}, ['1', '2'],['1','2'])
    plt.show()
