import numpy as np
import sys
import io
import pandas 
import plotly.express as px
import math 
import matplotlib.pyplot as plt


fin = open('Independent_File.in.txt', mode='r', encoding='utf-8-sig')

#read all the inputs in variables
k1 = float(fin.readline()) #gain #1
k2 = float(fin.readline()) #gain #2
k3 = float(fin.readline()) #gain #3
k4 = float(fin.readline()) #gain #4
x1 = float(fin.readline()) #initial vehicle position
x2 = float(fin.readline()) #initial vehicle velocity
x3 = float(fin.readline()) #initial wheel angular velocity
x2cmd = float(fin.readline())#command controller of vehicle velocity
tn = int(fin.readline()) #number of time steps
ts = float(fin.readline()) #time steps duration
controlTorque = fin.readline()#names what control torque is currently being used

#define all the variables
A21 = 29.6
A22 = 0.0000559
A31 = 1459.327
A32 = 0.0189
d3 = 14.3201
B = 0.0225
rw = 15.37/12.0
lpk = 0.17

xK=0
x3_k=0
x3_k1=0
alOLD=0
x2_k1=0

x2 = x2*5280/3600
x3 = x2/rw #(rw*(1-lpk))

x2cmd = (x2cmd*5280)/3600
if(controlTorque == 'Lyapunov Control Torque'):
    x3cmd = x2cmd/rw 
else:
    x3cmd = x2cmd/rw
    

def calc_al(l_val): 
    al_local = 2*(l_val/lpk)/(1+(l_val/lpk)**2)
    # -1<= a_local <=1
    al_local = max(-1, al_local)
    al_local = min(1,  al_local)
    
    return al_local
    
def calc_lamda(v, w):
    l_local = 0
   
    if ( v*w < 0 ): 
       print (v, w)
    if ( v < 0 ): 
        print(v)
       
    if (v<= rw*w):
        if(w !=0):
            l_local = 1-v/(rw*w)
    else: 
        if(v !=0):
            l_local = (rw*w)/v-1
    # -lpk<=l <=lpk
    l_local = min(l_local, lpk)
    l_local = max(l_local, -lpk)

    return l_local

def f(x):
    l_x = calc_lamda(x[1][0], x[2][0])
    al_x = calc_al(l_x)
    
    x1dot = x[1][0]
    x2dot = A21*al_x -A22*(x[1][0]**2)
    x3dot = (-A31*al_x)-A32*(x[1][0]**2)-d3+B*Tcon

    return np.array([[x1dot],[x2dot],[x3dot]])


x3Old =x3 #x3 one time step before



Tcon =0.0 #temporary control torque value
#if controlTorque=="Sliding Mode Control Torque":
    #Tcon = (1/B)*(((A31-k4*A21)*al) + ((A32+k4*A22)*(x2**2)) +d3-((k3*k4)*(x2-x2cmd))-((k3)*(x3-x3cmd)))#for Sliding Mode Control Torque
#else:
    #Tcon = (1/B)*(((A31-A21)*al) + ((A32+A22)*(x2**2)) +d3-((k3)*(x2-x2cmd)) + ((1/(tn*ts))*(x3-x3Old)))#for Lyapunov Control Torque

    #Performing the Runge-Kutta Method on Autonomous Vehicle Equations XD

x0= [[x1],[x2],[x3]]
x_values= []
y_values=[]
z_values=[]
Tcon_values=[]
time= []
x2Error = []
x3kError = []
x2MPH = []
x3MPH = []

for i in range(1,(tn+1)):
    
    l = calc_lamda(x2, x3)
    al = calc_al(l)#attenuation from wheel slip

    if(controlTorque == 'Lyapunov Control Torque'):
        Tcon = Tcon + (1/B)((A31*(al-alOLD) + A32 * ((x0[1][0]**2)-(x2_k1**2)) - k3*(x3_k-x3_k1) + (1/(k2(x3_k-x3_k1)))*(-k4*((x0[1][0]-x2cmd)**2)-k1*(x0[1][0]-x2cmd)*(A21*al-A22*(x0[1][0]**2)))))
    else: 
        Tcon1 = 1/B*(((A31-k4*A21)*al) + ((A32+k4*A22)*(x2**2))+d3)
        Tcon2 = 1/B*(-((k3*k4)*(x2-x2cmd))-((k3)*(x3-x3cmd)) )
        print( Tcon1, Tcon2)
        Tcon = Tcon1 + Tcon2
        #if ( i == 1 ):
        #    Tcon = 1.0/3*Tcon/B
        #Tcon = (1/B)*(((A31-k4*A21)*al) + ((A32+k4*A22)*(x2**2))+d3 -((k3*k4)*(x2-x2cmd))-((k3)*(x3-x3cmd)))#for Sliding Mode Control Torque
        
  
    K1 = f(x0)
    K2 = f(x0 + 0.5*ts*K1) #f(x0 + 2.0/3*ts*K1) #
    K3 = f(x0 + 0.5*ts*K2)
    K4 = f(x0+ ts*K3)

    #xK = x0 + np.dot(np.array(K1) + np.array(np.dot(K2,2)) + np.array(np.dot(K3,2)) + np.array(K4), (ts)*(1/6))
    xK = x0 + ts*K1 #ts/4 *(K1+3*K2) #ts*K1 # ts/6*(K1+2*K2+2*K3+K4) 
    x3_k1 = x0[2][0]
    x3_k=xK[2][0]
    x2_k1= x0[1][0]
   
    x0=xK 
    ## x1 = x0[0], x2 = x0[1] , x3 = x0[2]
    x1 = x0[0][0] 
    x2 = x0[1][0]
    x3 = x0[2][0]
    y1=3600/5280*x2
    y2=(60/(2*math.pi))*x3
    
    # save the old al value 
    alOLD = al

    x_values.append(x1)
    y_values.append(x2)
    z_values.append(x3)
    Tcon_values.append(Tcon)
    x2Error.append(x2-x2cmd)
    x3kError.append(x3_k-x3_k1)
    time.append(i*ts)
    x2MPH.append(y1)
    x3MPH.append(y2)
    print()





x_ValuesNum=np.array(x_values)
y_ValuesNum=np.array(y_values)
z_ValuesNum = np.array(z_values)
Tcon_ValuesNum=np.array(Tcon_values)
x2Error_ValuesNum = np.array(x2Error)
x3kError_ValuesNum = np.array(x3kError)
x2MPH_ValuesNum = np.array(x2MPH)
x3MPH_ValuesNum = np.array(x3MPH)

def plot_xy_values(x_values, y_values,title):
    # Create a scatter plot using Plotly
    fig = px.line(x=x_values, y=y_values, title = title)

    # Show the plot
    fig.show()

plot_xy_values(time, x_ValuesNum,'Position (ft)v Time Graph') #x1 v time
plot_xy_values(time, y_ValuesNum,'Velocity (ft/s) v Time Graph') #x2 v time
plot_xy_values(time, z_ValuesNum, 'Angular Velocity (rad/sec)v Time Graph') #x3 v time
plot_xy_values(time, Tcon_ValuesNum, 'Lyapunov Control Torque (ft-lbs)v Time Graph') #Torque control v time
plot_xy_values(time, x2Error_ValuesNum, 'Error in Velocity (ft/s) v Time Graph') #error in velocity v time
plot_xy_values(time, x3kError_ValuesNum, 'Angular Velocity Delta (rad/s)v Time Graph') #error in acceleration v time
plot_xy_values(time, x2MPH_ValuesNum, 'Velocity in MPH v Time Graph')
plot_xy_values(time, x3MPH_ValuesNum, 'Acceleration in RPM Graph')








