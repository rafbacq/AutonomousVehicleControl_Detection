import numpy as np
import sys
import io
import pandas 
import plotly.express as px


test_array = 2*np.array([[1],[2],[3]])

print(test_array[0][0])
print(test_array[1][0])

x0= [[1],[2],[3]]
x00= [1,2,3]
x1 = x0+np.dot(x0,(1)*(0.5))
x11 = x00+np.dot(x00,(1)*(0.5))

fin = open('Independent_File.in.txt', mode='r', encoding='utf-8-sig')

#read all the inputs in variables
k3 = float(fin.readline()) #gain #1
k4 = float(fin.readline()) #gain #2 (Applicable for Sliding Mode Control Torque)
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
A32 = 0.00157
d3 = -14.3201
B = 0.0225
rw = 15.37/12.0
lpk = 0.17

x3cmd = 0 #coming in the future :)
x3Old =x3 #x3 one time step before

l=0 #temporary lambda value

if(x2==0 and x3==0):
    l=0
elif (x2<= rw*x3):
    l = 1-(x2/(rw*x3))
elif(x2>rw*x3):
    l = ((rw*x3)/x2)-1

al = (2*(l/lpk))/(1+((l/lpk)**2)) #attenuation from wheel slip
Tcon =0 #temporary control torque value
if controlTorque=="Sliding Mode Control Torque":
    Tcon = (1/B)*(((A31-k4*A21)*al) + ((A32+k4*A22)*(x2**2)) +d3-((k3*k4)*(x2-x2cmd))-((k3)*(x3-x3cmd)))#for Sliding Mode Control Torque
else:
    Tcon = (1/B)*(((A31-A21)*al) + ((A32+A22)*(x2**2)) +d3-((k3)*(x2-x2cmd)) + ((1/(tn*ts))*(x3-x3Old)))#for Lyapunov Control Torque

#Performing the Runge-Kutta Method on Autonomous Vehicle Equations XD

def f(x):
    if(x[1][0]==0 and x[2][0]==0):
        l=0
    elif (x[1][0]<= rw*x[2][0]):
        l = 1-(x[1][0]/(rw*x[2][0]))
    elif(x[1][0]>rw*x[2][0]):
        l = ((rw*x[2][0])/x[1][0])-1
    al = (2*(l/lpk))/(1+((l/lpk)**2))
    
    x1dot = x[1][0]
    x2dot = A21*al -(A22)*(x[1][0]**2)
    x3dot = (-A31*al)-(A32*(x[1][0]**2))+d3+B*Tcon
    
    return np.array([x1dot],[x2dot],[x3dot])

x0= [[1],[2],[3]]
x00= [1,2,3]
x1 = x0+np.dot(x0,(1)*(0.5))
x11 = x00+np.dot(x00,(1)*(0.5))
####Shirley debug 

####Shirley done debugging
K1 = f(x0)