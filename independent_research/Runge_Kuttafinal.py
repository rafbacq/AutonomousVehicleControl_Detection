import numpy as np
import scipy.signal
import sys
import io
import plotly.express as px
import math
import matplotlib.pyplot as plt
import scipy
from scipy import signal
from scipy.signal import cheby2, filtfilt


fin = open('Independent_File.in.txt', mode='r', encoding='utf-8-sig')

# read all the inputs into variables
k1 = float(fin.readline())  # gain #1 =0
k2 = float(fin.readline())  # gain #2 =0
k3 = float(fin.readline())  # gain #3 =1.05
k4 = float(fin.readline())  # gain #4 =0.6
x1 = float(fin.readline())  # initial vehicle position = 0
x2 = float(fin.readline())  # initial vehicle velocity=0
x3 = float(fin.readline())  # initial wheel angular velocity=0
x2cmd = float(fin.readline())  # command controller of vehicle velocity=80
tn = int(fin.readline())  # number of time steps=100
ts = float(fin.readline())  # time steps duration=0.2
controlTorque = str(fin.readline()) #.strip() # names what control torque is currently being used= Lyapunov Control Torque
x2_2 = float(fin.readline()) #initial second vehicle velocity=70
dd_2 = float(fin.readline()) # delta of distance between two cars when autonomous vehicle needs to maintain=10

print(controlTorque)
print(type(controlTorque))
# define all the variables
A21 = 29.6
A22 = 0.0000559
A31 = 1459.327
A32 = 0.0189
d3 = 14.3201
B = 0.0225
rw = 15.37 / 12.0
lpk = 0.17

xK = 0
x3_k = 0
x3_k1 = 0
alOLD = 0
x2_k1 = 0

xKNoise = 0
x3_kNoise = 0
x3_k1Noise = 0
alOLDNoise = 0
x2_k1Noise = 0

cutoff_frequency = 1.0  # Adjust the cutoff frequency as needed
sampling_rate = 1 / ts  # Assuming uniform sampling

x2 = x2 * 5280 / 3600
x3 = x2 / rw  # (rw*(1-lpk))

x2cmd = (x2cmd * 5280) / 3600
if controlTorque == "Lyapunov Control Torque":
    x3cmd = x2cmd / rw
else:
    x3cmd = x2cmd / rw


def calc_al(l_val):
    al_local = 2 * (l_val / lpk) / (1 + (l_val / lpk) ** 2)
    # -1<= a_local <=1
    al_local = max(-1, al_local)
    al_local = min(1, al_local)

    return al_local


def calc_lamda(v, w):
    l_local = 0

    if v * w < 0:
        print(v, w)
    if v < 0:
        print(v)

    if v <= rw * w:
        if w != 0:
            l_local = 1 - v / (rw * w)
    else:
        if v != 0:
            l_local = (rw * w) / v - 1
    # -lpk<=l <=lpk
    l_local = min(l_local, lpk)
    l_local = max(l_local, -lpk)

    return l_local


def f(x,TconCurrent):
    l_x = calc_lamda(x[1][0], x[2][0])
    al_x = calc_al(l_x)

    x1dot = x[1][0]
    x2dot = A21 * al_x - A22 * (x[1][0] ** 2)
    x3dot = (-A31 * al_x) - A32 * (x[1][0] ** 2) - d3 + B * TconCurrent

    return np.array([[x1dot], [x2dot], [x3dot]])


def add_gaussian_noise(signal, mean=0, std=10):
    noise = np.random.normal(mean, std, signal.shape)
    noisy_signal = signal + noise
    return noisy_signal


# Function to apply a higher-order low-pass filter

def high_order_low_pass_filter(signal, cutoff_frequency, sampling_rate, order, ripple=1, stop_atten=30):
    nyquist = 0.5 * sampling_rate
    normal_cutoff = cutoff_frequency / nyquist

    # Design the filter using scipy
    b, a = cheby2(order, stop_atten, cutoff_frequency, 'low', fs=sampling_rate)

    # Apply the filter
    filtered_signal = filtfilt(b, a, signal)

    return filtered_signal


x3Old = x3  # x3 one time step before

Tcon = 0.0
TconNoise = 0.0  # temporary control torque value
TconFiltered=0.0

# defining the gaussian noise arrays that will affect the data
gaussianNoiseVelocity = np.random.normal(0, 5, tn)
gaussianNoiseAngularVelocity = np.random.normal(0, 5, tn)

x_values = []
y_values = []
z_values = []
Tcon_values = []
TconNoise_values = []
TconFiltered_values=[]
x0 = [[x1],[x2],[x3]]
x0Noise = [[x1],[x2],[x3]]
x0Filtered =[[x1],[x2],[x3]]
time = []
x2Error = []
x3kError = []
x2MPH = []
x3MPH = []

constantCar_NoisyPosValues = []
carDistances_NoisyArr= []
car_vsNoisy_pos = 50 #going to start ahead of the car by this much
carDistance_Noisy=0 # will calculate how much each time they are apart
x2cmd_noisy=x2cmd

for i in range(1, (tn + 1)):
    #Calculating Data with Noise (first time through will calculate the noisy data graph)
    car_vsNoisy_pos +=(x2_2*ts)
    constantCar_NoisyPosValues.append(car_vsNoisy_pos)

    x2Noise = x0Noise[1][0]+ gaussianNoiseVelocity[i-1]
    x3Noise = x0Noise[2][0] + gaussianNoiseAngularVelocity[i-1]
    x0Noise = [[x1],[x2Noise],[x3Noise]]
    lNoise = calc_lamda(x2Noise,x3Noise)
    alNoise = calc_al(lNoise)

    if controlTorque == "Lyapunov Control Torque":
        TconNoise = TconNoise + (1 / B) * (
                (A31 * (alNoise - alOLDNoise) + A32 * ((x2Noise ** 2) - (x2_k1Noise ** 2)) - k3 * (x3_kNoise - x3_k1Noise) + (
                1 / (k2 * (x3_kNoise - x3_k1Noise))) * (
                        -k4 * ((x2Noise - x2cmd_noisy) ** 2) - k1 * (x2Noise - x2cmd_noisy) * (
                        A21 * alNoise - A22 * (x2Noise ** 2)))))
    else:
        Tcon1 = 1 / B * ((A31 - k4 * A21) * alNoise + (A32 + k4 * A22) * (x2Noise ** 2) + d3)
        Tcon2 = 1 / B * (-(k3 * k4) * (x2Noise - x2cmd_noisy) - k3 * (x3Noise - x3cmd))
        TconNoise = Tcon1 + Tcon2

    

    K1Noise = f(x0Noise,TconNoise)
    K2Noise = f(x0Noise + 0.5 * ts * K1Noise,TconNoise)
    K3Noise = f(x0Noise + 0.5 * ts * K2Noise,TconNoise)
    K4Noise = f(x0Noise + ts * K3Noise,TconNoise)

    xKNoise = x0Noise + ts * K1Noise
    x3_k1Noise = x0Noise[2][0]
    x3_kNoise = xKNoise[2][0]
    x2_k1Noise = x0Noise[1][0]

    

    x0Noise = xKNoise
    y1Noise = 3600 / 5280 * x2
    y2Noise = (60 / (2 * math.pi)) * x3

    alOLDNoise = alNoise

    

    #Calculating Data without Noise





    l = calc_lamda(x2, x3)
    al = calc_al(l)  # attenuation from wheel slip
    if controlTorque == "Lyapunov Control Torque":
        Tcon = Tcon + (1 / B) * (
                (A31 * (al - alOLD) + A32 * ((x0[1][0] ** 2) - (x2_k1 ** 2)) - k3 * (x3_k - x3_k1) + (
                1 / (k2 * (x3_k - x3_k1))) * (
                        -k4 * ((x0[1][0] - x2cmd_noisy) ** 2) - k1 * (x0[1][0] - x2cmd_noisy) * (
                        A21 * al - A22 * (x0[1][0] ** 2)))))
    else:
        Tcon1 = 1 / B * ((A31 - k4 * A21) * al + (A32 + k4 * A22) * (x2 ** 2) + d3)
        Tcon2 = 1 / B * (-(k3 * k4) * (x2 - x2cmd_noisy) - k3 * (x3 - x3cmd))
        Tcon = Tcon1 + Tcon2

    K1 = f(x0,Tcon)
    K2 = f(x0 + 0.5 * ts * K1,Tcon)
    K3 = f(x0 + 0.5 * ts * K2,Tcon)
    K4 = f(x0 + ts * K3,Tcon)

    xK = x0 + ts * K1
    x3_k1 = x0[2][0]
    x3_k = xK[2][0]
    x2_k1 = x0[1][0]

    x0 = xK
    x1 = x0[0][0]
    x2 = x0[1][0]
    x3 = x0[2][0] 
    y1 = 3600 / 5280 * x2
    y2 = (60 / (2 * math.pi)) * x3
    alOLD = al

    carDistance_Noisy = car_vsNoisy_pos-x1
    if(carDistance_Noisy==dd_2):
        x2cmd_noisy = x2_2
    carDistances_NoisyArr.append(carDistance_Noisy)
    

    #Add values to arrays in order to graph
    x_values.append(x1)
    y_values.append(x2)
    z_values.append(x3)
    Tcon_values.append(Tcon)
    TconNoise_values.append(TconNoise)
    x2Error.append(x2 - x2cmd)
    x3kError.append(x3_k - x3_k1)
    time.append(i * ts)
    x2MPH.append(y1)
    x3MPH.append(y2)

# Generate more noisy signals
noisy_x2 = y_values + gaussianNoiseVelocity
noisy_x3 = z_values + gaussianNoiseAngularVelocity

# Filter the more noisy signals with a higher-order filter
filtered_x2 = high_order_low_pass_filter(noisy_x2, cutoff_frequency, sampling_rate, order=15)
filtered_x3 = high_order_low_pass_filter(noisy_x3, cutoff_frequency, sampling_rate, order=15)

#Run loop again this time with filtered noise
xK = 0
x3_k = 0
x3_k1 = 0
alOLD = 0
x2_k1 = 0

constantCar_FilteredPosValues = []
carDistances_FilteredArr= []


car_vsFiltered_pos = 50 #going to start ahead of the car by this much
carDistance_Filtered=0 # will calculate how much each time they are apart
x2cmd_Filtered=x2cmd


for i in range(1,(tn+1)):
    #Calculating Data with Noise (first time through will calculate the noisy data graph)
    car_vsFiltered_pos +=(x2_2*ts)
    constantCar_FilteredPosValues.append(car_vsFiltered_pos)


    #calculating the fixed data(will display the fixed data after getting filtered)
    l = calc_lamda(filtered_x2[i-1], filtered_x3[i-1])
    al = calc_al(l)#attenuation from wheel slip

    xK = x0Filtered + ts*K1 #ts/4 *(K1+3*K2) #ts*K1 # ts/6*(K1+2*K2+2*K3+K4) 
    x3_k1 = x0Filtered[2][0]
    x3_k=xK[2][0]
    x2_k1= x0Filtered[1][0]
   
    x0Filtered=xK 
    if(controlTorque == "Lyapunov Control Torque"):
        TconFiltered = TconFiltered + (1/B)((A31*(al-alOLD) + A32 * ((filtered_x2[i-1]**2)-(x2_k1**2)) - 
        k3*(x3_k-x3_k1) + (1/(k2(x3_k-x3_k1)))*(-k4*((filtered_x2[i-1]-x2cmd_Filtered)**2)-k1*(filtered_x2[i-1]-x2cmd_Filtered)*
        (A21*al-A22*(filtered_x2[i-1]**2)))))
    else: 
        Tcon1 = 1/B*(((A31-k4*A21)*al) + ((A32+k4*A22)*(filtered_x2[i-1]**2))+d3)
        Tcon2 = 1/B*(-((k3*k4)*(filtered_x2[i-1]-x2cmd_Filtered))-((k3)*(filtered_x3[i-1]-x3cmd)) )
        TconFiltered = Tcon1 + Tcon2
        #if ( i == 1 ):
        #    Tcon = 1.0/3*Tcon/B
        #Tcon = (1/B)*(((A31-k4*A21)*al) + ((A32+k4*A22)*(x2**2))+d3 -((k3*k4)*(x2-x2cmd))-((k3)*(x3-x3cmd)))#for Sliding Mode Control Torque
        
  
    K1 = f(x0Filtered,TconFiltered)
    K2 = f(x0Filtered + 0.5*ts*K1,TconFiltered) #f(x0 + 2.0/3*ts*K1) #
    K3 = f(x0Filtered + 0.5*ts*K2,TconFiltered)
    K4 = f(x0Filtered+ ts*K3,TconFiltered)

    #xK = x0 + np.dot(np.array(K1) + np.array(np.dot(K2,2)) + np.array(np.dot(K3,2)) + np.array(K4), (ts)*(1/6))
    
    ## x1 = x0[0], x2 = x0[1] , x3 = x0[2]
    x1 = x0Filtered[0][0] 
    x2 = x0Filtered[1][0]
    x3 = x0Filtered[2][0]
    y1Filtered=3600/5280*x2
    y2Filtered=(60/(2*math.pi))*x3
    
    # save the old al value 
    alOLD = al

    carDistance_Filtered = car_vsFiltered_pos-x1
    if(carDistance_Filtered==dd_2):
        x2cmd_Filtered = x2_2
    carDistances_FilteredArr.append(carDistance_Filtered)

    TconFiltered_values.append(TconFiltered)

# Plot the original, more noisy, and filtered signals
plt.figure(figsize=(12, 16))

plt.subplot(7, 1, 1)
plt.plot(time, y_values, label='Original x2')
plt.plot(time, noisy_x2, label='Noisy x2')
plt.plot(time, filtered_x2, label='Filtered x2')
plt.title('Velocity (ft/s) v Time Graph')
plt.legend(loc='upper right')

plt.subplot(7, 1, 2)
plt.plot(time, z_values, label='Original x3')
plt.plot(time, noisy_x3, label='Noisy x3')
plt.plot(time, filtered_x3, label='Filtered x3')
plt.title('Angular Velocity (rad/sec) v Time Graph')
plt.legend(loc = 'upper right')

plt.subplot(7, 1, 3)
plt.plot(time, Tcon_values, label='Original Torque Control')
plt.plot(time, TconNoise_values, label='Torque Control affected by Noise')
plt.plot(time, TconFiltered_values, label = 'Torque Control Filtered after Noise')
plt.title('Torque Control (ft-lbs) v Time Graph')
plt.legend(loc='upper right')


#constantCar_NoisyPosValues = []
#carDistances_NoisyArr= []
plt.subplot(7, 1, 4)
plt.plot(time, constantCar_NoisyPosValues, label = 'Constant Velocity Car Position (Noisy)')
plt.legend(loc= 'upper right')

plt.subplot(7, 1, 5)
plt.plot(time, carDistances_NoisyArr, label = 'Distance between the two cars (Noisy)')
plt.legend(loc= 'upper right')

plt.subplot(7, 1, 6)
plt.plot(time, constantCar_FilteredPosValues, label = 'Constant Velocity Car Position (Filtered)')
plt.legend(loc= 'upper right')

plt.subplot(7, 1, 7)
plt.plot(time, carDistances_FilteredArr, label = 'Distance between the two cars (Filtered)')
plt.legend(loc= 'upper right')

plt.show()
