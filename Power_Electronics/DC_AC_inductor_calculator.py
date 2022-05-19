
fs =  1000
PI = 3.14159269
I = 5

delta_L = 0.2 *I
V_in = 60

L = V_in/(4*delta_L*fs)


print(L*1000, "mH")

C = (pow(10/(2*PI*fs), 2))/L

print(C*1000000, "uF")

