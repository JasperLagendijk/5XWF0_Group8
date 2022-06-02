#include <stdio.h>
#include <math.h>
#define Kp 1
#define Ki 1

int main()
{
    
    double out = 0;
    double V_meas = ?; // where does the feedback come from?
    double error = 10;
    double preverror = 10;
    double integral = 0;
    
    double vref = 60;
    
    for(int x = 0; x<10; x++){
        error = vref - V_meas;
    
        integral = integral+((error + preverror)/2)*Ki;
    
        preverror = error;
    
        out = error*Kp + integral;
    
    }
}