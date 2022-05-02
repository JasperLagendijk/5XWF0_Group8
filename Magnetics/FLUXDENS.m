%Calculate flux density
function [Bz,rs]= FLUXDENS(Rp,Rs,h,I,NumSt)
mu0 = 4*pi*1.0e-7;
rs = linspace(0,Rs,NumSt); % Accuracy can be improved by increasing the number of elements
    for nn=1:length(rs)
        alpha = rs(nn)/Rp;
        beta = h/Rp;
        Q = ((1+alpha)^2+beta^2);
        k = sqrt((4*alpha)/Q);
        [KK EK] = ellipke(k^2);
        B0 = (I*mu0)/(2*Rp);
        Bz(nn) = B0*(1/(pi*sqrt(Q)))*(EK*((1-alpha^2-beta^2)/(Q-4*alpha))+KK);
    end
end