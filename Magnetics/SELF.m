function [Lp]=SELF(Rp_out, Rp_in, NumEL)
R = 1e-3; % assume wire radius equal to 1 mm;
mu0 = 4*pi*1.0e-7;
da = linspace(Rp_in,Rp_out,NumEL);
db = linspace(Rp_in,Rp_out,NumEL);
lp = 0;
    for aa = 1:length(da)
        for bb = 1:length(db)
            a = da(aa);
            b = db(bb);
            if a~=b %mutual inductance between two turns
                k = sqrt((4*a*b)/((a+b)^2));
                [EK EE] = ellipke(k^2);
                mut = mu0*sqrt(a*b)*(((2/k)-k)*EK-(2/k)*EE);
                lp = lp + mut;
            else %self-inductance of a single turn
                lp=lp+ mu0*a*(log((8*a)/R)-2);
            end
        end
    end
Lp = lp/(length(da)*length(db));
end