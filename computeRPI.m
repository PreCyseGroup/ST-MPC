function T0 = computeRPI(Acl,alpha,W)
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%%%%%   Building an RPI set for the terminal region in presence of bounded disturbace %%%%%%%
%Algorithm taken from :Invariant Approximations of the Minimal RobustPositively Invariant Set%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

Wa = alpha*W ;

%calculate the RPI region for polytopes, by fixing alpha, we find the 's'
%that satisfies the equation (4) in the paper.
for s=1:100
    Ws =(Acl^s)*W ;
    tf = Wa.contains(Ws);
    if tf==1
        break
    end
end

Fs=W;
for i=1:s-1
    Fi = (Acl^i)*W;
    Fs = plus(Fs,Fi);
end

%Refering to equation (5) of the paper, the RPI terminal region is the following:
al= 1/(1-alpha); 
T0 = al *Fs;

end

