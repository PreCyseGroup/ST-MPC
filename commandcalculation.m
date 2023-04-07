function controlcommand = commandcalculation(x_curr,T_prec,W,A,B,inputconstraint)
%control input computation according to ST-MPC. Optimization solved using
%Yalmip

%variable u denoting the command input we want to find
u=sdpvar(size(B,2),1);

set_to_reach=T_prec-W; % Using Minkowski difference to reduce T_prec by the amount of the noise
%the polyedral set to reach is "set_to_reach" represented as H_xx<=g_x
H_x=set_to_reach.A;
g_x=set_to_reach.b;

%the polyedral constraint set for the input constraint is "inputconstraint"
%represented as H_ux<=g_u
H_u=inputconstraint.A;
g_u=inputconstraint.b;

%Constraint modeling that the one-step evolution must be inside 'set_to_reach'
S=[H_x*(A*x_curr+B*u)<=g_x];
S=S+[H_u*u<=g_u];

%definition of the optimization problem
opt=sdpsettings('solver','sedumi','verbose',0);
diagnostics=optimize(S,norm(A*x_curr+B*u)^2,opt);
if diagnostics.problem >0
 disp('Solver thinks it is infeasible or numerical error')
 error('stop')
end
%optimal value (command)
controlcommand=value(u);


end

