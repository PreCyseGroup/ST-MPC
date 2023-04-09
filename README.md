# ST-MPC for polyedral sets

This code shows how to implementat, in a polyedral framework, the ST-MPC scheme developed in the paper entitled "An ellipsoidal off-line MPC scheme for uncertain polytopic discrete-time systems", co-authored by D. Angeli, A. Casavola, G. Franze', E. Mosca, and published in the Automatica journal in 2008.

Note that the mentioned paper developed the framework assuming an ellipoidal framework while this code assume a polyedral framework. However, the underlying set-theoretic MPC concepts and the used MPC philosophy are the same. 

This code has been developed in Matlab R2020b. It assumes that the MPT3 toolbox (including the Yalmip package) is installed in Matlab.

The example is developed for a simple mass-spring-damper model (2 states, 1 input) subject to state and constraints, as well as, bounded disturbances.
