# ST-MPC for polyedral sets

This code shows how to implement, in a polyedral framework, the ST-MPC scheme developed in [[Angeli et al., Automatica, 2008]](https://www.sciencedirect.com/science/article/abs/pii/S0005109808003014)

Note that the mentioned paper developed the framework assuming an ellipoidal framework while this code assume a polyedral framework. However, the underlying set-theoretic MPC concepts and the used MPC philosophy are the same.  The implemented algorithm is described in, e.g., [[Lucia et al., CONES, 2022], Section II.A](https://ieeexplore.ieee.org/document/9795085)

This code has been developed in Matlab R2020b. It assumes that the [MPT3 toolbox](https://www.mpt3.org/) (including the Yalmip package) is installed in Matlab.

The example is developed for a simple mass-spring-damper model (2 states, 1 input) subject to state and constraints, as well as, bounded disturbances.
