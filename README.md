# Dual-model Set-Theoretic MPC (ST-MPC) in a polyedral framework.

This code shows how to implement, in a polyedral framework, the  dual-mode receding horizon controller developed in [[Angeli et al., Automatica, 2008]](https://www.sciencedirect.com/science/article/abs/pii/S0005109808003014). Note that the mentioned article developed the control framework assuming an ellipoidal modeling for disturbances and constraints. On the other hand, the proposed implementation is for a polyedral framework (i.e., constraints and disturbance sets are polyedral sets). Nevertheless, the underlying set-theoretic MPC concepts and the used MPC philosophy are the same.  The implemented algorithm is described in, e.g., [[Lucia et al., CONES, 2022], Section II.A.](https://ieeexplore.ieee.org/document/9795085)

# Considered plant model and polyedral framework 
It is assumed that the plant is described by a LTI model subject to state and input constraints, as well as, bounded state disturbances. All the constraints and disturbances sets are modeled by means of convex polyedral sets.

# Matlab implementation and required packages
This code has been developed and tested for Matlab R2020b. It assumes that the [MPT3 toolbox](https://www.mpt3.org/) (including the Yalmip package) is correctly installed in Matlab.

# Example
The example is developed assiming a toy mass-spring-damper model (2 states, 1 input) subjects to state and input constraints, as well as, bounded disturbances. 

# Run the simulation
Just run the file "main.m"
