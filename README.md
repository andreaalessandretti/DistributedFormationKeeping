# Distributed formation-keeping of underactuated vehicles  

This git illustrate the use of a distributed formation-keeping control algorithm for vehicles described by a unicycle model. The controller is published in 

>Roth Y., Alessandretti A., Aguiar A. P., Jones C.	*A virtual vehicle approach to distributed control for formation keeping of underactuated vehicles.* Proc. of the 1th Indian Control Conf. (ICC 2015), Chennai, India 2015.

The control architecture presents a two layers structure: 

- A lower layer performs formation keeping (using consensus theory) among a set of virtual single integrators.
- An upper layer uses a trajectory-tracking controller to track the virtual vehicle.

For more details we refer to the publication above.

## Files
The simulations are made using [VirtualArena v2.1.0](https://github.com/andreaalessandretti/VirtualArena/releases/tag/v2.1.0).

- The file `runmeSingleIntegrators.m` simulates formation-keeping among a set of single integrators. The controller in this simulation is taken from:
>Wei Ren and RW Beard. *Distributed consensus in multi-vehicle Cooperative control.* Springer-VerlagLondon

- The file `runmeUnicycle.m` simulates formation-keeping among a set of unicycle models using the proposed virtual-vehicle approach.