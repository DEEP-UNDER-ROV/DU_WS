# rov_supri

Packages to simulate and interface hardware--software to SUPRI ROV. This is an interface layer packages that connect the software system to hardware system. This layer can be customized based on the hardware, while the upper layer like perception, planning or control can act like generic package for arbitrary hardware. 



## rov_mavros

package to connect upper layer with mavros, for example in the current development there are bridge from slam to mavros and launch file to launch mavros easily. This package can be used for both in sim and real deployment.

## rov_sim

This package contain everything to run rov simulation, like models, params, and worlds. This package need ardupilot_gazebo and Ardusupri to run.
