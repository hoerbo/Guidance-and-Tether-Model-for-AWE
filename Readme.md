# Guidance and Tether Model for Airborne Wind Energy

This repository contains a MATLAB/Simulink Guidance Model for autonomous flight on a lemniscate.
In addition, a Tether Model based on Kelvin-Voigt material is provided.
The functionality is demonstrated in a basic kinematic framework.
Further testing in a SiL Simulator (results shown below, but code not provided here) proofs robustness of the approach.
Master Thesis in Aerospace Engineering. Written copy attached to this repository (in German). 

## Demos of Operation in Kinematic Framework
[![Demo Onboard-Generation System](https://s4.gifyu.com/images/OG_320b1d3aa0076cdb89a.gif)](https://youtu.be/XXs6cW-sE3Y)
[![Demo Ground-Generation System](https://s4.gifyu.com/images/GG_320.gif)](https://youtu.be/OWJ5FSvkmaQ)
* **Onboard-Generation System**: flight on constant orbit, tether force set point tracking (1 kN)
* **Ground-Generation System**: flight on increasing orbit (tether reeling out), tether force set point tracking (1 kN) 
* Tether States: epsilon=tether strain, F=tether force
* clickable external links (youtube)

## Demo of Operation in SiL-Simulator
[![Demo Ground-Generation System](https://s8.gifyu.com/images/Majaec2d2b3943dcd9cf.gif)](https://youtu.be/o2d76z69M18)
* **Untethered Flight on Constant Orbit**
* fully-fledged SiL-Simulator maintained by Institute of Flight Mechanics and Control, University of Stuttgart
* model: Bormatec Maja UAV Drone (wingspan: 2.2 m, mass ~4 kg)
* linear MIMO Low-Level Flight Control System
* video playback speed is doubled
* clickable external link (youtube)

## Features

#### Guidance
* general-purpose Guidance Model for flexible (kites) and rigid aircraft 
* suitable for onboard-generation and ground-generation systems
* input: lemniscate of bernoulli (target track)
* output: course angle/course rate and path angle/path rate in earth-fixed system

#### Tether Model
* formulated as system of implicit differential equations
* point masses connected with segments based on spring/damper models
* incorporates elasticity and drag effects

#### Kinematic Framework and Winch Model
* elementary simulation of aircraft movement
* allows basic demonstration of functionality of Guidance and Tether Model
* plain winch model for tether force tracking

## Usage
* main parameters are adjustable in settings.m
* run simulink_model
* current folder has to be the root directory (Simulink looks for extrinsic function tether_model.m)
* /moviemaker/create_movie.m allows generation of video files (as shown above) based on simulation output

## Credits and Scientific Background
* Jehle et al.: https://doi.org/10.2514/1.62380
* Rapp et al.: https://doi.org/10.2514/1.G004246
* Fechner et al.: https://doi.org/10.1016/j.renene.2015.04.028
* Rafael Wiedenroth: Guidance Approach and Tether Model for an Airborne Wind Energy System, Master Thesis, Institute of Flight Mechanics and Control (iFR), University of Stuttgart

## License

This Software uses the MIT license. See [LICENSE](https://github.com/hoerbo/Guidance-and-Tether-Model-for-AWE/blob/master/LICENSE) for more details.
