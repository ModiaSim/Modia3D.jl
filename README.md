# Modia3D

[![Travis](https://travis-ci.org/ModiaSim/Modia3D.jl.svg?branch=master)](https://travis-ci.org/ModiaSim/Modia3D.jl)
[![AppVoyer](https://ci.appveyor.com/api/projects/status/github/ModiaSim/Modia3D.jl?svg=true)](https://ci.appveyor.com/project/MartinOtter/modia3d-jl)
[![codecov.io](https://codecov.io/github/ModiaSim/Modia3D.jl/coverage.svg?branch=master)](http://codecov.io/github/ModiaSim/Modia3D.jl?branch=master)
[![Stable](https://img.shields.io/badge/docs-stable-blue.svg)](https://ModiaSim.github.io/Modia3D.jl/stable)
[![Latest](https://img.shields.io/badge/docs-latest-blue.svg)](https://ModiaSim.github.io/Modia3D.jl/latest)
[![The MIT License](https://img.shields.io/badge/license-MIT-brightgreen.svg?style=flat-square)](https://github.com/ModiaSim/Modia3D.jl/blob/master/LICENSE.md)
[![DOI](https://img.shields.io/badge/DOI-10.5281%2Fzenodo.3978914-blue)](https://zenodo.org/record/3978914)

## Modia Platform

The *Modia platform* is a prototype system for the next modeling and simulation generation of physical systems described by differential and algebraic equations. It consists currently of the following Julia packages that are all under development (not all are yet publicly available):

| Package | Description |
|:----------------------------------------------------------|:------------------------------|
| *[Modia](https://github.com/ModiaSim/Modia.jl)*           | Equation based modeling       |
| *Modiator*                                                | 2D/3D web-app model editor    |
| *[Modia3D](https://github.com/ModiaSim/Modia3D.jl)*       | 3D geometry and 3D mechanics  |
| *[ModiaMedia](https://github.com/ModiaSim/ModiaMedia.jl)* | Thermodynamic property models |
| *Modelia* | [Modelica](https://www.modelica.org/modelicalanguage) model importer |

## Modia3D

Modia3D provides 3D geometry to physical systems so that geometrical objects can be
directly accessed and utilized in a model. Functions are provided, for example, to compute the volume,
mass, and inertia of a geometrical object or the distance between two objects.
Furthermore, Modia3D models 3D mechanical systems and are expanded into other domains.
It is possible, for example, to model the 3D mechanical part of a robot with Modia3D and the electrical motors and gearboxes that are driving the joints with Modia [6, 7]. Other examples are utilizing the 3D geometry to model heat flow in buildings or satellites.

Modia3D uses ideas from modern computer game engines to achieve a highly flexible setup of mechanical systems including collision handling. Other features are utilized from multi-body programs, such as support for closed kinematic loops, and elastic response calculation. The underlying mathematical formulation are hybrid Differential Algebraic Equations (DAEs) that are solved with the variable-step solver IDA via the [Sundials.jl](https://github.com/JuliaDiffEq/Sundials.jl) Julia package.

Collision handling with elastic response calculation is performed for geometrical objects that are defined with a contact material and have a convex geometry or are approximated by the convex hull of a concave geometry.
Penetration depths and Euclidean distances are computed with the improved Minkowski Portal Refinement (MPR) algorithm [1].
The details of the contact law are provided in [5] and several examples are discussed in [4a, 4b]. The user's view of Modia3D is introduced in [2] showing the very flexible definition of 3D systems. Some key algorithms are discussed in [3]. The combination of equation-based modeling with domain specific algorithms - in this case 3D dynamics is discussed in [6, 7].
A more detailed overview of the available features is also given in the [Modia3D documentation](https://ModiaSim.github.io/Modia3D.jl/stable).


## Status
The package has been tested with Julia on Windows 7, via the TravisCL on Linux (x86_64-pc-linux-gnu) and macOS (x86_64-apple-darwin14.5.0) and via the Appveyor CL on Windows.

Note, the collision handling is currently reasonably working for contacts between spheres as well as spheres and boxes. Still improvements are needed. Furthermore, kinematic loops are currently only supported for 2D loops and if they are driven kinematically. The technique to handle kinematic loops for dynamic simulations is demonstrated in the example `include("$(ModiaMath.path)/examples/withoutMacros_withoutVariables/Simulate_PendulumDAE.jl")` and is described in the paper *[Transformation of Differential Algebraic Array Equations to Index One Form](http://www.ep.liu.se/ecp/132/064/ecp17132565.pdf)*.

Up to now, Modia3D is implemented for functionality and not tuned for efficiency. Therefore, there are no benchmarks yet and in particular no comparison with Modelica models.

## Documentation
- [**STABLE**](https://ModiaSim.github.io/Modia3D.jl/stable) &mdash; *documentation of the last released version.*
- [**LATEST**](https://ModiaSim.github.io/Modia3D.jl/latest) &mdash; *in-development version of the documentation.*

### Publications

|    | Paper or Talk | Conference | DOI or YouTube |
|:---|:--------------|:-----------|:---------------|
| [1] |*[Collision Handling with Variable-Step Integrators](../resources/documentation/CollisionHandling_Neumayr_Otter_2017.pdf)*|[EOOLT 2017, December](http://www.eoolt.org/2017/)|[10.1145/3158191.3158193](https://doi.org/10.1145/3158191.3158193)|
| [2] |*[Component-Based 3D Modeling of Dynamic Systems](http://www.ep.liu.se/ecp/154/019/ecp18154019.pdf)*|[American Modelica Conference 2018, October](https://www.modelica.org/events/modelica2018Americas/index_html)|[10.3384/ECP18154175](https://doi.org/10.3384/ECP18154175)|
| [3] |*[Algorithms for Component-Based 3D Modeling](http://www.ep.liu.se/ecp/157/039/ecp19157039.pdf)*|[13th International Modelica Conference 2019, March](https://modelica.org/events/modelica2019)|[10.3384/ecp19157383](https://doi.org/10.3384/ecp19157383)|
| [4a] |*[Modia3D: Modeling and Simulation of 3D-Systems in Julia](https://proceedings.juliacon.org/papers/10.21105/jcon.00043)*|[JuliaCon 2019, July](https://juliacon.org/2019/)|[10.21105/jcon.00043](https://doi.org/10.21105/jcon.00043)|
| [4b] |*[Modia3D: Modeling and Simulation of 3D-Systems in Julia](https://proceedings.juliacon.org/papers/10.21105/jcon.00043)*|[JuliaCon 2019, July](https://juliacon.org/2019/)|[YouTube](https://www.youtube.com/watch?v=b3WfqXZRKpA)|
| [5] |*Collision Handling with Elastic Response Calculation and Zero-Crossing Functions*|[EOOLT 2019, November](http://www.eoolt.org/2019/)|[10.1145/3365984.3365986](https://doi.org/10.1145/3365984.3365986)|
| [6] |*Modia – Modeling Multidomain Engineering Systems with Julia*|[JuliaCon 2021, July](https://juliacon.org/2021/)|Video comming soon|
| [7] |*Modia – Equation Based Modeling and Domain Specific Algorithms*|[14th International Modelica Conference 2021, September](http://www.modelica.org/)|accepted for publication|


## Issues and Contributions
Contributions are welcome, as are feature requests and suggestions.
Please open an [issue](https://github.com/Modia/Modia3D.jl/issues) in this case and also if you encounter problems.


## Main Developers and License
[Andrea Neumayr](mailto:andrea.neumayr@dlr.de) and [Martin Otter](https://rmc.dlr.de/sr/de/staff/martin.otter/) and [Gerhard Hippmann](mailto:gerhard.hippmann@dlr.de)

[DLR - Institute of System Dynamics and Control](https://www.dlr.de/sr/en)

License: [MIT (expat)](LICENSE.md)
