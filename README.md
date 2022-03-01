# Modia3D.jl

[![Stable](https://img.shields.io/badge/docs-stable-blue.svg)](https://ModiaSim.github.io/Modia3D.jl/stable)
[![The MIT License](https://img.shields.io/badge/license-MIT-brightgreen.svg?style=flat-square)](https://github.com/ModiaSim/Modia3D.jl/blob/master/LICENSE.md)

[Modia3D](https://github.com/ModiaSim/Modia3D.jl) is a Julia package that adds a multibody program and 3D shapes for visualization and collision handling to the [Modia Modeling Language](https://github.com/ModiaSim/Modia.jl). It is then, for example, possible to model the 3D mechanical part of a robot with Modia3D and the electrical motors and gearboxes that are driving the joints with the Modia language. Collision handling with elastic response calculation is performed for shapes that are defined with a contact material and have a convex geometry or are approximated by the convex hull of a concave geometry. For more details, see the [Modia3D Tutorial](https://modiasim.github.io/Modia3D.jl/stable/tutorial/Tutorial.html).

Modia3D supports currently tree-structured multibody systems, but does not (yet) support kinematic loops.

Example videos:

- [YouBot robots gripping a workpiece](https://modiasim.github.io/Modia3D.jl/resources/videos/YouBotsGripping.mp4)
- [Billiard table with 16 balls](https://modiasim.github.io/Modia3D.jl/resources/videos/Billard16Balls.mp4)
- [Mobile with 8 levels](https://modiasim.github.io/Modia3D.jl/resources/videos/Mobile8.mp4)


## Installation

Modia3D requires Julia 1.7 or later and is installed with

```julia
julia> ]add Modia3D
```

It is advised to also install Modia and at least one Modia plot package (for details see [Installation of Modia](https://modiasim.github.io/Modia.jl/stable/#Installation)).

Note, Modia3D reexports the following definitions 

- `using Modia`
- `using Unitful`
- `using DifferentialEquations`
- and exports functions `CVODE_BDF` and `IDA` of [Sundials.jl](https://github.com/SciML/Sundials.jl).

As a result, it is usually sufficient to have `using Modia3D` in a model to utilize the relevant 
functionalities of these packages.

Modia3D has various *3D animation features*:

- With `world = Object3D(feature=Scene(animationFile="filename.json"))` the animation produced during a simulation run
  is exported in [three.js JSON Object Scene format](https://github.com/mrdoob/three.js/wiki/JSON-Object-Scene-format-4).
  The generated file can be imported into the open source web app [three.js editor](https://threejs.org/editor/) and 
  use all the features of three.js, for example to export in the widely used glb format (the binary version of the [glTF](https://www.khronos.org/gltf/) format) 
  and use any glb viewer (for example 3D-Viewer of Windows).

- With the default option `world = Object3D(feature=Scene(enableVisualization=true))` the 
  [DLR Visualization Library](https://www.systemcontrolinnovationlab.de/the-dlr-visualization-library/)
  (see the many examples from various research and industrial projects) is used, if installed, for 
  online animation (during simulation), replays (after simulation), and the generation of mpg4-videos.
  This library is available as *(free) Community Edition* and as *(commercial) Professional Edition*
  ([Web page to request download of DLR Visualization Library](https://visualization.ltx.de/)).
  After download, make the library available in the following way:
  - Set Julia environment variable `ENV["DLR_VISUALIZATION"] = "<path-to-library>/Visualization/Extras/SimVis"` 
    or add it to the [Julia startup.jl file](https://github.com/ModiaSim/Modia3D.jl/wiki/Template-for-startup.jl). 
    *Make sure that the SimVis executable under this directory has execution rights.*
    For example in Linux with command: `chmod ug+x <path-to-library>/Visualization/Extras/SimVis/linux/SimVis`


## Main Developers and License

[Andrea Neumayr](mailto:andrea.neumayr@dlr.de),
[Martin Otter](https://rmc.dlr.de/sr/de/staff/martin.otter/) and
[Gerhard Hippmann](mailto:gerhard.hippmann@dlr.de),\
[DLR - Institute of System Dynamics and Control](https://www.dlr.de/sr/en)

License: [MIT (expat)](LICENSE.md)
