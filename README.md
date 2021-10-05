# Modia3D

[![Stable](https://img.shields.io/badge/docs-stable-blue.svg)](https://ModiaSim.github.io/Modia3D.jl/stable)
[![The MIT License](https://img.shields.io/badge/license-MIT-brightgreen.svg?style=flat-square)](https://github.com/ModiaSim/Modia3D.jl/blob/master/LICENSE.md)

[Modia3D](https://github.com/ModiaSim/Modia3D.jl) is a Julia package that integrates a multibody program and 3D shapes for visualization and collision handling into the [Modia Modeling Language](https://github.com/ModiaSim/ModiaLang.jl). It is then, for example, possible to model the 3D mechanical part of a robot with Modia3D and the electrical motors and gearboxes that are driving the joints with the Modia language. Collision handling with elastic response calculation is performed for shapes that are defined with a contact material and have a convex geometry or are approximated by the convex hull of a concave geometry. For more details, see the [Modia3D Tutorial](https://modiasim.github.io/Modia3D.jl/stable/tutorial/Tutorial.html).

Modia3D supports currently tree-structured multibody systems, but does not (yet) support kinematic loops.

Example videos:

- [YouBot robots gripping a workpiece](https://modiasim.github.io/Modia3D.jl/resources/videos/YouBotsGripping.mp4)
- [Billiard table with 16 balls](https://modiasim.github.io/Modia3D.jl/resources/videos/Billard16Balls.mp4)
- [Mobile with 8 levels](https://modiasim.github.io/Modia3D.jl/resources/videos/Mobile8.mp4)


## Installation

Modia3D is included in [Modia](https://github.com/ModiaSim/Modia.jl) and is available, when Modia was installed. Please, follow the [extensive installation guide](https://github.com/ModiaSim/Modia3D.jl/wiki/Full-Installation-Guide-for-Julia,-Modia3D,-Modia) or the [lightweight installation guide](https://github.com/ModiaSim/Modia3D.jl/wiki/Lightweight-Installation-Guide:-Julia-Modia3D) for Modia3D.

A standalone Modia3D version is installed with

```julia
julia> ]add ModiaLang, Modia3D
```

Modia3D animation can be exported in [three.js JSON Object Scene format](https://github.com/mrdoob/three.js/wiki/JSON-Object-Scene-format-4) and then imported in the open source web app [three.js editor](https://threejs.org/editor/) and use all the features of three.js, for example to export in the widely used glb format (the binary version of the [glTF](https://www.khronos.org/gltf/) format) and use any glb viewer (for example 3D-Viewer of Windows 10).

Additionally, Modia3D uses the (free) community or the (commercial) professional version of the [DLR Visualization Library](https://visualization.ltx.de/) for 3D simulations. It provides online animation (during simulation) and the generation of mpg4-videos. If you don't use the DLR Visualization Library result animation is switched of.
Download and install the free DLR SimVis Community Edition, e.g. with [https://visualization.ltx.de/](https://visualization.ltx.de/) .

* Set an environment variable or add it to the [startup.jl file](https://github.com/ModiaSim/Modia3D.jl/wiki/Template-for-startup.jl). Please, read also the [installation guide](https://github.com/ModiaSim/Modia3D.jl/wiki/Full-Installation-Guide-for-Julia,-Modia3D,-Modia).
   * `ENV["DLR_VISUALIZATION"] = "<path-to-library>/Visualization/Extras/SimVis"`
   * Make sure that the SimVis executable under this directory has execution rights. For example in Linux with command: `chmod ug+x <path-to-library>/Visualization/Extras/SimVis/linux/SimVis`


## Main Developers and License

[Andrea Neumayr](mailto:andrea.neumayr@dlr.de),
[Martin Otter](https://rmc.dlr.de/sr/de/staff/martin.otter/) and
[Gerhard Hippmann](mailto:gerhard.hippmann@dlr.de),\
[DLR - Institute of System Dynamics and Control](https://www.dlr.de/sr/en)

License: [MIT (expat)](LICENSE.md)
