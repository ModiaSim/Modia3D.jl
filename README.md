# Modia3D

[![Stable](https://img.shields.io/badge/docs-stable-blue.svg)](https://ModiaSim.github.io/Modia3D.jl/stable)
[![The MIT License](https://img.shields.io/badge/license-MIT-brightgreen.svg?style=flat-square)](https://github.com/ModiaSim/Modia3D.jl/blob/master/LICENSE.md)

[Modia3D](https://github.com/ModiaSim/Modia3D.jl) is a Julia package that integrates a multibody program and 3D shapes for visualization and collision handling into the [Modia Modeling Language](https://github.com/ModiaSim/ModiaLang.jl). It is then, for example, possible to model the 3D mechanical part of a robot with Modia3D and the electrical motors and gearboxes that are driving the joints with the Modia language. Collision handling with elastic response calculation is performed for shapes that are defined with a contact material and have a convex geometry or are approximated by the convex hull of a concave geometry. For more details, see the [Modia3D Tutorial](@ref).

The multibody program supports currently tree-structured multibody systems, but does not (yet) support kinematic loops.

Example videos: xxx


## Installation

Modia3D is included in [Modia](https://github.com/ModiaSim/Modia.jl) and is available, when Modia was installed.

A standalone Modia3D version is installed with

```julia
julia> ]add ModiaLang, Modia3D
```

Modia3D animation can be exported in threejs-json-format and then imported in the open source web app [threejs.org](https://threejs.org/editor/) and use all the features of threejs, for example to export in the widely used glb format (= the binary version of the [glTF](https://www.khronos.org/gltf/) format) and use any glb-viewer (for example 3D-Viewer of Windows 10).

Additionally, the (free) community or the (commercial) professional version of the [DLR Visualization](http://www.systemcontrolinnovationlab.de/the-dlr-visualization-library/) library is supported that provides online animation (during simulation) and generation of mpg4-videos. To install the free version for *Windows* or for *Linux* perform the following steps:

1. Go to [https://visualization.ltx.de/](https://visualization.ltx.de/), provide your contact information and click on *Request download* for *Community Edition*. Afterwards, you get a link to download the library and you need to unzip the file.
2. In your Julia **startup file** (`HOME/.julia/config/startup.jl`) include the environment variable `ENV["DLR_VISUALIZATION"] = "<path-to-library>/Visualization/Extras/SimVis"`. Make sure that the SimVis executable under this directory has execution rights. For example in Linux with command:`chmod ug+x <path-to-library>/Visualization/Extras/SimVis/linux/SimVis`
3. Start Julia and run one of the tests, for example\
   `include("$(Modia3D.path)/test/Basic/PendulumWithBar1.jl")`


## Main Developers and License

[Andrea Neumayr](mailto:andrea.neumayr@dlr.de),
[Martin Otter](https://rmc.dlr.de/sr/de/staff/martin.otter/) and
[Gerhard Hippmann](mailto:gerhard.hippmann@dlr.de),\
[DLR - Institute of System Dynamics and Control](https://www.dlr.de/sr/en)

License: [MIT (expat)](LICENSE.md)
