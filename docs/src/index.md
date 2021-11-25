# Modia3D Documentation

[Modia3D](https://github.com/ModiaSim/Modia3D.jl) is a Julia package that integrates a multibody program and 3D shapes for visualization and collision handling into the [Modia Modeling Language](https://github.com/ModiaSim/ModiaLang.jl). It is then, for example, possible to model the 3D mechanical part of a robot with Modia3D and the electrical motors and gearboxes that are driving the joints with the Modia language. Collision handling with elastic response calculation is performed for shapes that are defined with a contact material and have a convex geometry or are approximated by the convex hull of a concave geometry. For more details, see the [Modia3D Tutorial](@ref).

The multibody program supports currently tree-structured multibody systems, but does not (yet) support kinematic loops.

Example videos:

- [YouBot robots with gripping](https://modiasim.github.io/Modia3D.jl/resources/videos/YouBotsGripping.mp4)
- [Billiard table with 16 balls](https://modiasim.github.io/Modia3D.jl/resources/videos/Billard16Balls.mp4)
- [Mobile with depthmax=8](https://modiasim.github.io/Modia3D.jl/resources/videos/Mobile8.mp4)


## Installation

Modia3D is included in [Modia](https://github.com/ModiaSim/Modia.jl) and is available, when Modia was installed. Please, follow the [extensive installation guide](https://github.com/ModiaSim/Modia3D.jl/wiki/Full-Installation-Guide-for-Julia,-Modia3D,-Modia) or the [lightweight installation guide](https://github.com/ModiaSim/Modia3D.jl/wiki/Lightweight-Installation-Guide:-Julia-Modia3D) for Modia3D.

A standalone Modia3D version is installed with

```julia
julia> ]add ModiaLang, Modia3D
```

Modia3D animation can be exported in threejs-json-format and then imported in the open source web app [threejs.org](https://threejs.org/editor/) and use all the features of threejs, for example to export in the widely used glb format (= the binary version of the [glTF](https://www.khronos.org/gltf/) format) and use any glb-viewer (for example 3D-Viewer of Windows 10).

Additionally, Modia3D uses the (free) community or the (commercial) professional version of the [DLR Visualization Library](https://visualization.ltx.de/) for 3D simulations. It provides online animation (during simulation) and the generation of mpg4-videos. If you don't use the DLR Visualization Library result animation is switched of.
Download and install the free DLR SimVis Community Edition, e.g. with [https://visualization.ltx.de/](https://visualization.ltx.de/) .

* Set an environment variable or add it to the [startup.jl file](https://github.com/ModiaSim/Modia3D.jl/wiki/Template-for-startup.jl). Please, read also the [installation guide](https://github.com/ModiaSim/Modia3D.jl/wiki/Full-Installation-Guide-for-Julia,-Modia3D,-Modia).
   * `ENV["DLR_VISUALIZATION"] = "<path-to-library>/Visualization/Extras/SimVis"`
   * Make sure that the SimVis executable under this directory has execution rights. For example in Linux with command: `chmod ug+x <path-to-library>/Visualization/Extras/SimVis/linux/SimVis`


## Release Notes

### Version 0.5.1

- Collision handling - MPR algorithm improved to enhance speed and robustness
    - collision pairing material
        - at initialization: a warning is given if a collision pairing material is not defined
    - improve support points computation of
        - FileMesh: use `SVectors` and better computation of support points
        - Capsule: a Capsule is already smooth therefore collisionSmoothingRadius is removed
    - MPR algorithm
        - `Double64` from DoubleFloats.jl package is used for mpr algorithm to increase accuracy
        - amount of iteration steps is increased if more are needed for phase 2 and phase 3
        - if it's not possible to quit with the predefined mprTolerance the iteration quits with the best possible tolerance instead
        - collisions between ellipsoids and other shapes are treated like collisions between spheres and other shapes
    - store information like centroid in Object3D
    - remove some type instabilities
- FreeMotion joint
    - enable adaptive rotation sequence
    - add test model
- Enable ForceElements
    - add infrastructure
    - add Bushing and SpringDamperPtP
    - add test models and documentation
- Animation export (three.js JSON object scene)
    - fix initial orientation
    - enable visualization of bounding boxes (AABB)
    - enable Beam support
    - enable Capsule support
    - enable inner cylinder radius support (pipe)
    - enable CoordinateSystem support
    - enable Grid support
- Visualization with DLR Visualization Library
    - add visual shape kind ModelicaShape
    - remove some type instabilities
- improve testing
    - enable short and complete test runs
    - add planar motion test
- update documentation and installation guide
- clean up: remove unused code snippets


### Version 0.5.0

- Largely redesigned.
- Integrated into Modia.
- Not backwards compatible to previous versions.


## Publications

|  | Paper or Talk | Conference | DOI or YouTube |
|:---|:--------------|:-----------|:---------------|
| [1] |*[Collision Handling with Variable-Step Integrators](../resources/documentation/CollisionHandling_Neumayr_Otter_2017.pdf)*|[EOOLT 2017, December](http://www.eoolt.org/2017/)|[10.1145/3158191.3158193](https://doi.org/10.1145/3158191.3158193)|
| [2] |*[Component-Based 3D Modeling of Dynamic Systems](http://www.ep.liu.se/ecp/154/019/ecp18154019.pdf)*|[American Modelica Conference 2018, October](https://www.modelica.org/events/modelica2018Americas/index_html)|[10.3384/ECP18154175](https://doi.org/10.3384/ECP18154175)|
| [3] |*[Algorithms for Component-Based 3D Modeling](http://www.ep.liu.se/ecp/157/039/ecp19157039.pdf)*|[13th International Modelica Conference 2019, March](https://modelica.org/events/modelica2019)|[10.3384/ecp19157383](https://doi.org/10.3384/ecp19157383)|
| [4a] |*[Modia3D: Modeling and Simulation of 3D-Systems in Julia](https://proceedings.juliacon.org/papers/10.21105/jcon.00043)*|[JuliaCon 2019, July](https://juliacon.org/2019/)|[10.21105/jcon.00043](https://doi.org/10.21105/jcon.00043)|
| [4b] |*[Modia3D: Modeling and Simulation of 3D-Systems in Julia](https://proceedings.juliacon.org/papers/10.21105/jcon.00043)*|[JuliaCon 2019, July](https://juliacon.org/2019/)|[YouTube](https://www.youtube.com/watch?v=b3WfqXZRKpA)|
| [5] |*[Collision Handling with Elastic Response Calculation and Zero-Crossing Functions](https://doi.org/10.1145/3365984.3365986)*|[EOOLT 2019, November](http://www.eoolt.org/2019/)|[10.1145/3365984.3365986](https://doi.org/10.1145/3365984.3365986)|
| [6] |*[Modia – Modeling Multidomain Engineering Systems with Julia](https://www.youtube.com/watch?v=N94si3rOl1g)*|[JuliaCon 2021, July](https://juliacon.org/2021/)|[YouTube](https://www.youtube.com/watch?v=N94si3rOl1g)|
| [7a] |*[Modia – Equation Based Modeling and Domain Specific Algorithms](https://doi.org/10.3384/ecp2118173)*|[14th International Modelica Conference 2021, September](http://www.modelica.org/)| [10.3384/ecp2118173](https://doi.org/10.3384/ecp2118173) |
| [7b] |*[Modia – Equation Based Modeling and Domain Specific Algorithms](https://doi.org/10.3384/ecp2118173)*|[14th International Modelica Conference 2021, September](http://www.modelica.org/)| YouTube comming soon |




# Main developers

[Andrea Neumayr](mailto:andrea.neumayr@dlr.de),
[Martin Otter](https://rmc.dlr.de/sr/de/staff/martin.otter/) and
[Gerhard Hippmann](mailto:gerhard.hippmann@dlr.de),\
[DLR - Institute of System Dynamics and Control](https://www.dlr.de/sr/en)

License: MIT (expat)
