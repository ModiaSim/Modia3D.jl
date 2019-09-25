# Modia3D.jl

[![Travis](https://travis-ci.org/ModiaSim/Modia3D.jl.svg?branch=master)](https://travis-ci.org/ModiaSim/Modia3D.jl)
[![AppVoyer](https://ci.appveyor.com/api/projects/status/github/ModiaSim/Modia3D.jl?svg=true)](https://ci.appveyor.com/project/MartinOtter/modia3d-jl)
[![codecov.io](http://codecov.io/github/ModiaSim/Modia3D.jl/coverage.svg?branch=master)](http://codecov.io/github/ModiaSim/Modia3D.jl?branch=master)
[![Stable](https://img.shields.io/badge/docs-stable-blue.svg)](https://ModiaSim.github.io/Modia3D.jl/stable)
[![Latest](https://img.shields.io/badge/docs-latest-blue.svg)](https://ModiaSim.github.io/Modia3D.jl/latest)
[![The MIT License](https://img.shields.io/badge/license-MIT-brightgreen.svg?style=flat-square)](https://github.com/ModiaSim/Modia3D.jl/blob/master/LICENSE.md)


## Modia Project
The [Modelica standard library](https://github.com/modelica/ModelicaStandardLibrary) supports the modeling of 3-dimensional multi-body systems with its sub library Modelica.Mechanics.MultiBody. There have been several attempts to improve this library with regards to visualization, collision handling or support of larger models. Over the years it was recognized that the technology of current [Modelica](https://www.modelica.org) has some natural limitations.

Therefore, the open source [Modia](https://github.com/ModiaSim/Modia.jl) project was launched as a domain-specific extension of the  [Julia](https://julialang.org) programming language. The equation-based modeling language is called Modia as well as the experimental [modeling environment](https://github.com/ModiaSim) consisting of several Julia packages. The intention is to utilize the results of this research and prototyping in the design of the next Modelica language generation.


## Modia3D
One part of the Modia project is [Modia3D](https://github.com/ModiaSim/Modia3D.jl). Initially, it is an experimental modeling and simulation environment for 3D mechanical systems, but it shall be expanded into other domains in the future.

Ideas from modern computer game engines are used to achieve a highly flexible setup of mechanical systems including collision handling. Other features are utilized from multi-body programs, such as hierarchical structuring, support for closed kinematic loops, and elastic response calculation.
The underlying mathematical formulation are hybrid Differential Algebraic Equations (DAEs) that are solved with the variable-step solver IDA via the [Sundials.jl](https://github.com/JuliaDiffEq/Sundials.jl) Julia package.


Features of the Modia3D Julia package are modeling fixed and moving objects in 3D (e.g. visual shapes, rigid bodies). These objects are driven kinematically by pre-defined time functions or are moving dynamically by solving Differential Algebraic Equations (DAEs) with a variable-step DAE solver.

Further, collision handling with elastic response calculation is performed for objects that are defined with a contact material and have a convex geometry, or can be approximated by a set of convex geometries, or have a concave geometry which is (automatically) approximated by its convex hull.
Collision handling with elastic response calculation and error controlled integration is challenging.
Contact handling in Modia3D uses variable-step solvers where the penetration depths and Euclidean distances computed with the improved Minkowski Portal Refinement (MPR) algorithm [1] are utilized to construct appropriate zero-crossing functions.
Whenever a collision with elastic response calculation occurs, the model response is drastically changed. An advanced contact law is introduced in [5] and many detailed examples using it are discussed in [4]. The user's view of Modia3D is introduced in [2] showing the very flexible definition of 3D systems. Some key algorithms are discussed in [3].
A more detailed overview of the available features is also given in the [Modia3D documentation](https://ModiaSim.github.io/Modia3D.jl/stable).

Papers and videos about Modia3D:

- [1] *[Collision Handling with Variable-Step Integrators](docs/resources/documentation/CollisionHandling_Neumayr_Otter_2017.pdf)* ([EOOLT 2017, December](http://www.eoolt.org/2017/))
- [2] *[Component-Based 3D Modeling of Dynamic Systems](http://www.ep.liu.se/ecp/154/019/ecp18154019.pdf)* ([American Modelica Conference 2018, October](https://www.modelica.org/events/modelica2018Americas/index_html))
- [3] *[Algorithms for Component-Based 3D Modeling](http://www.ep.liu.se/ecp/157/039/ecp19157039.pdf)* ([13th International Modelica Conference 2019, March](https://modelica.org/events/modelica2019))
- [4] *Modia3D: Modeling and Simulation of 3D-Systems in Julia* ([JuliaCon 2019, July](https://juliacon.org/2019/), under review); talk recorded on [YouTube](https://www.youtube.com/watch?v=b3WfqXZRKpA)
- [5] *Collision Handling with Elastic Response Calculation and Zero-Crossing Functions* (will be published on ([EOOLT 2019, November](http://www.eoolt.org/2019/))

### 3D Renderer and Work-In-Progress Web App
Modia3D provides a generic interface to visualize simulation results with different 3D renderers. The [free community edition](https://visualization.ltx.de/) as well as the professional edition of the
[DLR Visualization library](http://www.systemcontrolinnovationlab.de/the-dlr-visualization-library/) are supported. Currently, another team is developing a free 2D/3D web-based authoring tool that includes result visualization, but it is not available at the moment.

### Goals for Modia3D
Before releasing version 1.0, Modia3D should be combined with equation-based modeling, using Modia3D assemblies as components within one high level programming environment (for example a joint of a Modia3D system is driven by a Modia model of an electrical motor and gearbox). Therefore use the best of both worlds: special 3D algorithms (Modia3D) and the power/flexibility of equation based modeling (Modia).


## Installation
**Modia3D** is registered in METADATA.jl and can be installed in the following way (Julia >= 1.0 is required):

```julia
julia> ]add Modia3D
        add ModiaMath  # in order to use simulate!(..) and plot(..)
        add PyPlot     # in order that plots are shown
```

Modia3D uses [PyPlot](https://github.com/JuliaPy/PyPlot.jl) for plotting.
If `PyPlot` is not available in your current Julia environment
an information message is printed and all `plot(..)` calls are ignored.

In order that plot windows are displayed, you need to add `PyPlot` to your current environment
via `]add PyPlot`. Often this automatic installation fails and it is recommended to follow
the instructions
[Installing PyPlot in a robust way](https://github.com/ModiaSim/ModiaMath.jl/wiki/Installing-PyPlot-in-a-robust-way).

Modia3D visualizes the movement of 3D objects with a renderer.
Currently, the (free) community or the (commercial) professional version of the
[DLR Visualization](http://www.systemcontrolinnovationlab.de/the-dlr-visualization-library/) library
are supported. To install the free version for *Windows* or for *Linux* perform the following steps:

1. Go to [https://visualization.ltx.de/](https://visualization.ltx.de/),
   provide your contact information and click on *Request download* for *Community Edition*.
   Afterwards, you get a link to download the library and you need to unzip the file.

2. In your Julia **startup file** (`HOME/.julia/config/startup.jl`) include the environment variable
   `ENV["DLR_VISUALIZATION"] = "<path-to-library>/Visualization/Extras/SimVis"`.
   Make sure that the SimVis executable under this directory has execution rights.
   For example in Linux with command:
   `chmod +x <path-to-library>/Visualization/Extras/SimVis`

3. Start Julia and run one of the examples, for example
   `include("$(Modia3D.path)/examples/dynamics/Simulate_DoublePendulumWithDampers.jl")`

If Modia3D cannot use one of the renderers above, it will continue with renderer **NoRenderer**
where result animation is switched off.

## Usage of Modia3D by Example
Modia3D has a very modular and flexible component-based design, and it allows hierarchical structuring. The following example, of a single pendulum, gives a short insight of both concepts. For further information see [2, 3] and the documentation.

### To define a model
The [ModiaMath]( https://github.com/ModiaSim/ModiaMath.jl) package is imported. It belongs also to the Modia modeling environment, and it is needed to solve and communicate with the IDA DAE-solver.

Two visualization materials (visuMaterial1, visuMaterial2) are defined, and optional parameters (color, transparency) are set. There are further optional parameters, but they are not used here.

The component-based structuring is realized with the Object3D function. It is a coordinate system in 3D with optional associated data, and it is allowed to move freely or relatively to its parent Object3D. It is modular and flexible. Therefore, any meaningful combinations of components are allowed.

The world Object3D takes as input argument a coordinate system and the beam Object3D is a solid. In this setting the blue beam is a solid beam made of aluminium, with its dimensions Lx, Ly, Lz. The bearing Object3D's parent is the beam Object3D, and it is positioned relatively to its parent, with the optional argument r. The red cylinder Object3D its parent is the bearing. It is for representing a revolute joint between the world and the bearing.

The hierarchical structuring is realized with the @assembly macro. It is for structuring, grouping features, and allows the easy reusability of these groups.  
A simulation model consists of the top assembly, and other properties needed for simulation, e.g the pendulum is simulated for 5 sec.

```julia
import ModiaMath
using Modia3D

visuMaterial1 = Modia3D.Material(color="LightBlue", transparency=0.3);
visuMaterial2 = Modia3D.Material(color="Red");

@assembly Pendulum(;Lx = 1.0, Ly=0.2*Lx, Lz=0.2*Lx) begin
   world    = Object3D(Modia3D.CoordinateSystem(0.5*Lx))
   beam     = Object3D(Modia3D.Solid(Modia3D.SolidBeam(Lx,Ly,Lz), "Aluminium", visuMaterial1))
   bearing  = Object3D(beam; r=[-Lx/2, 0.0, 0.0])
   cylinder = Object3D(bearing,Modia3D.Cylinder(Ly/2,1.2*Ly; material=visuMaterial2))
   revolute = Modia3D.Revolute(world, bearing)
end;
simulationModel = Modia3D.SimulationModel(Pendulum(Lx=0.8),stopTime=5.0);
```


### To simulate a model, animate and plot results
The model is simulated with the ModiaMath command simulate!, and the results are plotted (see right figure). A 3D animation is shown, if the DLR Visualization Library is enabled (see left figure).

```julia
result = ModiaMath.simulate!(simulationModel);
ModiaMath.plot(result, ["revolute.phi", "revolute.w"]);
```

![PendulumPlot](docs/resources/images/pendulum_readme.png)


### To run examples
This is just a selection of examples. There are further [examples](https://github.com/Modia/examples) available (see Modia3D/examples).
```julia
  import Modia3D
  include("$(Modia3D.path)/examples/dynamics/Simulate_DoublePendulumWithDampers.jl")
  include("$(Modia3D.path)/examples/collisions/Simulate_NewtonsCradle.jl")
  include("$(Modia3D.path)/examples/kinematics/Move_FourBar.jl")
  include("$(Modia3D.path)/examples/visual/Move_AllVisualObjects.jl")

```

### To run tests
To make sure the available version of Modia3D is working, please execute runtests.jl. Additionally to unit tests there are further [test examples](https://github.com/Modia/test) available.
```julia
  import Modia3D
  include("$(Modia3D.path)/test/runtests.jl")
```

## Status
The package has been tested with Julia on Windows 7, via the TravisCL on Linux (x86_64-pc-linux-gnu) and macOS (x86_64-apple-darwin14.5.0) and via the Appveyor CL on Windows.

Note, the collision handling has still bugs and the elastic response calculation is not yet robust. This needs to be improved.

Furthermore, kinematic loops are currently only supported for 2D loops and if they are driven kinematically.
The technique to handle kinematic loops for dynamic simulations is demonstrated in the example
`include("$(ModiaMath.path)/examples/withoutMacros_withoutVariables/Simulate_PendulumDAE.jl")`
and is described in the paper *[Transformation of Differential Algebraic Array Equations to
Index One Form](http://www.ep.liu.se/ecp/132/064/ecp17132565.pdf)*.

Up to now, Modia3D is implemented for functionality and not tuned for efficiency. Therefore, there are no benchmarks yet and in particular no comparison with Modelica models.

## Documentation
- [**STABLE**](https://ModiaSim.github.io/Modia3D.jl/stable) &mdash; *documentation of the last released version.*
- [**LATEST**](https://ModiaSim.github.io/Modia3D.jl/latest) &mdash; *in-development version of the documentation.*

## Issues and Contributions
Contributions are welcome, as are feature requests and suggestions.
Please open an [issue](https://github.com/Modia/Modia3D.jl/issues) in this case and also if you encounter problems.


## Main Developers and License
[Andrea Neumayr](mailto:andrea.neumayr@dlr.de) and [Martin Otter](https://rmc.dlr.de/sr/de/staff/martin.otter/)

[DLR - Institute of System Dynamics and Control](https://www.dlr.de/sr/en)

License: [MIT (expat)](LICENSE.md)
