# Modia3D

[![Travis](https://travis-ci.org/ModiaSim/Modia3D.jl.svg?branch=master)](https://travis-ci.org/ModiaSim/Modia3D.jl)
[![AppVoyer](https://ci.appveyor.com/api/projects/status/github/ModiaSim/Modia3D.jl?svg=true)](https://ci.appveyor.com/project/MartinOtter/modia3d-jl)
[![codecov.io](http://codecov.io/github/ModiaSim/Modia3D.jl/coverage.svg?branch=master)](http://codecov.io/github/ModiaSim/Modia3D.jl?branch=master)
[![Stable](https://img.shields.io/badge/docs-stable-blue.svg)](https://ModiaSim.github.io/Modia3D.jl/stable)
[![Latest](https://img.shields.io/badge/docs-latest-blue.svg)](https://ModiaSim.github.io/Modia3D.jl/latest)
[![The MIT License](https://img.shields.io/badge/license-MIT-brightgreen.svg?style=flat-square)](https://github.com/ModiaSim/Modia3D.jl/blob/master/LICENSE.md)


Modia3D is a Julia package to model fixed and moving objects in 3D (*e.g.* visual shapes, rigid bodies).
These objects are driven kinematically by pre-defined time functions or are moving dynamically by
solving Differential Algebraic Equations (DAEs)
with a variable-step DAE solver.

Collision handling with elastic response calculation is
performed for objects that are defined with a contact material and (a) have a convex geometry,
(b) can be approximated by a set of convex geometries, or (c) have a concave geometry
that is (automatically) approximated by its convex hull.
A more detailed overview of the available features is given in the
[Modia3D documentation](https://ModiaSim.github.io/Modia3D.jl/stable).
Papers about Modia3D:

- *[Collision Handling with Variable-Step Integrators](docs/resources/documentation/CollisionHandling_Neumayr_Otter_2017.pdf)* ([EOOLT 2017, December](http://www.eoolt.org/2017/))
- *[Component-Based 3D Modeling of Dynamic Systems](http://www.ep.liu.se/ecp/154/019/ecp18154019.pdf)* ([American Modelica Conference 2018, October](https://www.modelica.org/events/modelica2018Americas/index_html))
- *[Algorithms for Component-Based 3D Modeling](http://www.ep.liu.se/ecp/157/039/ecp19157039.pdf)* ([13th International Modelica Conference 2019, March](https://modelica.org/events/modelica2019))


Before releasing version 1.0, Modia3D shall be
easily combinable with [Modia](https://github.com/ModiaSim/Modia.jl), for example to define a controlled
electrical motor with Modia, and add 3D behavior/visualization with Modia3D.
By this approach the best of both worlds can be combined:
Special 3D algorithms (Modia3D) + power/flexibility of equation based modeling (Modia).


## Modia Project

The [Modelica standard library](https://github.com/modelica/ModelicaStandardLibrary) supports the modeling of 3-dimensional multi-body systems with its sub library Modelica.Mechanics.MultiBody. There have been several attempts to improve this library with regards to visualization, collision handling or support of larger models, for example. Over the years it was recognized that the technology of current [Modelica](https://www.modelica.org) has some natural limitations.

Therefore, the open source [Modia](https://github.com/ModiaSim/Modia.jl) project was launched as a domain-specific extension of the  [Julia](https://julialang.org) programming language. The equation-based modeling language is called Modia as well as the experimental [modeling environment](https://github.com/ModiaSim) consisting of several Julia packages. The intention is to utilize the results of this prototyping in the design of the next Modelica language generation.


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
that is animation is switched off.


## Documentation

- [**STABLE**](https://ModiaSim.github.io/Modia3D.jl/stable) &mdash; *documentation of the last released version.*
- [**LATEST**](https://ModiaSim.github.io/Modia3D.jl/latest) &mdash; *in-development version of the documentation.*


## Use

### To define a model
```julia
import ModiaMath
using Modia3D

material1 = Modia3D.Material(color="LightBlue", transparency=0.3);
material2 = Modia3D.Material(color="Red");

@assembly Pendulum(;Lx = 1.0, Ly=0.2*Lx, Lz=0.2*Lx) begin
   world       = Object3D(Modia3D.CoordinateSystem(0.5*Lx))
   beam_frame0 = Object3D(Modia3D.Solid(Modia3D.SolidBeam(Lx,Ly,Lz), "Aluminium", material1))
   beam_frame1 = Object3D(beam_frame0; r=[-Lx/2, 0.0, 0.0])
   cylinder    = Object3D(beam_frame1,Modia3D.Cylinder(Ly/2,1.2*Ly; material=material2))
   revolute    = Modia3D.Revolute(world, beam_frame1)
end;
simulationModel = Modia3D.SimulationModel(Pendulum(Lx=0.8),stopTime=5.0);
```


### To simulate a model, animate and plot results

```julia
result = ModiaMath.simulate!(simulationModel);
ModiaMath.plot(result, ["revolute.phi", "revolute.w"]);
```

![PendulumPlot](docs/resources/images/pendulum_readme.png)


### To run examples
```julia
  import Modia3D
  include("$(Modia3D.path)/examples/dynamics/Simulate_DoublePendulumWithDampers.jl")
  include("$(Modia3D.path)/examples/dynamics/Simulate_FallingBall4.jl")
  include("$(Modia3D.path)/examples/kinematics/Move_FourBar.jl")
  include("$(Modia3D.path)/examples/visual/Move_AllVisualObjects.jl")

```

### To run tests
```julia
  import Modia3D
  include("$(Modia3D.path)/test/runtests.jl")
```

## Status

The package has been tested with Julia on Windows 7,
via the TravisCL on Linux (x86_64-pc-linux-gnu) and macOS (x86_64-apple-darwin14.5.0)
and via the Appveyor CL on Windows.

Note, the collision handling has still bugs and the elastic response calculation is not
yet robust. This needs to be improved.

Furthermore, kinematic loops are currently only supported for 2D loops and if they are driven kinematically.
The technique to handle kinematic loops for dynamic simulations is demonstrated in the example
`include("$(ModiaMath.path)/examples/withoutMacros_withoutVariables/Simulate_PendulumDAE.jl")`
and is described in the paper *[Transformation of Differential Algebraic Array Equations to
Index One Form](http://www.ep.liu.se/ecp/132/064/ecp17132565.pdf)*.


## Issues and Contributions

Contributions are welcome, as are feature requests and suggestions.
Please open an [issue](https://github.com/Modia/Modia3D.jl/issues) in this case and also if you encounter problems.


## Main Developers and License
[Andrea Neumayr](mailto:andrea.neumayr@dlr.de) and [Martin Otter](https://rmc.dlr.de/sr/de/staff/martin.otter/)

[DLR - Institute of System Dynamics and Control](https://www.dlr.de/sr/en)

License: [MIT (expat)](LICENSE.md)
