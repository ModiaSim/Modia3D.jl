# Modia3D

Modia3D is a Julia package to model fixed and moving objects in 3D (*e.g.* visual shapes, rigid bodies).
These objects are driven kinematically by pre-defined time functions or are moving dynamically by
solving Differential Algebraic Equations (DAEs)
with a variable-step DAE solver. Collision handling with elastic response calculation is
performed for objects that are defined with a contact material and (a) have a convex geometry,
(b) can be approximated by a set of convex geometries, or (c) have a concave geometry
that is (automatically) approximated by its convex hull.
A more detailed overview of the available features is given in the
[Modia3D documentation](https://ModiaSim.github.io/Modia3D.jl/latest).
Papers about Modia3D:

- *[Collision Handling with Variable-Step Integrators](docs/resources/documentation/CollisionHandling_Neumayr_Otter_2017.pdf)* ([EOOLT 2017, December](http://www.eoolt.org/2017/))
- *Component-Based 3D Modeling Combined with Equation-Based Modeling*, accepted for publication at the
  [American Modelica Conference 2018, October 9-10](https://www.modelica.org/events/modelica2018Americas/index_html)

Before releasing version 1.0, Modia3D shall be
easily combinable with [Modia](https://github.com/ModiaSim/Modia.jl), for example to define a controlled
electrical motor with Modia, and add 3D behavior/visualization with Modia3D.
By this approach the best of both worlds can be combined:
Special 3D algorithms (Modia3D) + power/flexibility of equation based modeling (Modia).


## Installation

This package is not yet registered in `METADATA.jl`and need to be installed with `Pkg.clone`:

```
julia> Pkg.clone("https://github.com/ModiaSim/ModiaMath.jl")
julia> Pkg.clone("https://github.com/ModiaSim/Modia3D.jl")
```

Modia3D performs simulation and plotting with ModiaMath. ModiaMath in turn
uses `PyPlot` as basis for the plotting. Since installation of `PyPlot` is not
robust with the automatic installation procedure of current Julia, it is recommended
to first install `PyPlot` as described in the
[installation procedure of ModiaMath](https://modiasim.github.io/ModiaMath.jl/latest/index.html#Installation-1).

Modia3D visualizes the movement of 3D objects with a renderer.
Currently, the (free) community or the (commercial) professional version of the
[DLR Visualization](http://www.systemcontrolinnovationlab.de/the-dlr-visualization-library/) library
are supported. To install the free version for *Windows* perform the following steps
(the free *Linux* version will become available in a few days):

1. Go to [https://visualization.ltx.de/](https://visualization.ltx.de/),
   provide your contact information and click on *Request download* for *Community Edition*.
   Afterwards, you get a link to download the library and you need to unzip the file.
2. In your `HOME/.juliarc.jl` file, include the environment variable
   `ENV["DLR_VISUALIZATION"] = "<path-to-library>/Visualization/Extras/SimVis"`.
3. Start Julia and run one of the examples, for example
   `include("$(Modia3D.path)/examples/dynamics/Simulate_DoublePendulumWithDampers.jl")`

If Modia3D cannot use one of the renderers above, it will continue with renderer **NoRenderer**
that is animation is switched off.

## Documentation

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
  include("$(Modia3D.path)/examples/dynamics/Simulate_DoublePendulumWithDampers.jl")
  include("$(Modia3D.path)/examples/dynamics/Simulate_FallingBall4.jl")
  include("$(Modia3D.path)/examples/kinematics/Move_FourBar.jl")
  include("$(Modia3D.path)/examples/visual/Move_AllVisualObjects.jl")
```

### To run tests
```julia
  include("$(Modia3D.path)/test/runtests.jl")
```

## Status

The package is tested against Julia `0.6` on Windows 7, Kubuntu 18.04, Ubuntu 14.04, OpenSUSE42 and Fedora 28.
The actual version number is 0.2.0-beta.1 and functionality and robustness is planned to be improved for the 1.0 version.

Especially note, the collision handling has still bugs and the elastic response calculation is not
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

