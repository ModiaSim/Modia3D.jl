# Getting Started


## Installation

Modia3D is not yet registered in `METADATA.jl`and need to be installed with `Pkg.clone`:

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



## To run examples

```julia
  include("$(Modia3D.path)/examples/dynamics/Simulate_DoublePendulumWithDampers.jl")
  include("$(Modia3D.path)/examples/dynamics/Simulate_FallingBall4.jl")
  include("$(Modia3D.path)/examples/kinematics/Move_FourBar.jl")
  include("$(Modia3D.path)/examples/visual/Move_AllVisualObjects.jl")
```

## To run tests

```julia
  include("$(Modia3D.path)/test/runtests.jl")
```