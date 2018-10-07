# Getting Started


## Installation

**Modia3D** is registered in METADATA.jl and can be installed with Pkg.add:

```julia
# Julia 0.6:
julia> Pkg.add("Modia3D")
julia> Pkg.add("ModiaMath")   # in order to use simulate!(..) and plot(..)
julia> Pkg.add("PyPlot")      # in order that plots are shown

# Julia 0.7 and 1.0 (registration is pending)
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

2. In your Julia **startup file** (Julia 0.6: `HOME/.juliarc.jl`, Julia 0.7 and 1.0:
   `HOME/.julia/config/startup.jl`), include the environment variable
   `ENV["DLR_VISUALIZATION"] = "<path-to-library>/Visualization/Extras/SimVis"`.
   Make sure that the SimVis executable under this directory has execution rights.
   For example in Linux with command:
   `chmod +x <path-to-library>/Visualization/Extras/SimVis`

3. Start Julia and run one of the examples, for example
   `include("$(Modia3D.path)/examples/dynamics/Simulate_DoublePendulumWithDampers.jl")`

If Modia3D cannot use one of the renderers above, it will continue with renderer **NoRenderer**
that is animation is switched off.




## To run examples

```julia
  import Modia3D
  include("$(Modia3D.path)/examples/dynamics/Simulate_DoublePendulumWithDampers.jl")
  include("$(Modia3D.path)/examples/dynamics/Simulate_FallingBall4.jl")
  include("$(Modia3D.path)/examples/kinematics/Move_FourBar.jl")
  include("$(Modia3D.path)/examples/visual/Move_AllVisualObjects.jl")
```

## To run tests

```julia
  import Modia3D
  include("$(Modia3D.path)/test/runtests.jl")
```