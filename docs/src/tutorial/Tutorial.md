# Modia3D Tutorial

This tutorial sketches the main features of Modia3D with examples. A detailed description is available in Modia3D Components.

!!! info
    TinyModia has an interface to various plot packages. A plot package can be
    either selected by setting `ENV["MODIA_PLOT"] = XXX`, for example in the `config/startup.jl`
    file of Julia or by command `TinyModia.usePlotPackage(XXX)`.
    All examples assume that a plot package is defined.
    Possible values for `XXX`:
    - "[PyPlot](https://github.com/JuliaPy/PyPlot.jl)" (plots with Matplotlib from Python),
    - "[GLMakie](https://github.com/JuliaPlots/GLMakie.jl)" (interactive plots in an OpenGL window),
    - "[WGLMakie](https://github.com/JuliaPlots/WGLMakie.jl)" (interactive plots in a browser window),
    - "[CairoMakie](https://github.com/JuliaPlots/CairoMakie.jl)" (static plots on file with publication quality),
    - "NoPlot" (= all `plot(...)` calls are ignored), or
    - "SilentNoPlot" (= NoPlot without messages).
