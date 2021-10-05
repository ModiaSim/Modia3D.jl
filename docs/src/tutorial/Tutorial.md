# Modia3D Tutorial

This tutorial sketches the main features of Modia3D with examples. A detailed description is available in Modia3D Components.

## Plotting

Modia offers an interface to various plot packages. Please, read also the [installation guide](https://github.com/ModiaSim/Modia3D.jl/wiki/Full-Installation-Guide-for-Julia,-Modia3D,-Modia). Valid options are:

| Environment option | Modia package | Description |
|:---|:--------------|:---|
|"[GLMakie](https://github.com/JuliaPlots/GLMakie.jl)"| [ModiaPlot_GLMakie](https://github.com/ModiaSim/ModiaPlot_GLMakie.jl) | interactive plots in an OpenGL window |
|"[WGLMakie](https://github.com/JuliaPlots/WGLMakie.jl)"| [ModiaPlot_WGLMakie](https://github.com/ModiaSim/ModiaPlot_WGLMakie.jl) | interactive plots in a browser window |
|"[CairoMakie](https://github.com/JuliaPlots/CairoMakie.jl)"| [ModiaPlot_CairoMakie](https://github.com/ModiaSim/ModiaPlot_CairoMakie.jl) | static plots on file with publication quality |
|"[PyPlot](https://github.com/JuliaPy/PyPlot.jl)"| [ModiaPlot_PyPlot](https://github.com/ModiaSim/ModiaPlot_PyPlot.jl) | plots with Matplotlib from Python |
|"NoPlot"| | all `plot(...)` calls are ignored |
|"SilentNoPlot"| | like "NoPlot" without messages |

First, you need to add the corresponding Modia package, e.g.
```julia
julia> ] add ModiaPlot_GLMakie
```
A plot package can be either selected by setting `ENV["MODIA_PLOT"] = "GLMakie"`, in the [startup.jl file](https://github.com/ModiaSim/Modia3D.jl/wiki/Template-for-startup.jl) of Julia or with command `Modia.usePlotPackage("GLMakie")`.


## 3D Renderer

### Three.js

Modia3D animation can be exported in threejs-json-format and then imported in the open source web app [threejs.org](https://threejs.org/editor/) and use all the features of threejs, for example to export in the widely used glb format (= the binary version of the [glTF](https://www.khronos.org/gltf/) format) and use any glb-viewer (for example 3D-Viewer of Windows 10).

Objects are visualized and exported for offline animation by defining `animationFile = "Pendulum2.json"` in [Scene](@ref), see [2. Pendulum with Animation](@ref).

### The DLR Visualization Library

Modia3D uses the (free) community or the (commercial) professional version of the [DLR Visualization Library](https://visualization.ltx.de/) for 3D simulations. It provides online animation (during simulation) and the generation of mpg4-videos. If you don't use the DLR Visualization Library result animation is switched of.
Download and install the free DLR SimVis Community Edition, e.g. with [https://visualization.ltx.de/](https://visualization.ltx.de/) .

* Set an environment variable or add it to the [startup.jl file](https://github.com/ModiaSim/Modia3D.jl/wiki/Template-for-startup.jl). Please, read also the [installation guide](https://github.com/ModiaSim/Modia3D.jl/wiki/Full-Installation-Guide-for-Julia,-Modia3D,-Modia).
   * `ENV["DLR_VISUALIZATION"] = "<path-to-library>/Visualization/Extras/SimVis"`
   * Make sure that the SimVis executable under this directory has execution rights. For example in Linux with command: `chmod ug+x <path-to-library>/Visualization/Extras/SimVis/linux/SimVis`
