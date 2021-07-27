# License for this file: MIT (expat)
# Copyright 2017-2018, DLR Institute of System Dynamics and Control
#
# This file is part of module
#   Modia3D.Shapes (Modia3D/Shapes/_module.jl)
#

import Colors

const RGBColor = MVector{3,Cint}


"""
    color = rgb([name::String | vec::AbstractVector | r::Number,g::Number,b::Number])

defines a color as a 3-vector of RGB values. Colors from String are parsed by [Colors.jl](https://github.com/JuliaGraphics/Colors.jl).

# Examples

```julia
color1 = Modia3D.rgb("Red")            # = [255,0,0]
color2 = Modia3D.rgb("rgb(255,0,0)")   # = [255,0,0]
color3 = Modia3D.rgb("#FF0000")        # = [255,0,0]
color4 = Modia3D.rgb([255,0,0])        # = [255,0,0]
color5 = Modia3D.rgb(255,0,0)          # = [255,0,0]
```
"""
function rgb(name::String)
    try
        color = Colors.parse(Colors.Colorant, name)
        RGBColor(255*[Colors.red(color), Colors.green(color), Colors.blue(color)])
    catch
        default = defaultColor()
        @warn("Color $name is not supported, using $default.")
        color = Colors.parse(Colors.Colorant, default)
        RGBColor(255*[Colors.red(color), Colors.green(color), Colors.blue(color)])
    end
end
rgb(vec::AbstractVector)             = RGBColor(vec)
rgb(r::Number, g::Number, b::Number) = RGBColor(r, g, b)

defaultColor() = "Blue"
