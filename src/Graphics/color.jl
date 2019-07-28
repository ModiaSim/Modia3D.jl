# License for this file: MIT (expat)
# Copyright 2017-2018, DLR Institute of System Dynamics and Control
#
# This file is part of module
#   Modia3D.Graphics (Modia3D/Graphics/_module.jl)
#

# Color definitions (default colors are read from file src\Graphics\colors.json)

import JSON

const defaultColorFile = joinpath(Modia3D.path, "palettes", "colors.json")
const RGBColor         = MVector{3,Cint}


"""
    const colorPalette

Dictionary with default colors.
"""
const colorPalette = JSON.parsefile(defaultColorFile; dicttype=Dict{String, RGBColor})


"""
    color = rgb([name::String | vec::AbstractVector | r::Number,g::Number,b::Number])

defines the color as a 3-vector of RGB values. Currently, the following names of colors are defined:
Black, DarkRed, Red, LightRed, DarkGreen, Green, LightGreen, DarkBlue, Blue,
LightBlue, Yello, Pink DarkGrey, Grey, White.


# Examples

```julia
color1 = Modia3D.rgb("Red")       # = [255,0,0]
color2 = Modia3D.rgb([255,0,0])   # = [255,0,0]
color3 = Modia3D.rgb(255,0,0)     # = [255,0,0]
```
"""
rgb(name::String)                    = RGBColor( copy(colorPalette[name]) )
rgb(vec::AbstractVector)             = RGBColor(vec)
rgb(r::Number, g::Number, b::Number) = RGBColor(r,g,b)

defaultColor() = "Blue"
