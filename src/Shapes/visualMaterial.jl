# License for this file: MIT (expat)
# Copyright 2017-2018, DLR Institute of System Dynamics and Control
#
# This file is part of module
#   Modia3D.Shapes (Modia3D/Shapes/_module.jl)
#

import JSON

# Shadow definitions
@enum ShadowMask NoShadows=0 CastsShadows=1 ReceivesShadows=2 CastsAndReceivesShadows=3


# Material for visual elements
"""
    VisualMaterial(; color="Green",
                     wireframe=false, transparency=0.5,
                     reflectslight=true, shininess=0.7,
                     shadowMask=CastsAndReceivesShadows))

Return a material object that defines attributes for the visualization of an Object3D
that has visual or solid properties.

# Arguments
- `color` defines the material color as a String or RGB values. In case of Strings the color
          is constructed using [Colors.jl](https://github.com/JuliaGraphics/Colors.jl). So
          [named colors](https://juliagraphics.github.io/Colors.jl/stable/namedcolors/) and
          many other kinds of [color specifications](https://juliagraphics.github.io/Colors.jl/stable/constructionandconversion/#Color-Parsing)
          are available. In addition, colors can be directly defined by a vector of RGB values.
          E.g. `color="MediumVioletRed"` or `color=[255, 0, 0]`.
- `wireframe`: = false, if solid, otherwise wireframe representation.
- `transparency`: = 0.0 (opaque) ... 1.0 (fully transparent).
- `reflectslight`: = true if it reflects light and false, if it does not reflect light.
- `shininess`: = 0.0 (matte surface) ... 1.0 (very shiny surface).
- `shadowMask`: defines whether or not an object casts or receives shadows. Possible values:
   NoShadows, CastsShadows, ReceivesShadows, CastsAndReceivesShadows.
"""
mutable struct VisualMaterial
  color::MVector{3,Cint}  # RGB color
  wireframe::Bool         # if true, the wireframe of object is displayed
  transparency::Float64   # [0,1]... 0.0 is opaque, 1.0 is transparent
  reflectslight::Bool     # if true, specular Highlights are active
  shininess::Float64      # [0,1]... 0.0 is matte surface, 1.0 is very shiny
  shadowMask::ShadowMask  # defines whether or not an object casts or receives shadows 0: No Shadows, 1: Casts Shadows, 2: Receives Shadows, 3: Casts and Recieves Shadows

  function VisualMaterial(;color=defaultColor(), wireframe=false, transparency=0.0, reflectslight=true, shininess=0.7, shadowMask=CastsAndReceivesShadows)
    color2 = rgb(color)
    @assert(color2[1]>=0 && color2[1]<=255)
    @assert(color2[2]>=0 && color2[2]<=255)
    @assert(color2[3]>=0 && color2[3]<=255)
    @assert(transparency >= 0.0 && transparency <= 1.0)
    @assert(shininess    >= 0.0 && shininess    <= 1.0)
  new(color2, wireframe, transparency, reflectslight, shininess, shadowMask)
  end
end


"""
    readDictOfStructsFromJSONVisualMaterial(fileName, StructType)

Read a JSON file from `fileName` and return a `Dict{String, StructType}` dictionary.
`StructType` must be a mutable struct type with a constructor `StructType()`.
"""

function readDictOfStructsFromJSONVisualMaterial(fileName, StructType)
    dict1 = JSON.parsefile(fileName)
    palette = Dict{String, StructType}()
    for (key1,value1) in dict1
        obj = StructType()
        for (key2,value2) in value1
            if Symbol(key2) == Symbol("color")
              value2 = Shapes.rgb(value2)
            end
            setfield!(obj, Symbol(key2), value2)
        end
        palette[key1] = obj
    end
    return palette
end


"""
    readVisualMaterialFromJSON(fileName)

Read a JSON file consisting of a dictionary of VisualMaterial instances from `fileName` and
return a `Dict{String, VisualMaterial}` dictionary.
"""
readVisualMaterialFromJSON(fileName::AbstractString) = readDictOfStructsFromJSONVisualMaterial(fileName, VisualMaterial)


"""
    visualMaterialPalette

Dictionary of visual material data, see [`Modia3D.Shapes.VisualMaterial`](@ref)
"""
visualMaterialPalette = readVisualMaterialFromJSON( joinpath(Modia3D.path, "palettes", "visualMaterials.json") )

function rereadVisualMaterialFromJSON(; file="")
    if file == ""
        file = joinpath(Modia3D.path, "palettes", "visualMaterials.json")
    end
    global visualMaterialPalette = readVisualMaterialFromJSON( file )
    return nothing
end
