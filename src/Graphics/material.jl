# License for this file: MIT (expat)
# Copyright 2017-2018, DLR Institute of System Dynamics and Control
#
# This file is part of module 
#   Modia3D.Graphics (Modia3D/Graphics/_module.jl)
#


# Shadow definitions
@enum ShadowMask NoShadows=0 CastsShadows=1 ReceivesShadows=2 CastsAndReceivesShadows=3


# Material for visual elements
"""
    material = Modia3D.Material(;color=defaultColor(), wireframe=false, transparency=0.0,
                                 reflectslight=true, shininess=0.7, 
                                 shadowMask=CastsAndReceivesShadows))

Return a material object that defines attributes for the visualization of an Object3D 
that has visual or solid properties.

# Arguments
- `color`: This argument is passed to function [`Modia3D.rgb`](@ref)`(color)` to return the RGB color value in
           form of a vector. E.g. `color="Red"` or `color=[255,0,0]`.
- `wireframe`: = false, if solid, otherwise wireframe representation.
- `transparency`: = 0.0 (opaque) ... 1.0 (fully transparent).
- `reflectslight`: = true if it reflects light and false, if it does not reflect light.
- `shininess`: = 0.0 (matte surface) ... 1.0 (very shiny surface).
- `shadowMask`: defines whether or not an object casts or receives shadows. Possible values:
   NoShadows, CastsShadows, ReceivesShadows, CastsAndReceivesShadows.
"""
mutable struct Material
  color::MVector{3,Cint}  # RGB color
  wireframe::Bool         # if true, the wireframe of object is displayed
  transparency::Float64   # [0,1]... 0.0 is opaque, 1.0 is transparent
  reflectslight::Bool     # if true, specular Highlights are active
  shininess::Float64      # [0,1]... 0.0 is matte surface, 1.0 is very shiny
  shadowMask::ShadowMask  # defines whether or not an object casts or receives shadows 0: No Shadows, 1: Casts Shadows, 2: Receives Shadows, 3: Casts and Recieves Shadows

  function Material(;color=defaultColor(), wireframe=false, transparency=0.0, reflectslight=true, shininess=0.7, shadowMask=CastsAndReceivesShadows)
    color2 = rgb(color)
    @assert(color2[1]>=0 && color2[1]<=255)
    @assert(color2[2]>=0 && color2[2]<=255)
    @assert(color2[3]>=0 && color2[3]<=255)
    @assert(transparency >= 0.0 && transparency <= 1.0)
    @assert(shininess    >= 0.0 && shininess    <= 1.0)
    new(color2, wireframe, transparency, reflectslight, shininess, shadowMask)
  end
end
