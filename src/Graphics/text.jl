# License for this file: MIT (expat)
# Copyright 2017-2018, DLR Institute of System Dynamics and Control
#
# This file is part of module
#   Modia3D.Graphics (Modia3D/Graphics/_module.jl)
#
# Content:
#   This file contains elements to show text in the visualization window


const fontFamilyPalette = Dict{String,String}([("FreeSans"     , "FreeSans"),
                                               ("FreeSerif"    , "FreeSerif"),
                                               ("Arial"        , "ARIAL"),
                                               ("ArialNarrow"  , "ARIALN"),
                                               ("CourierNew"   , "COUR"),
                                               ("TimesNewRoman", "Times"),
                                               ("Verdana"      , "verdana")
                                               ])

function getFontFileName(fontFamily::String, bold::Bool, italic::Bool)::String
   # Handle special cases first

   if fontFamily == "ArialNarrow" && bold && !italic
      fontFileName = "ARIALNB"
   elseif fontFamily == "Verdana"
      fontFileName = bold ? ( italic ? "verdanaz" : "verdanab") :
                            ( italic ? "verdanai" : "verdana" )
   else
      if !haskey(fontFamilyPalette, fontFamily)
         error("\nError from Modia3D.visualizer: fontFamily = \"", fontFamily, "\" is not supported.\n",
               "Supported font families: FreeSans, FreeSerif, Arial, ArialNarrow, CourierNew, TimesNewRoman, Verdana.")
      end
      name = fontFamilyPalette[fontFamily]
      fontFileName = bold ? ( italic ? name * "BI" : name * "BD" ) :
                            ( italic ? name * "I"  : name )
   end
   fontFileName = fontFileName * ".TTF"
end



"""
    font = Modia3D.Font(;fontFamily="FreeSans", bold=false, italic=false, charSize=0.1,
                         color="LightBlue", transparency=0.0)

Return a Font definition.

# Arguments
- `fontFamily::String`: Font family ("FreeSans", "FreeSerif", "Arial", "ArialNarrow", "CourierNew", "TimesNewRoman", or "Verdana").
- `bold::Bool`: = true, if bold font.
- `italic::Bool`: = true, if italic font.
- `charSize::Number`: Character size in [m].
- `color::Modia3D.RGBColor`: Color; Examples: rgb("Blue"), rgb([0,0,255]), rgb(0,0,255).
- `transparency::Number`: 0.0 (opaque) ... 1.0 (transparent)

# Examples
```julia
import Modia3D
font1 = Modia3D.Font()
font2 = Modia3D.Font(fontFamily="Arial", bold=true, charSize=0.2, 
                     color="LightBlue", transparency=0.5)
```
"""
struct Font
   fontFamily::String
   bold::Bool
   italic::Bool
   charSize::Float64
   color::RGBColor
   transparency::Float64

   # Constructed by constructor
   fontFileName::String  # File name of font

   function Font(;fontFamily::String    = "FreeSans",
                  bold::Bool            = false,
                  italic::Bool          = false,
                  charSize::Number      = 0.1,
                  color                 = "LightBlue",
                  transparency::Number  = 0.0)
      @assert(charSize >= 0.0)
      @assert(transparency >= 0.0 && transparency <= 1.0)
      fontFileName = getFontFileName(fontFamily, bold, italic)

      new(fontFamily, bold, italic, charSize, rgb(color), transparency, fontFileName)
   end
end

@enum AxisAlignment::Cint Screen=0 XY_Plane=1 XZ_Plane=2 YZ_Plane=3
@enum Alignment::Cint     Left=1 Right=2 Center=3



"""
    textShape = Modia3D.TextShape(text; font=Modia3D.Font(), offset=[0.0,0.0,0.0], 
                          axisAlignment=Modia3D.Screen, alignment=Modia3D.Center)

Return a text shape.

# Arguments
- `text::AbstractString`: String of the text.
- `font::`[`Modia3D.Font`](@ref): Font of the text.
- `offset::AbstractVector`: Offset from origin to text alignment point.
- `axisAlignment::Modia3D.AxisAlignment`: Alignment of Text (parallel to screen or in
   planes of frame: = Modia3D.Screen, Modia3D.XY_Plane, Modia3D.XZ_Plane, Modia3D.YZ_Plane).
- `axisAlignment::Modia3D.Alignment`: Alignment of Text relative to its origin
  (= Modia3D.Left, Modia3D.Right or Modia3D.Center).

# Examples
```julia
import Modia3D
font  = Modia3D.Font(fontFamily="Arial", charSize=0.4, color=Modia3D.rgb("Red"))
text1 = Modia3D.TextShape("This is a box")
text2 = Modia3D.TextShape("This is the xy plane";
                          font=font, axisAlignment=Modia3D.XY_Plane, 
                          alignment=Modia3D.Left)
```
"""
mutable struct TextShape <: Modia3D.AbstractVisualElement
   text::String                   # String to be displayed
   font::Font                     # Font
   offset::MVector{3,Float64}     # Offset from origin to text alignment point
   axisAlignment::AxisAlignment   # Alignment of Text (parallel to screen or in planes of frame)
   alignment::Alignment           # Alignment of Text relative to its origin

   function TextShape(text::AbstractString;
                      font::Font=Font(),
                      offset::AbstractVector = Basics.zeroMVector(),
                      axisAlignment::AxisAlignment=Screen,
                      alignment::Alignment=Center)

      new(text,font,offset,axisAlignment,alignment)
   end
end
