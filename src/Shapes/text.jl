# License for this file: MIT (expat)
# Copyright 2017-2018, DLR Institute of System Dynamics and Control
#
# This file is part of module
#   Modia3D.Shapes (Modia3D/Shapes/_module.jl)
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
    Font(; fontFamily="FreeSans", bold=false, italic=false,
           charSize=0.1, color=defaultColor(), transparency=0.0)

Generate a Font.

# Arguments
- `fontFamily`: name of the font family
    - "FreeSans", "FreeSerif", "Arial", "ArialNarrow", "CourierNew", "TimesNewRoman", or "Verdana"
![fontFamily](../../resources/images/TextFonts.png)
Courtesy to DLR Visualization Library
- `bold`: if true, the text is displayed in bold font.
- `italic`: if true, the text is displayed in italic font.
- `charSize`: character size in [m].
- `color` defines the material color as a String or RGB values. In case of Strings the color
          is constructed using [Colors.jl](https://github.com/JuliaGraphics/Colors.jl). So
          [named colors](https://juliagraphics.github.io/Colors.jl/stable/namedcolors/) and
          many other kinds of [color specifications](https://juliagraphics.github.io/Colors.jl/stable/constructionandconversion/#Color-Parsing)
          are available. In addition, colors can be directly defined by a vector of RGB values.
          E.g. `color="MediumVioletRed"` or `color=[255, 0, 0]`.
- `transparency`: transparency of the fon , 0.0 (opaque) ... 1.0 (transparent)

# Notes
- `fontFamily` like "Arial", "Verdana" are only supported under Windows.

# Examples
```julia
import Modia3D
font1 = Font()
font2 = Font(fontFamily="Arial", bold=true, charSize=0.2,
             color="LightBlue", transparency=0.5)
```
"""
struct Font
    fontFamily::String
    bold::Bool
    italic::Bool
    charSize::Float64
    color::MVector{3,Cint}
    transparency::Float64

    # Constructed by constructor
    fontFileName::String  # File name of font

    function Font(; fontFamily::String = "FreeSans",
        bold::Bool            = false,
        italic::Bool          = false,
        charSize::Number      = 0.1,
        color                 = defaultColor() ,
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
    TextShape(; text="Hello world", font=Font(), offset=[0.0,0.0,0.0],
                axisAlignment=Screen, alignment=Center)

Generate a new visual shape representing a text shape.

# Arguments
- `text::AbstractString`: String of the text
- `font::`[`Font`](@ref): [`Font`](@ref) of the text
- `offset`: a 3D vector as offset from origin to text alignment point
- `axisAlignment::Modia3D.AxisAlignment`: alignment of text
   - `Modia3D.Screen`: parallel to screen
   - `Modia3D.XY_Plane`: in xy-planes of Object3D
   - `Modia3D.XZ_Plane`: in xz-planes of Object3D
   - `Modia3D.YZ_Plane`: in yz-planes of Object3D
- `axisAlignment::Modia3D.Alignment`: defines the direction the text is displayed, relative to its origin
  - `Modia3D.Left`, `Modia3D.Right` or `Modia3D.Center`

# Notes
- TextShape is not supported by the community edition of SimVis.
- TextShape is [not supported by animation export](https://github.com/ModiaSim/PrivateModia3D.jl/issues/77).

# Examples
```julia
import Modia3D
font  = Font(fontFamily="Arial", charSize=0.4, color="MediumVioletRed")
text1 = TextShape("This is a box")
text2 = TextShape("This is the xy plane";
                   font = font, axisAlignment=Modia3D.XY_Plane,
                   alignment = Modia3D.Left)
```
"""
mutable struct TextShape <: Modia3D.AbstractShape
    text::String                   # String to be displayed
    font::Font                     # Font
    offset::MVector{3,Float64}     # Offset from origin to text alignment point
    axisAlignment::AxisAlignment   # Alignment of Text (parallel to screen or in planes of frame)
    alignment::Alignment           # Alignment of Text relative to its origin

    function TextShape(; text::AbstractString,
        font::Font=Font(),
        offset::AbstractVector = @MVector[0.0, 0.0, 0.0],
        axisAlignment::AxisAlignment=Screen,
        alignment::Alignment=Center)

        new(text,font,offset,axisAlignment,alignment)
    end
end
