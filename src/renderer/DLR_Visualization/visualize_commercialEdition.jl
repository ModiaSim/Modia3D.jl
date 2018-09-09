# License for this file: MIT (expat)
# Copyright 2017-2018, DLR Institute of System Dynamics and Control
#
# This file is part of module
#   Modia3D.DLR_Visualization (Modia3D/renderer/DLR_Visualization/_module.jl)
#


visualizeTextShape(data::Graphics.TextShape, frame::Composition.Object3D, id::Ptr{NOTHING}) =
    SimVis_setTextObject(id, Cint(data.axisAlignment), data.text, 0.0, Cint(0), frame.r_abs, frame.R_abs,
                         data.font.charSize, data.font.fontFileName, data.font.color, data.font.transparency, 
                         data.offset, Cint(data.alignment), Cint(0)) 
