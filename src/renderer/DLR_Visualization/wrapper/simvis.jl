# License for this file: MIT (expat)
# Copyright 2017-2018, DLR Institute of System Dynamics and Control
#
# This file is part of module
#   Modia3D.DLR_Visualization (Modia3D/renderer/DLR_Visualization/_module.jl)
#



### Functions available for community and professional edition of SimVis

function SimVis_init(simVis::SimVis_Renderer)
   ccall(simVis.init, Nothing,(Cstring,Cstring,Cint,Cint),
                               simVis.directory, simVis.host, simVis.port, Int(simVis.sync))
end

function SimVis_shutdown(simVis::SimVis_Renderer)
   ccall(simVis.shutdown, Nothing,())
end

function SimVis_getObjectID(simVis::SimVis_Renderer, emptyObjectID::Int)
   ccall(simVis.getObjectID, Ptr{Nothing},(Cint,), emptyObjectID)
end

function SimVis_freeObjectID(simVis::SimVis_Renderer, obj::Ptr{Nothing})
   ccall(simVis.freeObjectID, Nothing,(Ptr{Nothing},), obj)
end

function SimVis_setTime(simVis::SimVis_Renderer, time)
   ccall(simVis.setTime, Nothing,(Cdouble,), Float64(time) )
end

function SimVis_setBaseObject(simVis::SimVis_Renderer,
                              ID::Ptr{Nothing},
                              state::Cint,
                              baseObjType::Cint,
                              pos::MVector{3,Float64},
                              T::MMatrix{3,3,Float64,9},
                              scale::MVector{3,Float64},
                              color::MVector{3,Cint},
                              wireframe::Cint,
                              reflectslight::Cint,
                              specularCoefficient::Float64,
                              extra::MVector{3,Float64},
                              alpha::Float64,
                              canCollide::Cint,
                              shadowMask::Cint)
   ccall(simVis.setBaseObject, Nothing,
           (Ptr{Nothing},Cint,Cint,Ref{SVector{3,Float64}},Ref{SMatrix{3,3,Float64,9}},Ref{MVector{3,Float64}},Ref{MVector{3,Cint}},Cint,Cint,Cdouble,Ref{MVector{3,Float64}},Cdouble,Cint,Cint),
           ID,state,baseObjType,pos,T,scale,color,wireframe,reflectslight,specularCoefficient,extra,alpha,canCollide,shadowMask)
end

function SimVis_setBaseObject(simVis::SimVis_Renderer,
                              ID::Ptr{Nothing},
                              state::Cint,
                              baseObjType::Cint,
                              pos::SVector{3,Float64},
                              T::SMatrix{3,3,Float64,9},
                              scale::MVector{3,Float64},
                              color::MVector{3,Cint},
                              wireframe::Cint,
                              reflectslight::Cint,
                              specularCoefficient::Float64,
                              extra::MVector{3,Float64},
                              alpha::Float64,
                              canCollide::Cint,
                              shadowMask::Cint)
   ccall(simVis.setBaseObject, Nothing,
           (Ptr{Nothing},Cint,Cint,Ref{SVector{3,Float64}},Ref{SMatrix{3,3,Float64,9}},Ref{MVector{3,Float64}},Ref{MVector{3,Cint}},Cint,Cint,Cdouble,Ref{MVector{3,Float64}},Cdouble,Cint,Cint),
           ID,state,baseObjType,pos,T,scale,color,wireframe,reflectslight,specularCoefficient,extra,alpha,canCollide,shadowMask)
end


function SimVis_setFileObject(simVis::SimVis_Renderer,
                              ID::Ptr{Nothing},
                              state::Cint,
                              pos::MVector{3,Float64},
                              T::MMatrix{3,3,Float64,9},
                              scale::MVector{3,Float64},
                              reflectslight::Cint,
                              specularCoefficient::Float64,
                              alpha::Float64,
                              wireframe::Cint,
                              canCollide::Cint,
                              filename::String,
                              smooth::Cint,
                              overwriteColor::Bool,
                              color::MVector{3,Cint},
                              shadowMask::Cint,
                              shaderName::String)
   ccall(simVis.setFileObject, Nothing,
            (Ptr{Nothing},Cint,Ref{MVector{3,Float64}},Ref{MVector{3,Float64}},Ref{MVector{3,Float64}},Cint,Cdouble,Cdouble,Cint,Cint,Cstring,Cint,Cint,Ref{MVector{3,Cint}},Cint,Cstring),
             ID,state,pos,T,scale,reflectslight,specularCoefficient,alpha,wireframe,canCollide,filename,smooth,overwriteColor,color,shadowMask,shaderName)
end



function SimVis_setFileObject(simVis::SimVis_Renderer,
                              ID::Ptr{Nothing},
                              state::Cint,
                              pos::SVector{3,Float64},
                              T::SMatrix{3,3,Float64,9},
                              scale::MVector{3,Float64},
                              reflectslight::Cint,
                              specularCoefficient::Float64,
                              alpha::Float64,
                              wireframe::Cint,
                              canCollide::Cint,
                              filename::String,
                              smooth::Cint,
                              overwriteColor::Bool,
                              color::MVector{3,Cint},
                              shadowMask::Cint,
                              shaderName::String)
   ccall(simVis.setFileObject, Nothing,
            (Ptr{Nothing},Cint,Ref{SVector{3,Cdouble}},Ref{SMatrix{3,3,Cdouble,9}},Ref{MVector{3,Float64}},Cint,Cdouble,Cdouble,Cint,Cint,Cstring,Cint,Cint,Ref{MVector{3,Cint}},Cint,Cstring),
             ID,state,pos,T, scale,reflectslight,specularCoefficient,alpha,wireframe,canCollide,filename,smooth,overwriteColor,color,shadowMask,shaderName)
end



### Functions only available for professional edition of SimVis
function SimVis_setTextObject(simVis::SimVis_Renderer,
                              ID::Ptr{Nothing},
                              screenAlignment::Cint,
                              text::String,
                              textvalue::Float64,
                              valueactive::Cint,
                              pos::MVector{3,Float64},
                              T::MMatrix{3,3,Float64,9},
                              charsize::Float64,
                              fontname::String,
                              color::MVector{3,Cint},
                              alpha::Float64,
                              offset::MVector{3,Float64},
                              alignment::Cint,
                              digits::Cint)
   ccall(simVis.setTextObject, Nothing,
           (Ptr{Nothing},Cint,Cstring,Cdouble,Cint,Ref{MVector{3,Float64}},Ref{MMatrix{3,3,Float64,9}},Cdouble,
            Cstring,Ref{MVector{3,Cint}},Cdouble,Ref{MVector{3,Float64}},Cint,Cint),
           ID, screenAlignment, text, textvalue, valueactive, pos, T, charsize,
           fontname, color, alpha, offset, alignment, digits)
end


function SimVis_setTextObject(simVis::SimVis_Renderer,
                              ID::Ptr{Nothing},
                              screenAlignment::Cint,
                              text::String,
                              textvalue::Float64,
                              valueactive::Cint,
                              pos::SVector{3,Float64},
                              T::SMatrix{3,3,Float64,9},
                              charsize::Float64,
                              fontname::String,
                              color::MVector{3,Cint},
                              alpha::Float64,
                              offset::MVector{3,Float64},
                              alignment::Cint,
                              digits::Cint)
   ccall(simVis.setTextObject, Nothing,
           (Ptr{Nothing},Cint,Cstring,Cdouble,Cint,Ref{SVector{3,Cdouble}},Ref{SMatrix{3,3,Cdouble,9}},Cdouble,
            Cstring,Ref{MVector{3,Cint}},Cdouble,Ref{MVector{3,Float64}},Cint,Cint),
           ID, screenAlignment, text, textvalue, valueactive, pos, T, charsize,
           fontname, color, alpha, offset, alignment, digits)
end
