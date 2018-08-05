#
# This file is part of module
#   Modia3D.DLR_Visualization (Modia3D/renderer/DLR_Visualization/_module.jl)
#


# Function available for community and commercial version of SimVis 
function SimVis_shutdown()
   ccall(simVisFunctions.shutdown, Void,())
end

function SimVis_getObjectID(emptyObjectID::Int)
   ccall(simVisFunctions.getObjectID, Ptr{Void},(Cint,), emptyObjectID)
end

function SimVis_freeObjectID(obj::Ptr{Void})
   ccall(simVisFunctions.freeObjectID, Void,(Ptr{Void},), obj)
end

function SimVis_setTime(time::Float64)
   ccall(simVisFunctions.setTime, Void,(Cdouble,), time)
end

function SimVis_setBaseObject(ID::Ptr{Void},
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
   ccall(simVisFunctions.setBaseObject, Void,
           (Ptr{Void},Cint,Cint,Ref{Cdouble},Ref{Cdouble},Ref{Cdouble},Ref{Cint},Cint,Cint,Cdouble,Ref{Cdouble},Cdouble,Cint,Cint),
           ID,state,baseObjType,pos,T,scale,color,wireframe,reflectslight,specularCoefficient,extra,alpha,canCollide,shadowMask)
end

function SimVis_setBaseObject(ID::Ptr{Void},
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
   ccall(simVisFunctions.setBaseObject, Void,
           (Ptr{Void},Cint,Cint,SVector{3,Float64},SMatrix{3,3,Float64,9},Ptr{Cdouble},Ptr{Cint},Cint,Cint,Cdouble,Ptr{Cdouble},Cdouble,Cint,Cint),
           ID,state,baseObjType,pos,T,scale,color,wireframe,reflectslight,specularCoefficient,extra,alpha,canCollide,shadowMask)
end


function SimVis_setFileObject(ID::Ptr{Void},
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
   ccall(simVisFunctions.setFileObject, Void,
            (Ptr{Void},Cint,Ref{Cdouble},Ref{Cdouble},Ref{Cdouble},Cint,Cdouble,Cdouble,Cint,Cint,Cstring,Cint,Cint,Ref{Cint},Cint,Cstring),
             ID,state,pos,T,scale,reflectslight,specularCoefficient,alpha,wireframe,canCollide,filename,smooth,overwriteColor,color,shadowMask,shaderName)
end



function SimVis_setFileObject(ID::Ptr{Void},
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
   ccall(simVisFunctions.setFileObject, Void,
            (Ptr{Void},Cint,SVector{3,Cdouble},SMatrix{3,3,Cdouble,9},Ref{Cdouble},Cint,Cdouble,Cdouble,Cint,Cint,Cstring,Cint,Cint,Ref{Cint},Cint,Cstring),
             ID,state,pos,T, scale,reflectslight,specularCoefficient,alpha,wireframe,canCollide,filename,smooth,overwriteColor,color,shadowMask,shaderName)
end
