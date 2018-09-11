#
# This file is part of module
#   Modia3D.DLR_Visualization (Modia3D/renderer/DLR_Visualization/_module.jl)
#


# Function available for community and commercial version of SimVis 
function SimVis_shutdown()
   ccall(simVisFunctions.shutdown, NOTHING,())

   # If dlclose is included, there are many warning messages in runtests:
   #     [SimVis Client Error] Connection to SimVis was interupted!
   # Libdl.dlclose(simVisFunctions.dll_handle)
end

function SimVis_getObjectID(emptyObjectID::Int)
   ccall(simVisFunctions.getObjectID, Ptr{NOTHING},(Cint,), emptyObjectID)
end

function SimVis_freeObjectID(obj::Ptr{NOTHING})
   ccall(simVisFunctions.freeObjectID, NOTHING,(Ptr{NOTHING},), obj)
end

function SimVis_setTime(time::Float64)
   ccall(simVisFunctions.setTime, NOTHING,(Cdouble,), time)
end

function SimVis_setBaseObject(ID::Ptr{NOTHING},
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
   ccall(simVisFunctions.setBaseObject, NOTHING,
           (Ptr{NOTHING},Cint,Cint,Ref{SVector{3,Float64}},Ref{SMatrix{3,3,Float64,9}},Ref{MVector{3,Float64}},Ref{MVector{3,Cint}},Cint,Cint,Cdouble,Ref{MVector{3,Float64}},Cdouble,Cint,Cint),
           ID,state,baseObjType,pos,T,scale,color,wireframe,reflectslight,specularCoefficient,extra,alpha,canCollide,shadowMask)
end

function SimVis_setBaseObject(ID::Ptr{NOTHING},
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
   ccall(simVisFunctions.setBaseObject, NOTHING,
           (Ptr{NOTHING},Cint,Cint,Ref{SVector{3,Float64}},Ref{SMatrix{3,3,Float64,9}},Ref{MVector{3,Float64}},Ref{MVector{3,Cint}},Cint,Cint,Cdouble,Ref{MVector{3,Float64}},Cdouble,Cint,Cint),
           ID,state,baseObjType,pos,T,scale,color,wireframe,reflectslight,specularCoefficient,extra,alpha,canCollide,shadowMask)
end


function SimVis_setFileObject(ID::Ptr{NOTHING},
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
   ccall(simVisFunctions.setFileObject, NOTHING,
            (Ptr{NOTHING},Cint,Ref{MVector{3,Float64}},Ref{MVector{3,Float64}},Ref{MVector{3,Float64}},Cint,Cdouble,Cdouble,Cint,Cint,Cstring,Cint,Cint,Ref{MVector{3,Cint}},Cint,Cstring),
             ID,state,pos,T,scale,reflectslight,specularCoefficient,alpha,wireframe,canCollide,filename,smooth,overwriteColor,color,shadowMask,shaderName)
end



function SimVis_setFileObject(ID::Ptr{NOTHING},
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
   ccall(simVisFunctions.setFileObject, NOTHING,
            (Ptr{NOTHING},Cint,Ref{SVector{3,Cdouble}},Ref{SMatrix{3,3,Cdouble,9}},Ref{MVector{3,Float64}},Cint,Cdouble,Cdouble,Cint,Cint,Cstring,Cint,Cint,Ref{MVector{3,Cint}},Cint,Cstring),
             ID,state,pos,T, scale,reflectslight,specularCoefficient,alpha,wireframe,canCollide,filename,smooth,overwriteColor,color,shadowMask,shaderName)
end
