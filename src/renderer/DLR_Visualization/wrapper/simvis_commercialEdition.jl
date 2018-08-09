#
# This file is part of module
#   Modia3D.DLR_Visualization (Modia3D/renderer/DLR_Visualization/_module.jl)
#
# and is included from file
#   Modia3D/renderer/DLR_Visualization/wrapper/simvis.jl
#
# This file contains functionality specific to the commercial edition of SimVis 

mutable struct SimVisFunctions
  # SimVis DLL
  dll::Ptr{Void}

  # SimVis functions
  init::Ptr{Void}
  shutdown::Ptr{Void}
  getObjectID::Ptr{Void}
  freeObjectID::Ptr{Void}
  setTime::Ptr{Void}
  setBaseObject::Ptr{Void}
  setFileObject::Ptr{Void}
  setTextObject::Ptr{Void}

  SimVisFunctions() = new(Base.Libdl.dlopen(simVisInfo.dll_name))
end

const simVisFunctions = SimVisFunctions()

function SimVis_init(SimVisHost::String, SimVisPort::Int, sync::Int)
   #simVisFunctions.dll           = Base.Libdl.dlopen(simVisInfo.dll_name)
   simVisFunctions.init          = Base.Libdl.dlsym(simVisFunctions.dll, :SimVis_init)
   simVisFunctions.shutdown      = Base.Libdl.dlsym(simVisFunctions.dll, :SimVis_shutdown)
   simVisFunctions.getObjectID   = Base.Libdl.dlsym(simVisFunctions.dll, :SimVis_getObjectID)
   simVisFunctions.freeObjectID  = Base.Libdl.dlsym(simVisFunctions.dll, :SimVis_freeObjectID) 
   simVisFunctions.setTime       = Base.Libdl.dlsym(simVisFunctions.dll, :SimVis_setTime)
   simVisFunctions.setBaseObject = Base.Libdl.dlsym(simVisFunctions.dll, :SimVis_setBaseObject)    
   simVisFunctions.setFileObject = Base.Libdl.dlsym(simVisFunctions.dll, :SimVis_setFileObject)
   simVisFunctions.setTextObject = Base.Libdl.dlsym(simVisFunctions.dll, :SimVis_setTextObject)

   ccall(simVisFunctions.init, Void,(Cstring,Cstring,Cint,Cint),
                                     simVisInfo.directory,SimVisHost,SimVisPort,sync)
end


function SimVis_setTextObject(ID::Ptr{Void},
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
   ccall(simVisFunctions.setTextObject, Void,
           (Ptr{Void},Cint,Cstring,Cdouble,Cint,Ref{MVector{3,Float64}},Ref{MMatrix{3,3,Float64,9}},Cdouble,
            Cstring,Ref{MVector{3,Cint}},Cdouble,Ref{MVector{3,Float64}},Cint,Cint),
           ID, screenAlignment, text, textvalue, valueactive, pos, T, charsize,
           fontname, color, alpha, offset, alignment, digits)
end


function SimVis_setTextObject(ID::Ptr{Void},
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
   ccall(simVisFunctions.setTextObject, Void,
           (Ptr{Void},Cint,Cstring,Cdouble,Cint,Ref{SVector{3,Cdouble}},Ref{SMatrix{3,3,Cdouble,9}},Cdouble,
            Cstring,Ref{MVector{3,Cint}},Cdouble,Ref{MVector{3,Float64}},Cint,Cint),
           ID, screenAlignment, text, textvalue, valueactive, pos, T, charsize,
           fontname, color, alpha, offset, alignment, digits)
end


Composition.isVisible(data::Modia3D.AbstractVisualElement, renderer::Composition.DLR_Visualization_renderer) = true
Composition.isVisible(data::Solids.Solid, renderer::Composition.DLR_Visualization_renderer) = typeof(data.material) != Void && typeof(data.geo) != Void

