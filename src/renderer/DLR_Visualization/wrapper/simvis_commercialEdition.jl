#
# This file is part of module
#   Modia3D.DLR_Visualization (Modia3D/renderer/DLR_Visualization/_module.jl)
#
# and is included from file
#   Modia3D/renderer/DLR_Visualization/wrapper/simvis.jl
#
# This file contains functionality specific to the commercial edition of SimVis 

mutable struct SimVisFunctions
  # Handle for SimVis DLL
  dll_handle::Ptr{NOTHING}

  # SimVis functions
  init::Ptr{NOTHING}
  shutdown::Ptr{NOTHING}
  getObjectID::Ptr{NOTHING}
  freeObjectID::Ptr{NOTHING}
  setTime::Ptr{NOTHING}
  setBaseObject::Ptr{NOTHING}
  setFileObject::Ptr{NOTHING}
  setTextObject::Ptr{NOTHING}

  SimVisFunctions() = new()
end

const simVisFunctions = SimVisFunctions()

function SimVis_init(SimVisHost::String, SimVisPort::Int, sync::Int)
   simVisFunctions.dll_handle    = Libdl.dlopen(simVisInfo.dll_name)
   dll_handle                    = simVisFunctions.dll_handle
   simVisFunctions.init          = Libdl.dlsym(dll_handle, :SimVis_init)
   simVisFunctions.shutdown      = Libdl.dlsym(dll_handle, :SimVis_shutdown)
   simVisFunctions.getObjectID   = Libdl.dlsym(dll_handle, :SimVis_getObjectID)
   simVisFunctions.freeObjectID  = Libdl.dlsym(dll_handle, :SimVis_freeObjectID) 
   simVisFunctions.setTime       = Libdl.dlsym(dll_handle, :SimVis_setTime)
   simVisFunctions.setBaseObject = Libdl.dlsym(dll_handle, :SimVis_setBaseObject)    
   simVisFunctions.setFileObject = Libdl.dlsym(dll_handle, :SimVis_setFileObject)
   simVisFunctions.setTextObject = Libdl.dlsym(dll_handle, :SimVis_setTextObject)
   ccall(simVisFunctions.init, NOTHING,(Cstring,Cstring,Cint,Cint),
                                     simVisInfo.directory,SimVisHost,SimVisPort,sync)
end


function SimVis_setTextObject(ID::Ptr{NOTHING},
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
   ccall(simVisFunctions.setTextObject, NOTHING,
           (Ptr{NOTHING},Cint,Cstring,Cdouble,Cint,Ref{MVector{3,Float64}},Ref{MMatrix{3,3,Float64,9}},Cdouble,
            Cstring,Ref{MVector{3,Cint}},Cdouble,Ref{MVector{3,Float64}},Cint,Cint),
           ID, screenAlignment, text, textvalue, valueactive, pos, T, charsize,
           fontname, color, alpha, offset, alignment, digits)
end


function SimVis_setTextObject(ID::Ptr{NOTHING},
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
   ccall(simVisFunctions.setTextObject, NOTHING,
           (Ptr{NOTHING},Cint,Cstring,Cdouble,Cint,Ref{SVector{3,Cdouble}},Ref{SMatrix{3,3,Cdouble,9}},Cdouble,
            Cstring,Ref{MVector{3,Cint}},Cdouble,Ref{MVector{3,Float64}},Cint,Cint),
           ID, screenAlignment, text, textvalue, valueactive, pos, T, charsize,
           fontname, color, alpha, offset, alignment, digits)
end


Composition.isVisible(data::Modia3D.AbstractVisualElement, renderer::Composition.DLR_Visualization_renderer) = true
Composition.isVisible(data::Solids.Solid, renderer::Composition.DLR_Visualization_renderer) = typeof(data.material) != NOTHING && typeof(data.geo) != NOTHING

