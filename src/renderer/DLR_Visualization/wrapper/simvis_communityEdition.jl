#
# This file is part of module
#   Modia3D.DLR_Visualization (Modia3D/renderer/DLR_Visualization/_module.jl)
#
# and is included from file
#   Modia3D/renderer/DLR_Visualization/wrapper/simvis.jl
#
# This file contains functionality specific to the community edition of SimVis 

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

   ccall(simVisFunctions.init, NOTHING,(Cstring,Cstring,Cint,Cint),
                                     simVisInfo.directory,SimVisHost,SimVisPort,sync)
end



Composition.isVisible(data::Graphics.Spring          , renderer::Composition.DLR_Visualization_renderer) = true
Composition.isVisible(data::Graphics.GearWheel       , renderer::Composition.DLR_Visualization_renderer) = true
Composition.isVisible(data::Graphics.CoordinateSystem, renderer::Composition.DLR_Visualization_renderer) = true
Composition.isVisible(data::Modia3D.AbstractGeometry , renderer::Composition.DLR_Visualization_renderer) = true
Composition.isVisible(data::Solids.Solid             , renderer::Composition.DLR_Visualization_renderer) = typeof(data.material) != NOTHING && typeof(data.geo) != NOTHING
