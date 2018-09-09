#
# This file is part of module
#   Modia3D.DLR_Visualization (Modia3D/renderer/DLR_Visualization/_module.jl)
#
# and is included from file
#   Modia3D/renderer/DLR_Visualization/wrapper/simvis.jl
#
# This file contains functionality specific to the community edition of SimVis 

mutable struct SimVisFunctions
  # SimVis DLL
  dll::Ptr{NOTHING}

  # SimVis functions
  init::Ptr{NOTHING}
  shutdown::Ptr{NOTHING}
  getObjectID::Ptr{NOTHING}
  freeObjectID::Ptr{NOTHING}
  setTime::Ptr{NOTHING}
  setBaseObject::Ptr{NOTHING}
  setFileObject::Ptr{NOTHING}

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

   ccall(simVisFunctions.init, NOTHING,(Cstring,Cstring,Cint,Cint),
                                     simVisInfo.directory,SimVisHost,SimVisPort,sync)
end


Composition.isVisible(data::Graphics.Spring          , renderer::Composition.DLR_Visualization_renderer) = true
Composition.isVisible(data::Graphics.GearWheel       , renderer::Composition.DLR_Visualization_renderer) = true
Composition.isVisible(data::Graphics.CoordinateSystem, renderer::Composition.DLR_Visualization_renderer) = true
Composition.isVisible(data::Modia3D.AbstractGeometry , renderer::Composition.DLR_Visualization_renderer) = true
Composition.isVisible(data::Solids.Solid             , renderer::Composition.DLR_Visualization_renderer) = typeof(data.material) != NOTHING && typeof(data.geo) != NOTHING
