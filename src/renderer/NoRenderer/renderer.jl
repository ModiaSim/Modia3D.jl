#
# This file is part of module 
#   Modia3D.Composition (Modia3D/Composition/_module.jl)
#
# It is included in file Modia3D/Composition/sceneOptions.jl
# in order to be used as default renderer in SceneOptions(..)
#


mutable struct DummyRenderer <: Modia3D.AbstractRenderer
  DummyRenderer(info; host="127.0.0.1",port=11000,sync=false) = new()
end

