#
# This file is part of module 
#   Modia3D.Composition (Modia3D/Composition/_module.jl)
#
# It is included in file Modia3D/Composition/sceneOptions.jl
# in order to be used as default renderer in SceneOptions(..)
#


mutable struct NoRenderer <: Modia3D.AbstractRenderer

  NoRenderer(; host="127.0.0.1",port=11000,sync=false) = new()
end

Composition.isVisible(data::Modia3D.AbstractVisualElement, renderer::Composition.NoRenderer) = false
Composition.isVisible(data::Solids.Solid, renderer::Composition.NoRenderer) = false
