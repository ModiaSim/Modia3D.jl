# License for this file: MIT (expat)
# Copyright 2017-2018, DLR Institute of System Dynamics and Control
#
# This file is part of module
#   Modia3D.DLR_Visualization (Modia3D/renderer/DLR_Visualization/_module.jl)
#

function Composition.initializeVisualization(renderer::DummyRenderer, velements::Vector{Composition.Object3D})::NOTHING
end


function Composition.visualize!(renderer::DummyRenderer, time::Float64)
end


function Composition.closeVisualization(renderer::DummyRenderer)
end


Composition.isVisible(data::Modia3D.AbstractVisualElement, renderer::DummyRenderer) = false
Composition.isVisible(data::Solids.Solid                 , renderer::DummyRenderer) = false

