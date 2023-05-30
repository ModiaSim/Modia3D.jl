# License for this file: MIT (expat)
# Copyright 2017-2018, DLR Institute of System Dynamics and Control
#
# This file is part of module
#   Modia3D.DLR_Visualization (Modia3D/renderer/DLR_Visualization/_module.jl)
#

function Composition.initializeVisualization(renderer::DummyRenderer, scene::Composition.Scene{F})::Nothing where F <: Modia3D.VarFloatType
end


function Composition.visualize!(renderer::DummyRenderer, time)
end


function Composition.closeVisualization(renderer::DummyRenderer)
end


Composition.isVisible(feature::Modia3D.AbstractVisualElement, renderer::DummyRenderer) = false
