# License for this file: MIT (expat)
# Copyright 2017-2018, DLR Institute of System Dynamics and Control
#
# This file is part of module
#   Modia3D.DLR_Visualization (Modia3D/renderer/DLR_Visualization/_module.jl)
#

function Composition.initializeVisualization(renderer::Modia3D.AbstractDLR_VisualizationRenderer, scene::Composition.Scene{F})::Nothing where F <: Modia3D.VarFloatType
    simVis::SimVis_Renderer = renderer.simVis
    velements = scene.allVisuElements
    @assert(length(velements) > 0)
    if simVis.isInitialized
        Composition.closeVisualization(renderer)
    end

    # Initialize SimVis
    SimVis_init(simVis)

    # Determine visualize functions and ids of visual elements
    for i in eachindex(velements)
        push!(simVis.ids, SimVis_getObjectID(simVis, 0))
    end
    simVis.velements = copy(velements)
    simVis.isInitialized = true
    return nothing
end


function Composition.visualize!(renderer::Modia3D.AbstractDLR_VisualizationRenderer, time)
    simVis::SimVis_Renderer = renderer.simVis
    @assert(length(simVis.velements) > 0)
    if simVis.isInitialized
        velements = simVis.velements
        ids       = simVis.ids
        for i in eachindex(velements)
            visualizeObject(velements[i], ids[i], simVis)
        end
        SimVis_setTime(simVis, Float64(time) )
    else
        error("visualize! called without initializing visualization first.")
    end
    return nothing
end


function Composition.closeVisualization(renderer::Modia3D.AbstractDLR_VisualizationRenderer)
    simVis::SimVis_Renderer = renderer.simVis
    if simVis.isInitialized
        sleep(0.01)
        for id in simVis.ids
            SimVis_freeObjectID(simVis, id)
        end
        SimVis_shutdown(simVis)
    end
    empty!(simVis.ids)
    empty!(simVis.velements)
    simVis.isInitialized = false
    return nothing
end


function Composition.isVisible(feature::Shapes.Solid{F}, renderer::Modia3D.AbstractDLR_VisualizationRenderer) where F <: Modia3D.VarFloatType
    return !isnothing(feature.visualMaterial) && !isnothing(feature.shape)
end

function Composition.isVisible(feature::Shapes.Visual, renderer::Modia3D.AbstractDLR_VisualizationRenderer)
    if isnothing(feature.shape)
        return false
    elseif typeof(feature.shape) == Shapes.TextShape
        return typeof(renderer) != CommunityEdition
    else
        return !isnothing(feature.visualMaterial)
    end
end
