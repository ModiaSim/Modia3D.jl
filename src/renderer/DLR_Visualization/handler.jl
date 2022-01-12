# License for this file: MIT (expat)
# Copyright 2017-2018, DLR Institute of System Dynamics and Control
#
# This file is part of module
#   Modia3D.DLR_Visualization (Modia3D/renderer/DLR_Visualization/_module.jl)
#

function Composition.initializeVisualization(renderer::Modia3D.AbstractDLR_VisualizationRenderer, velements::Vector{Composition.Object3D{F}})::Nothing where F <: AbstractFloat
    simVis = renderer.simVis
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
    simVis = renderer.simVis
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
end


function Composition.closeVisualization(renderer::Modia3D.AbstractDLR_VisualizationRenderer)
    simVis = renderer.simVis
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
end


function Composition.isVisible(feature::Shapes.Solid, renderer::Modia3D.AbstractRenderer)
    return !isnothing(feature.visualMaterial) && !isnothing(feature.shape)
end

function Composition.isVisible(feature::Shapes.Visual, renderer::Modia3D.AbstractRenderer)
    if isnothing(feature.shape)
        return false
    elseif typeof(feature.shape) == Shapes.TextShape
        return typeof(renderer) == ProfessionalEdition
    else
        return !isnothing(feature.visualMaterial)
    end
end
