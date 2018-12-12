# License for this file: MIT (expat)
# Copyright 2017-2018, DLR Institute of System Dynamics and Control
#
# This file is part of module
#   Modia3D.DLR_Visualization (Modia3D/renderer/DLR_Visualization/_module.jl)
#

getVisualElement(data)               = data
getVisualElement(data::Solids.Solid) = data.geo


# Get function to visualize AbstractObject3Ddata object:
# - element  = getVisualElement(data): Get element from AbstractObject3Ddata that shall be visualized
# - name     = Basics.trailingPartOfTypeAsString( element ): Get trailing solid of typeof(element) as String (e.g. "Sphere")
# - symbol   = Symbol("visualize", name): Construct function name as symbol (e.g. :visualizeSphere)
# - function = getfield(DLR_Visualization, symbol): Get a Function pointer to symbol stored in module DLR_Visualization
#
# An alternative and even more efficient method would be to determine the C-function pointer with "cfunction(..)"
# and then call this function with "ccall(..)". However, if something goes wrong, Julia will crash.
# Therefore, this solution is not (yet) used.
getVisualizeFunction(data) = getfield(DLR_Visualization, Symbol("visualize", Basics.trailingPartOfTypeAsString( getVisualElement(data) )))


function Composition.initializeVisualization(renderer::Modia3D.AbstractDLR_VisualizationRenderer, velements::Vector{Composition.Object3D})::NOTHING
  simVis = renderer.simVis
  @assert(length(velements) > 0)
  if simVis.isInitialized
     Composition.closeVisualization(renderer)
  end

  # Initialize SimVis
  SimVis_init(simVis)

  # Determine visualize functions and ids of visual elements
  for i in eachindex(velements)
    push!(simVis.visualize, getVisualizeFunction(velements[i].data))
    push!(simVis.ids      , SimVis_getObjectID(simVis,0))
  end
  simVis.velements = velements
  simVis.isInitialized = true
  return nothing
end


function Composition.visualize!(renderer::Modia3D.AbstractDLR_VisualizationRenderer, time::Float64)
  simVis = renderer.simVis
  @assert(length(simVis.velements) > 0)
  if simVis.isInitialized
     velements = simVis.velements
     visualize = simVis.visualize
     ids       = simVis.ids
     for i in eachindex(velements)
        visualize[i](velements[i].data, velements[i], ids[i], simVis)
     end
     SimVis_setTime(simVis, time)
  else
     error("visualize! called without initializing visualization first.")
  end
end


function Composition.closeVisualization(renderer::Modia3D.AbstractDLR_VisualizationRenderer)
  simVis = renderer.simVis
  if simVis.isInitialized
    sleep(0.2)
    for id in simVis.ids
      SimVis_freeObjectID(simVis, id)
    end
    SimVis_shutdown(simVis)
  end
  empty!(simVis.ids)
  empty!(simVis.visualize)
  empty!(simVis.velements)
  simVis.isInitialized = false
end


Composition.isVisible(data::Modia3D.AbstractVisualElement, renderer::ProfessionalEdition) = true
Composition.isVisible(data::Solids.Solid                 , renderer::ProfessionalEdition) = typeof(data.material) != NOTHING && typeof(data.geo) != NOTHING

Composition.isVisible(data::Graphics.Spring              , renderer::CommunityEdition) = true
Composition.isVisible(data::Graphics.GearWheel           , renderer::CommunityEdition) = true
Composition.isVisible(data::Graphics.CoordinateSystem    , renderer::CommunityEdition) = true
Composition.isVisible(data::Modia3D.AbstractGeometry     , renderer::CommunityEdition) = true
Composition.isVisible(data::Solids.Solid                 , renderer::CommunityEdition) = typeof(data.material) != NOTHING && typeof(data.geo) != NOTHING
