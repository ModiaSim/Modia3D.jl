# License for this file: MIT (expat)
# Copyright 2017-2018, DLR Institute of System Dynamics and Control
#
# This file is part of module
#   Modia3D.DLR_Visualization (Modia3D/renderer/DLR_Visualization/_module.jl)
#

getVisualElement(data)                = data
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


function Composition.initializeVisualization(renderer::Composition.DLR_Visualization_renderer, velements::Vector{Composition.Object3D})::Void
  @assert(length(velements) > 0)
  if renderer.isInitialized
     Composition.closeVisualization(renderer)
  end

  # Initialize SimVis
  SimVis_init(renderer.host, renderer.port, Int(renderer.sync))

  # Determine visualize functions and ids of visual elements
  for i in eachindex(velements)
    push!(renderer.visualize, getVisualizeFunction(velements[i].data))
    push!(renderer.ids      , SimVis_getObjectID(0))
  end
  renderer.velements = velements
  renderer.isInitialized = true
  return nothing
end


function Composition.visualize!(renderer::Composition.DLR_Visualization_renderer, time::Float64)
  @assert(length(renderer.velements) > 0)
  if renderer.isInitialized
     velements = renderer.velements
     visualize = renderer.visualize
     ids       = renderer.ids
     for i in eachindex(velements)
        visualize[i](velements[i].data, velements[i], ids[i])
     end
     SimVis_setTime(time)
  else
     error("visualize! called without initializing visualization first.")
  end
end


function Composition.closeVisualization(renderer::Composition.DLR_Visualization_renderer)
  if renderer.isInitialized
    sleep(0.2)
    for id in renderer.ids
      SimVis_freeObjectID(id)
    end
    SimVis_shutdown()
  end
  empty!(renderer.ids)
  empty!(renderer.visualize)
  empty!(renderer.velements)
  renderer.isInitialized = false
end
