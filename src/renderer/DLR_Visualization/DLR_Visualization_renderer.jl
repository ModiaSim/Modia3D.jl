# License for this file: MIT (expat)
# Copyright 2017-2018, DLR Institute of System Dynamics and Control
#
# This file is part of module 
#   Modia3D.Composition (Modia3D/Composition/_module.jl)
#
# It is included in file Modia3D/Composition/sceneOptions.jl
# in order to be used as default renderer in SceneOptions(..)
#


mutable struct DLR_Visualization_renderer <: Modia3D.AbstractRenderer
  velements::Vector{Composition.Object3D}  # Objects to be visualized
  ids::Vector{Ptr{Void}}             # ids[i] is the SimVis id of velements[i]
  visualize::Vector{Function}        # visualize[i] is the function to visualize velements[i]
  isInitialized::Bool                # = true, if SimVis is initialized (SimVis_init was called)

  host::String
  port::Int
  sync::Bool

  DLR_Visualization_renderer(; host="127.0.0.1",port=11000,sync=false) = 
      new(Any[], Ptr{Void}[], Function[], false, host, port, sync)
end