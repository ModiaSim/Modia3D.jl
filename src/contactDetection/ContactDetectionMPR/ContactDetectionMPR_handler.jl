#
# This file is part of module
#   Modia3D.Composition (Modia3D/Composition/_module.jl)
#
# It is included in file Modia3D/Composition/sceneProperties.jl
# in order to be used as default for contact detection in SceneProperties(..)
#

using DataStructures

const Dict1ValueType = Tuple{Int64, MVector{3,Float64}, MVector{3,Float64}, MVector{3,Float64}, Union{Object3D,Void}, Union{Object3D,Void}}

mutable struct ContactDetectionMPR_handler <: Modia3D.AbstractContactDetection
  contactPairs::Composition.ContactPairs

  distanceComputed::Bool

  # dict1::SortedDict{Float64,Array{Array{Float64,1}}}
  dict1::SortedDict{Float64,Dict1ValueType}
  dict2::SortedDict{Int,Array{Float64,1}}


   contactPoint1::Vector{MVector{3,Float64}}       # Absolute position vector to first contact point on contactObj1
   contactPoint2::Vector{MVector{3,Float64}}       # Absolute position vector to second contact point on contactObj2
   contactNormal::Vector{MVector{3,Float64}}       # Unit normal to surface on contactPoint1 (in world frame)

   contactObj1::Vector{Union{Object3D,Void}}
   contactObj2::Vector{Union{Object3D,Void}}



  tol_rel::Float64
  niter_max::Int
  neps::Float64

  # Visualization options
  visualizeContactPoints::Bool
  visualizeSupportPoints::Bool

  function ContactDetectionMPR_handler(;tol_rel   = 1e-4,
                                        niter_max = 100 ,
                                        neps      = sqrt(eps()))
    @assert(tol_rel > 0.0)
    @assert(niter_max > 0)
    @assert(neps > 0.0)

    handler = new()

    handler.distanceComputed = false
    handler.dict1            = SortedDict{Float64,Dict1ValueType}()
    handler.dict2            = SortedDict{Int,Array{Float64,1}}()
    handler.tol_rel          = tol_rel
    handler.niter_max        = niter_max
    handler.neps             = neps
    handler.visualizeContactPoints = false
    handler.visualizeSupportPoints = false
    return handler
  end
end
