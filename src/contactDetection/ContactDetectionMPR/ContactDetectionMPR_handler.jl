#
# This file is part of module
#   Modia3D.Composition (Modia3D/Composition/_module.jl)
#
# It is included in file Modia3D/Composition/sceneProperties.jl
# in order to be used as default for contact detection in SceneProperties(..)
#

using DataStructures

const Dict1ValueType = Tuple{Union{SVector{3,Float64},NOTHING}, Union{SVector{3,Float64},NOTHING}, Union{SVector{3,Float64},NOTHING}, Union{Object3D,NOTHING}, Union{Object3D,NOTHING}, Union{Float64,NOTHING} }


struct KeyDict1 <: Modia3D.AbstractKeys
    contact::Bool
    distance::Float64
    index::Int
    KeyDict1(contact::Bool,distance::Float64,index::Int) = new(contact,distance,index)
end

function Base.:isless(key1::KeyDict1, key2::KeyDict1)
    if key1.contact > key2.contact
        return true
    elseif key1.contact < key2.contact
        return false
    end
    if key1.distance < key2.distance
        return true
    elseif key1.distance == key2.distance
        if key1.index < key2.index
            return true
        end
    end
    return false
end

function Base.:isequal(key1::KeyDict1, key2::KeyDict1)
    if key1.index == key2.index
        return true
    end
    return false
end


mutable struct ValuesDict <: Modia3D.AbstractValues
    i::Int
    delta_dot_initial::Float64
    commonCollisionProp::Union{Modia3D.AbstractContactMaterial,NOTHING}
    ValuesDict(index::Int; delta_dot_initial::Float64=-0.001, commProp::Union{Modia3D.AbstractContactMaterial,NOTHING}=nothing) = new(index, delta_dot_initial,commProp)
end

mutable struct ContactDetectionMPR_handler <: Modia3D.AbstractContactDetection
  contactPairs::Composition.ContactPairs
  distanceComputed::Bool
  dict1::SortedDict{KeyDict1,Dict1ValueType}
  dict2::SortedDict{Int,Array{Float64,1}}
  dictCommunicate::Dict{Int,ValuesDict}
  indexHasContact::Array{Int,1}
  dictCommunicateInitial::Bool


  tol_rel::Float64
  niter_max::Int
  neps::Float64

  # Visualization options
  visualizeContactPoints::Bool
  visualizeSupportPoints::Bool
  defaultContactSphereDiameter::Float64

  function ContactDetectionMPR_handler(;tol_rel   = 1e-4,
                                        niter_max = 100 ,
                                        neps      = sqrt(eps()))
    @assert(tol_rel > 0.0)
    @assert(niter_max > 0)
    @assert(neps > 0.0)

    handler = new()

    handler.distanceComputed = false
    handler.dict1            = SortedDict{KeyDict1,Dict1ValueType}()
    handler.dict2            = SortedDict{Int,Array{Float64,1}}()
    handler.dictCommunicate  = Dict{Int,ValuesDict}()
    handler.indexHasContact  = Array{Int,1}()
    handler.tol_rel          = tol_rel
    handler.niter_max        = niter_max
    handler.neps             = neps
    handler.dictCommunicateInitial = false
    return handler
  end
end
