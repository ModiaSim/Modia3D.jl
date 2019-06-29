#
# This file is part of module
#   Modia3D.Composition (Modia3D/Composition/_module.jl)
#
# It is included in file Modia3D/Composition/sceneProperties.jl
# in order to be used as default for contact detection in SceneProperties(..)
#

using DataStructures

#const Dict1ValueType = Tuple{Union{SVector{3,Float64},NOTHING}, Union{SVector{3,Float64},NOTHING}, Union{SVector{3,Float64},NOTHING}, Union{Object3D,NOTHING}, Union{Object3D,NOTHING}, Union{Float64,NOTHING} }

struct KeyDict1 <: Modia3D.AbstractKeys
    contact::Bool
    distanceWithHysteresis::Float64
    index::Int64      # Coding of the two objects that are in contact to each other
end

struct ValueDict1
    contactPoint1::SVector{3,Float64}
    contactPoint2::SVector{3,Float64}
    contactNormal::SVector{3,Float64}
    actObj::Object3D
    nextObj::Object3D
    distanceOrg::Float64
    distanceWithHysteresis::Float64
    changeDirection::Int    # +1: changing at an event from negative to positive
                            #  0: no change
                            # -1: changing at an event from positive to negative
end


function Base.:isless(key1::KeyDict1, key2::KeyDict1)
    if key1.contact > key2.contact
        return true
    elseif key1.contact < key2.contact
        return false
    end
    if key1.distanceWithHysteresis < key2.distanceWithHysteresis
        return true

    elseif key1.distanceWithHysteresis == key2.distanceWithHysteresis
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

"""
    handler = ContactDetectionMPR_handler(;tol_rel = 1e-4, niter_max=100, neps=sqrt(eps()))

Generate a new contact handler for usage of the MPR algorithm in module
[`Modia3D.ContactDectionMPR`](@ref). The handler instance contains all information
about the contact situation.

# Arguments

- `tol_rel`: Relative tolerance to compute the contact point (> 0.0)
- `niter_max`: Maximum number of iterations of the MPR algorithm. If this number is reached,
               an error occurs (> 0).
- `neps`: Small number used to check whether a floating number is close to zero (> 0.0).

"""
mutable struct ContactDetectionMPR_handler <: Modia3D.AbstractContactDetection
  contactPairs::Composition.ContactPairs
  distanceComputed::Bool
  dict1::SortedDict{KeyDict1,ValueDict1}
  dict2::SortedDict{Int64,KeyDict1}
  dictCommunicate::Dict{Int64,ValuesDict}
  indexHasContact::Set{Int64}      # Set to have fast query whether index is in Set
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

    handler.distanceComputed    = false
    handler.dict1               = SortedDict{KeyDict1,ValueDict1}()
    handler.dict2               = SortedDict{Int,KeyDict1}()
    handler.dictCommunicate     = Dict{Int,ValuesDict}()
    handler.indexHasContact     = Set{Int}()
    handler.indexHasContactTemp = Set{Int}()
    handler.tol_rel             = tol_rel
    handler.niter_max           = niter_max
    handler.neps                = neps
    handler.dictCommunicateInitial = false
    return handler
  end
end
