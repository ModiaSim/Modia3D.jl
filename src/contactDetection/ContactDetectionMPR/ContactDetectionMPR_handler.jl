#
# This file is part of module
#   Modia3D.Composition (Modia3D/Composition/_module.jl)
#
# It is included in file Modia3D/Composition/sceneProperties.jl
# in order to be used as default for contact detection in SceneProperties(..)
#
export CollisionPair

using DataStructures

#const Dict1ValueType = Tuple{Union{SVector{3,Float64},NOTHING}, Union{SVector{3,Float64},NOTHING}, Union{SVector{3,Float64},NOTHING}, Union{Object3D,NOTHING}, Union{Object3D,NOTHING}, Union{Float64,NOTHING} }

const PairKey = Int64


"""
    key = DistanceKey(contact, distanceWithHysteresis, pairKey)

Generate a new distance key.
"""
struct DistanceKey
    contact::Bool
    distanceWithHysteresis::Float64
    pairKey::PairKey
end


"""
    pair = CollisionPair(contactPoint1,contactPoint2,contactNormal,obj1,obj2,
                         distanceWithHysteresis)

Generate a new `CollisionPair` object of two objects providing information about the
collision situation.
"""
mutable struct CollisionPair
    contactPoint1::SVector{3,Float64}
    contactPoint2::SVector{3,Float64}
    contactNormal::SVector{3,Float64}
    obj1::Object3D
    obj2::Object3D
    distanceWithHysteresis::Float64

    contactPairMaterial::Modia3D.AbstractContactPairMaterial  # only if contact = true, otherwise not defined

    CollisionPair(contactPoint1, contactPoint2, contactNormal, obj1, obj2, distanceWithHysteresis) =
        new(contactPoint1, contactPoint2, contactNormal, obj1, obj2, distanceWithHysteresi)
end

function updateCollisionPair!(pair::CollisionPair, contactPoint1, contactPoint2, contactNormal, obj1, obj2, distanceWithHysteresis)::Nothing
    pair.contactPoint1 = contactPoint1
    pair.contactPoint2 = contactPoint2
    pair.contactNormal = contactNormal
    pair.obj1          = obj1
    pair.obj2          = obj2
    pair.distanceWithHysteresis = distanceWithHysteresis
    return nothing
end


Base.:isless(key1::DistanceKey, key2::DistanceKey) =   key1.contact &&  key2.contact ? key1.pairKey < key2.pairKey :
                                                     ( key1.contact && !key2.contact ? true                        :
                                                     (!key1.contact &&  key2.contact ? false                       :
                                                     ( key1.distanceWithHysteresis != key2.distanceWithHysteresis   ?
                                                       key1.distanceWithHysteresis < key2.distanceWithHysteresis    :
                                                       key1.pairKey                < key2.pairKey)))

Base.:isequal(key1::DistanceKey, key2::DistanceKey) = key1.pairKey == key2.pairKey



"""
    handler = ContactDetectionMPR_handler(;tol_rel = 1e-4, niter_max=100, neps=sqrt(eps()))

Generate a new contact handler for usage of the MPR algorithm in module
[`Modia3D.ContactDetectionMPR`](@ref). The handler instance contains all information
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

  lastContactDict::Dict{PairKey,CollisionPair}
  contactDict::Dict{PairKey,CollisionPair}
  noContactDict::Dict{PairKey,CollisionPair}
  distanceDict::SortedDict{DistanceKey,Float64}

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
    handler.lastContactDict  = Dict{PairKey,CollisionPair}()
    handler.contactDict      = Dict{PairKey,CollisionPair}()
    handler.noContactDict    = Dict{PairKey,CollisionPair}()
    handler.distanceDict     = SortedDict{DistanceKey,Float64}()

    handler.tol_rel          = tol_rel
    handler.niter_max        = niter_max
    handler.neps             = neps
    return handler
  end
end
