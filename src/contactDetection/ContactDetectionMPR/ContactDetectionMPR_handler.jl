#
# This file is part of module
#   Modia3D.Composition (Modia3D/Composition/_module.jl)
#
# It is included in file Modia3D/Composition/sceneProperties.jl
# in order to be used as default for contact detection in SceneProperties(..)
#
export ContactPair
export NoContactPair
export updateContactPair!
export updateNoContactPair!

using DataStructures

#const Dict1ValueType = Tuple{Union{SVector{3,Float64},NOTHING}, Union{SVector{3,Float64},NOTHING}, Union{SVector{3,Float64},NOTHING}, Union{Object3D,NOTHING}, Union{Object3D,NOTHING}, Union{Float64,NOTHING} }

const PairID = Int64

"""
    key = DistanceKey(contact, distanceWithHysteresis, pairID)

Generate a new distance key.
"""
struct DistanceKey
    contact::Bool
    distanceWithHysteresis::Float64
    pairID::PairID
end


"""
    pair = ContactPair(contactPoint1,contactPoint2,contactNormal,obj1,obj2,
                       distanceWithHysteresis)

Generate a new `ContactPair` object of two objects that have contact=true.
"""
mutable struct ContactPair
    contactPoint1::SVector{3,Float64}
    contactPoint2::SVector{3,Float64}
    contactNormal::SVector{3,Float64}
    obj1::Object3D
    obj2::Object3D
    distanceWithHysteresis::Float64

    contactPairMaterial::Modia3D.AbstractContactPairMaterial  # only if contact = true, otherwise not defined

    ContactPair(contactPoint1::SVector{3,Float64}, contactPoint2::SVector{3,Float64}, contactNormal::SVector{3,Float64},
                obj1::Object3D, obj2::Object3D, distanceWithHysteresis::Float64) =
        new(contactPoint1, contactPoint2, contactNormal, obj1, obj2, distanceWithHysteresis)
end


"""
    pair = NoContactPair(contactPoint1,contactPoint2,contactNormal,obj1,obj2,
                         distanceWithHysteresis)

Generate a new `NoContactPair` object of two objects that have contact=false.
"""
mutable struct NoContactPair
    contactPoint1::Union{SVector{3,Float64},Nothing}
    contactPoint2::Union{SVector{3,Float64},Nothing}
    contactNormal::Union{SVector{3,Float64},Nothing}
    obj1::Object3D
    obj2::Object3D
    distanceWithHysteresis::Float64

    contactPairMaterial::Modia3D.AbstractContactPairMaterial  # only if contact = true, otherwise not defined

    NoContactPair(contactPoint1, contactPoint2, contactNormal,
                  obj1::Object3D, obj2::Object3D, distanceWithHysteresis::Float64) =
        new(contactPoint1, contactPoint2, contactNormal, obj1, obj2, distanceWithHysteresis)
end


function updateContactPair!(pair::ContactPair, contactPoint1::SVector{3,Float64}, contactPoint2::SVector{3,Float64}, contactNormal::SVector{3,Float64},
                            obj1::Object3D, obj2::Object3D, distanceWithHysteresis::Float64)::Nothing
    pair.contactPoint1 = contactPoint1
    pair.contactPoint2 = contactPoint2
    pair.contactNormal = contactNormal
    pair.obj1          = obj1
    pair.obj2          = obj2
    pair.distanceWithHysteresis = distanceWithHysteresis
    return nothing
end


function updateNoContactPair!(pair::NoContactPair, contactPoint1, contactPoint2, contactNormal,
                              obj1::Object3D, obj2::Object3D, distanceWithHysteresis::Float64)::Nothing
    pair.contactPoint1 = contactPoint1
    pair.contactPoint2 = contactPoint2
    pair.contactNormal = contactNormal
    pair.obj1          = obj1
    pair.obj2          = obj2
    pair.distanceWithHysteresis = distanceWithHysteresis
    return nothing
end



Base.:isless(key1::DistanceKey, key2::DistanceKey) =   key1.contact &&  key2.contact ? key1.pairID < key2.pairID :
                                                     ( key1.contact && !key2.contact ? true                        :
                                                     (!key1.contact &&  key2.contact ? false                       :
                                                     ( key1.distanceWithHysteresis != key2.distanceWithHysteresis   ?
                                                       key1.distanceWithHysteresis < key2.distanceWithHysteresis    :
                                                       key1.pairID                < key2.pairID)))

Base.:isequal(key1::DistanceKey, key2::DistanceKey) = key1.pairID == key2.pairID



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

  lastContactDict::Dict{PairID,ContactPair}
  contactDict::Dict{    PairID,ContactPair}
  noContactDict::Dict{  PairID,NoContactPair}
  distanceSet::SortedSet{DistanceKey}

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
    handler.lastContactDict  = Dict{PairID,ContactPair}()
    handler.contactDict      = Dict{PairID,ContactPair}()
    handler.noContactDict    = Dict{PairID,NoContactPair}()
    handler.distanceSet      = SortedSet{DistanceKey}()

    handler.tol_rel          = tol_rel
    handler.niter_max        = niter_max
    handler.neps             = neps
    return handler
  end
end
