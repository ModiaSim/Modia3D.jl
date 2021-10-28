#
# This file is part of module
#   Modia3D.Composition (Modia3D/Composition/_module.jl)

export ContactPair
export updateContactPair!

using OrderedCollections


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
    ContactPair(contactPointA, contactPointB,
        contactNormal, objA, objB,
        distanceWithHysteresis, supportPointsDefined,
        support1A, support2A, support3A,
        support1B, support2B, support3B)

Generate a new `ContactPair` object of two objects that have `contact = true`.
"""
mutable struct ContactPair
    contactPoint1::SVector{3,Float64}
    contactPoint2::SVector{3,Float64}
    contactNormal::SVector{3,Float64}
    obj1::Object3D
    obj2::Object3D
    distanceWithHysteresis::Float64

    supportPointsDefined::Bool
    support1A::SVector{3,Float64}
    support2A::SVector{3,Float64}
    support3A::SVector{3,Float64}
    support1B::SVector{3,Float64}
    support2B::SVector{3,Float64}
    support3B::SVector{3,Float64}

    contactPairMaterial::Union{Modia3D.AbstractContactPairMaterial,Nothing}  # only if contact = true, otherwise not defined

    ContactPair(contactPoint1::SVector{3,Float64}, contactPoint2::SVector{3,Float64},
            contactNormal::SVector{3,Float64},
            obj1::Object3D, obj2::Object3D, distanceWithHysteresis::Float64, supportPointsDefined::Bool,
            support1A::SVector{3,Float64}, support2A::SVector{3,Float64}, support3A::SVector{3,Float64}, support1B::SVector{3,Float64}, support2B::SVector{3,Float64}, support3B::SVector{3,Float64}) =
        new(contactPoint1, contactPoint2, contactNormal, obj1, obj2, distanceWithHysteresis, supportPointsDefined,
        support1A, support2A, support3A, support1B, support2B, support3B)
end


function updateContactPair!(pair::ContactPair, contactPoint1::SVector{3,Float64}, contactPoint2::SVector{3,Float64}, contactNormal::SVector{3,Float64},
                            obj1::Object3D, obj2::Object3D, distanceWithHysteresis::Float64, supportPointsDefined::Bool,
                            support1A::SVector{3,Float64}, support2A::SVector{3,Float64}, support3A::SVector{3,Float64}, support1B::SVector{3,Float64}, support2B::SVector{3,Float64}, support3B::SVector{3,Float64})::Nothing
    pair.contactPoint1 = contactPoint1
    pair.contactPoint2 = contactPoint2
    pair.contactNormal = contactNormal
    pair.obj1          = obj1
    pair.obj2          = obj2
    pair.distanceWithHysteresis = distanceWithHysteresis
    pair.supportPointsDefined = supportPointsDefined
    pair.support1A     = support1A
    pair.support2A     = support2A
    pair.support3A     = support3A
    pair.support1B     = support1B
    pair.support2B     = support2B
    pair.support3B     = support3B
    return nothing
end


Base.:isless( value1::ContactPair, value2::ContactPair) =
    value1.distanceWithHysteresis < value2.distanceWithHysteresis
Base.:isequal(value1::ContactPair, value2::ContactPair) =
    value1.distanceWithHysteresis == value2.distanceWithHysteresis

Base.:isless(key1::DistanceKey, key2::DistanceKey) =
    key1.contact &&  key2.contact ? key1.pairID < key2.pairID : ( key1.contact && !key2.contact ? true : (!key1.contact &&  key2.contact ? false                       :
                                                     ( key1.distanceWithHysteresis != key2.distanceWithHysteresis   ?
                                                       key1.distanceWithHysteresis < key2.distanceWithHysteresis    :
                                                       key1.pairID                < key2.pairID)))

Base.:isequal(key1::DistanceKey, key2::DistanceKey) =
    key1.pairID == key2.pairID


"""
    handler = ContactDetectionMPR_handler(;tol_rel = 1e-4, niter_max=100, neps=sqrt(eps()))

Generate a new contact handler for usage of the MPR algorithm
The handler instance contains all information
about the contact situation.

# Arguments

- `tol_rel`: Relative tolerance to compute the contact point (> 0.0)
- `niter_max`: Maximum number of iterations of the MPR algorithm. If this number is reached,
               an error occurs (> 0).
- `neps`: Small number used to check whether a floating number is close to zero (> 0.0).

"""
mutable struct ContactDetectionMPR_handler <: Modia3D.AbstractContactDetection
    distanceComputed::Bool

    lastContactDict::Dict{PairID,ContactPair}
    contactDict::Dict{    PairID,ContactPair}
    noContactMinVal::Float64

    tol_rel::Double64
    niter_max::Int
    neps::Double64

    contactPairs::Composition.ContactPairs

    # Visualization options
    visualizeContactPoints::Bool
    visualizeSupportPoints::Bool
    defaultContactSphereDiameter::Float64

    function ContactDetectionMPR_handler(; tol_rel= 1.0e-20,
                                        niter_max = 100,
                                        neps      = 100.0 * eps(Double64) )
        @assert(tol_rel > 0.0)
        @assert(niter_max > 0)
        @assert(neps > 0.0)
        new(false, Dict{PairID,ContactPair}(), Dict{PairID,ContactPair}(), 42.0, tol_rel, niter_max, neps)
    end
end
