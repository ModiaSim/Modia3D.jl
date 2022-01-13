#
# This file is part of module
#   Modia3D.Composition (Modia3D/Composition/_module.jl)

export ContactPair
export updateContactPair!

using OrderedCollections


const PairID = Int64


"""
    ContactPair(contactPointA, contactPointB,
        contactNormal, objA, objB,
        distanceWithHysteresis, supportPointsDefined,
        support1A, support2A, support3A,
        support1B, support2B, support3B)

Generate a new `ContactPair` object of two objects that have `contact = true`.
"""
mutable struct ContactPair{F <: Modia3D.VarFloatType}
    contactPoint1::SVector{3,F}
    contactPoint2::SVector{3,F}
    contactNormal::SVector{3,F}
    obj1::Object3D{F}
    obj2::Object3D{F}
    distanceWithHysteresis::F

    supportPointsDefined::Bool
    support1A::SVector{3,F}
    support2A::SVector{3,F}
    support3A::SVector{3,F}
    support1B::SVector{3,F}
    support2B::SVector{3,F}
    support3B::SVector{3,F}

    contactPairMaterial::Union{Modia3D.AbstractContactPairMaterial,Nothing}  # only if contact = true, otherwise not defined

    ContactPair{F}(contactPoint1::SVector{3,F}, contactPoint2::SVector{3,F},
            contactNormal::SVector{3,F},
            obj1::Object3D{F}, obj2::Object3D{F}, distanceWithHysteresis::F, supportPointsDefined::Bool,
            support1A::SVector{3,F}, support2A::SVector{3,F}, support3A::SVector{3,F}, support1B::SVector{3,F}, support2B::SVector{3,F}, support3B::SVector{3,F}) where F <: Modia3D.VarFloatType =
        new(contactPoint1, contactPoint2, contactNormal, obj1, obj2, distanceWithHysteresis, supportPointsDefined,
        support1A, support2A, support3A, support1B, support2B, support3B)
end


function updateContactPair!(pair::ContactPair{F}, contactPoint1::SVector{3,F}, contactPoint2::SVector{3,F}, contactNormal::SVector{3,F},
                            obj1::Object3D{F}, obj2::Object3D{F}, distanceWithHysteresis::F, supportPointsDefined::Bool,
                            support1A::SVector{3,F}, support2A::SVector{3,F}, support3A::SVector{3,F}, support1B::SVector{3,F}, support2B::SVector{3,F}, support3B::SVector{3,F})::Nothing where F <: Modia3D.VarFloatType
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


"""
    handler = ContactDetectionMPR_handler(;tol_rel = 1e-7, niter_max=100)

Generate a new contact handler for usage of the MPR algorithm
The handler instance contains all information
about the contact situation.

# Arguments

- `tol_rel`: Relative tolerance to compute the contact point (> 0.0)
- `niter_max`: Maximum number of iterations of the MPR algorithm. If this number is reached,
               an error occurs (> 0).
"""
mutable struct ContactDetectionMPR_handler{T,F} <: Modia3D.AbstractContactDetection
    distanceComputed::Bool

    lastContactDict::Dict{PairID,ContactPair{F}}
    contactDict::Dict{    PairID,ContactPair{F}}
    noContactMinVal::F

    tol_rel::T
    niter_max::Int

    contactPairs::Composition.ContactPairs

    # Visualization options
    visualizeContactPoints::Bool
    visualizeSupportPoints::Bool
    defaultContactSphereDiameter::Float64

    function ContactDetectionMPR_handler{T,F}(;tol_rel  = 1.0e-20,
                                            niter_max = 100) where {T,F}
        @assert(tol_rel > 0.0)
        @assert(niter_max > 0)
        new(false, Dict{PairID,ContactPair{F}}(), Dict{PairID,ContactPair{F}}(), F(42.0), tol_rel, niter_max)
    end
end
ContactDetectionMPR_handler(; kwargs...) = ContactDetectionMPR_handler{Modia3D.MPRFloatType, Float64}(; kwargs...)
