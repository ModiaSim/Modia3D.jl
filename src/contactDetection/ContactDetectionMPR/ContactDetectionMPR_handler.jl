#
# This file is part of module
#   Modia3D.Composition (Modia3D/Composition/_module.jl)
#
# It is included in file Modia3D/Composition/sceneProperties.jl
# in order to be used as default for contact detection in SceneProperties(..)
#

using DataStructures

#const Dict1ValueType = Tuple{Union{SVector{3,Float64},NOTHING}, Union{SVector{3,Float64},NOTHING}, Union{SVector{3,Float64},NOTHING}, Union{Object3D,NOTHING}, Union{Object3D,NOTHING}, Union{Float64,NOTHING} }

const PairID = Int64

struct PairKey <: Modia3D.AbstractKeys
    distanceWithHysteresis::Float64
    pairID::PairID
end


"""
    m = MaterialContactPair(m1::ElasticContactMaterial2,
                            m2::ElasticContactMaterial2,
                            delta_dot_initial::Float64)

Generate a MaterialContactPair object (only temporary now)
"""
mutable struct MaterialContactPair
    c_res::Float64
    d_res::Float64
    mu_k::Float64
    mu_r::Float64
    vsmall::Float64
    wsmall::Float64

    function MaterialContactPair(m1::ElasticContactMaterial2, m2::ElasticContactMaterial2, delta_dot_initial::Float64)
        collMaterial = getCommonCollisionProperties(m1.name, m2.name)
        c_res  = m1.c*m2.c/(m1.c + m2.c)
        vsmall = (m1.v_small + m2.v_small)/2
        wsmall = (m1.w_small + m2.w_small)/2
        mu_k   = collMaterial.mu_k
        mu_r   = collMaterial.mu_r
        d_res  = Modia3D.resultantDampingCoefficient(collMaterial.cor, abs(delta_dot_initial), vsmall)

        new(c_res, d_res, mu_k, mu_r, vsmall, wsmall)
    end
end


"""
    contactPair = ContactPair(...)

Generate an new `ContactPair` object of two objects that are penetrating each other.
"""
mutable struct ContactPair
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
    contactMaterial::MaterialContactPair
end


"""
    noContactPair = NoContactPair(...)

Generate an new `NoContactPair` object of two objects that are not penetrating each other
(= `ContactPair` but without `contactMaterial`).
"""
mutable struct NoContactPair
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



Base.:isless(key1::KeyDict1, key2::KeyDict1) = key1.distanceWithHysteresis != key2.distanceWithHysteresis ?
                                                  key1.distanceWithHysteresis < key2.distanceWithHysteresis :
                                                  key1.pairID                 < key2.pairID

Base.:isequal(key1::KeyDict1, key2::KeyDict1) = key1.index == key2.index



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

  lastContactDict::Dict{PairID,MaterialContactPair}
  contactDict::Dict{PairID,ContactPair}
  distanceDict::SortedDict{PairKey,Float64}
  noContactDict::Dict{PairKey,NoContactPair}

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
    handler.lastContactDict  = Dict{PairID,MaterialContactPair}()
    handler.contactDict      = Dict{PairID,ContactPair}()
    handler.distanceDict     = SortedDict{PairKey,Float64}()
    handler.noContactDict    = Dict{PairKey,NoContactPair}()
    handler.tol_rel          = tol_rel
    handler.niter_max        = niter_max
    handler.neps             = neps
    return handler
  end
end
