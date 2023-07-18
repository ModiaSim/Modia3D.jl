# License for this file: MIT (expat)
# Copyright 2017-2018, DLR Institute of System Dynamics and Control
#
# This file is part of module
#   Modia3D.Composition (Modia3D/Composition/_module.jl)

"""
    m = ElasticContactPairResponseMaterial(c_res, d_res, mu_k, mu_r, vsmall, wsmall)

Return an `ElasticContactPairResponseMaterial < AbstractContactPairMaterial` object.
"""
mutable struct ElasticContactPairResponseMaterial <: Modia3D.AbstractContactPairMaterial
    c_res::Float64
    c_geo::Float64
    n_geo::Float64
    d_res::Float64
    mu_k::Float64
    mu_r::Float64
    mu_r_geo::Float64
    vsmall::Float64
    wsmall::Float64
end


regularize(absv,vsmall) = absv >= vsmall ? absv : absv*(absv/vsmall)*(1.0 - (absv/vsmall)/3.0) + vsmall/3.0


function resultantDampingCoefficient(cor, abs_vreln, vsmall, maximumContactDamping)
    @assert(cor >= 0.0 && cor <= 1.0)
    @assert(abs_vreln >= 0.0)
    @assert(vsmall > 0.0)
    @assert(maximumContactDamping > 0.0)

    cor_res = abs_vreln > vsmall ? cor : 0.0
    d_res   = min(maximumContactDamping, 8.0*(1.0 - cor_res)/(5*cor_res*abs_vreln))
    return d_res
end

function elasticContactPairCoefficients(obj1::Object3D{F}, obj2::Object3D{F}) where F <: Modia3D.VarFloatType
    solid1::Shapes.Solid{F} = obj1.feature
    solid2::Shapes.Solid{F} = obj2.feature
    mu_r_geo = F(0.0)
    if !solid1.isFlat && solid2.isFlat
        mu_r_geo = solid1.contactSphereRadius
    elseif solid1.isFlat && !solid2.isFlat
        mu_r_geo = solid2.contactSphereRadius
    else # (solid1.isFlat && solid2.isFlat) || (!solid1.isFlat && !solid2.isFlat)
        r1::F = solid1.contactSphereRadius
        r2::F = solid2.contactSphereRadius
        mu_r_geo = r1*r2/(r1 + r2)
    end

    n_geo = F(1.5)
    c_geo = F(4/3*sqrt(mu_r_geo))

    return (c_geo, n_geo, mu_r_geo)
end


"""
   resonseMaterial = contactStart(matPair::Modia3D.ElasticContactPairMaterial,
                                  obj1,obj2,rContact,contactNormal,elasticContactReductionFactor,
                                  maximumContactDamping)

Return a `responseMaterial::ElasticContactPairResponseMaterial` object
at the start of a collision.
"""
function contactStart(matPair::Shapes.ElasticContactPairMaterial,
                      obj1::Object3D{F},
                      obj2::Object3D{F},
                      rContact::SVector{3,F},
                      contactNormal::SVector{3,F},
                      elasticContactReductionFactor::F,
                      maximumContactDamping::F) where F <: Modia3D.VarFloatType
    # Compute spring constant
    name1 = obj1.feature.contactMaterial
    name2 = obj2.feature.contactMaterial

    solidMaterialPalette::OrderedCollections.OrderedDict{String,Shapes.SolidMaterial} = Shapes.solidMaterialPalette[1]
    mat1  = solidMaterialPalette[name1]
    mat2  = solidMaterialPalette[name2]

    E1    = mat1.YoungsModulus
    E2    = mat2.YoungsModulus
    nu1   = mat1.PoissonsRatio
    nu2   = mat2.PoissonsRatio
    if E1 <= 0.0 || E2 <= 0.0 || nu1 <= 0.0 || nu1 >= 1.0 ||
        nu2 <= 0.0 || nu2 >= 1.0 || elasticContactReductionFactor <= 0.0
        responseMaterial = nothing
    else
        @assert(E1 > 0.0)
        @assert(E2 > 0.0)
        @assert(nu1 > 0.0 && nu1 < 1.0)
        @assert(nu2 > 0.0 && nu2 < 1.0)
        @assert(elasticContactReductionFactor > 0.0)
        c1 = E1/(1 - nu1^2)
        c2 = E2/(1 - nu2^2)
        c_res = elasticContactReductionFactor*c1*c2/(c1 + c2)

        # Compute damping constant
        delta_dot_start = normalRelativeVelocityAtContact(obj1, obj2, rContact, contactNormal)
        d_res = Modia3D.resultantDampingCoefficient(matPair.coefficientOfRestitution, abs(delta_dot_start), matPair.vsmall, maximumContactDamping)

        # Determine other coefficients
        (c_geo, n_geo, mu_r_geo) = elasticContactPairCoefficients(obj1,obj2)
        responseMaterial = ElasticContactPairResponseMaterial(c_res, c_geo, n_geo, d_res,
                                matPair.slidingFrictionCoefficient,
                                matPair.rotationalResistanceCoefficient, mu_r_geo,
                                matPair.vsmall, matPair.wsmall)
    end
    return (responseMaterial, Shapes.ElasticContactPairKind)
end


contactEnd(mat::ElasticContactPairResponseMaterial,obj1,obj2)::Nothing = nothing
contactEnd(mat::Nothing,obj1,obj2)::Nothing = nothing

"""
    (f1,f2,t1,t2) = responseCalculation(material::ElasticContactPairResponseMaterial,
                            obj1, obj2, rContact, contactNormal, distanceWithHysteresis, time)

Compute contact forces `f1,f2` and contact torques `t1,t2` from the elastic contact pair
response material `material` the two penetrating objects `obj1, obj2`, the contact point `rContact`,
the contact normal `contactNormal` and the largest penetration depth `distanceWithHysteresis`
at time `time`.
"""
function responseCalculation(material::ElasticContactPairResponseMaterial, obj1::Object3D{F}, obj2::Object3D{F},
                             rContact::SVector{3,F}, e_n::SVector{3,F},
                             s::F, time, file, sim)::Tuple{SVector{3,F},SVector{3,F},SVector{3,F},SVector{3,F},ContactResults{F}} where F <: Modia3D.VarFloatType
    # Material
    c_res    = F(material.c_res)
    c_geo    = F(material.c_geo)
    n_geo    = F(material.n_geo)
    d_res    = F(material.d_res)
    mu_k     = F(material.mu_k)
    mu_r     = F(material.mu_r)
    mu_r_geo = F(material.mu_r_geo)
    vsmall   = F(material.vsmall)
    wsmall   = F(material.wsmall)

    # Contact points and distances to local part frame (in world frame)
    r_rel1 = rContact - obj1.r_abs  # position vector from obj1 to average contact point, resolved in world
    r_rel2 = rContact - obj2.r_abs  # position vector from obj2 to average contact point, resolved in world

    # Velocities and angular velocities of contact frames in world frame
    w1 = obj1.R_abs'*obj1.w           # absolute angular velocity vector of obj1, resolved in world
    w2 = obj2.R_abs'*obj2.w           # absolute angular velocity vector of obj2, resolved in world
    v1 = obj1.v0 + cross(w1, r_rel1)  # absolute velocity vector of obj1 at average contact point, resolved in world
    v2 = obj2.v0 + cross(w2, r_rel2)  # absolute velocity vector of obj2 at average contact point, resolved in world

    # Velocities and angular velocities in normal and tangential direction
    w_rel   = w2 - w1                                        # angular velocity vector of obj2 relative to obj1, resolved in world
    e_w_reg = w_rel/Modia3D.regularize(norm(w_rel), wsmall)  # regularized direction vector of w_rel, resolved in world
    v_rel   = v2 - v1                                        # velocity vector of obj2 relative to obj1 at average contact point, resolved in world

    # e_n points from contact point on obj2 to contact point on obj1, i.e. normal direction into obj2
    # s = signed distance of the contact points, negative during contact
    delta_dot  = dot(v_rel, e_n)                            # signed relative velocity in normal direction, positive for expansion (sign inconsistent with delta!)
    v_t        = v_rel - delta_dot*e_n                      # tangential velocity vector of obj2 relative to obj1 at average contact point, resolved in world
    e_t_reg    = v_t/Modia3D.regularize(norm(v_t), vsmall)  # regularized direction vector of v_t, resolved in world
    delta      = -s                                         # penetration, positive during contact
    delta_comp = delta * sqrt(abs(delta))

    fn = -c_res * c_geo * delta_comp * (1 - d_res*delta_dot)  # normal force, negative for pressure!
    ft = -mu_k*fn*e_t_reg                                     # tangential force vector acting on obj1, resolved in world
    f1 = fn * e_n + ft                                        # total force vector acting at average contact point on obj1, resolved in world
    f2 = -f1                                                  # total force vector acting at average contact point on obj2, resolved in world

    tau = -mu_r * mu_r_geo * fn * e_w_reg                     # torque vector acting at average contact point on obj1, resolved in world
    t1  = cross(r_rel1, f1) + tau                             # total torque vector acting at obj1, resolved in world
    t2  = cross(r_rel2, f2) - tau                             # total torque vector acting at obj2, resolved in world

    return (f1, f2, t1, t2,
            ContactResults(delta,        # normal contact penetration (positive during contact)
                           -delta_dot,   # normal contact penetration velocity (positive for compression)
                           norm(v_t),    # absolute value of the tangential relative velocity
                           norm(w_rel),  # absolute value of the relative angular velocity
                           -fn,          # normal contact force (positive for pressure)
                           norm(ft),     # absolute value of the tangential contact force
                           norm(tau),    # absolute value of the contact torque
                           rContact,     # absolute position vector of the contact point, resolved in world
                           -e_n,         # unit vector in contact normal direction, pointing into obj1, resolved in world
                           f1,           # total contact force vector acting at the contact point on obj1, resolved in world
                           tau))         # total contact torque vector acting at the contact point on obj1, resolved in world
end

responseCalculation(material::Nothing, obj1::Object3D{F}, obj2::Object3D{F},
                    rContact::SVector{3,F}, e_n::SVector{3,F},
                    s::F, time, file) where F <: Modia3D.VarFloatType =
                    (Modia3D.ZeroVector3D(F), Modia3D.ZeroVector3D(F), Modia3D.ZeroVector3D(F), Modia3D.ZeroVector3D(F))
