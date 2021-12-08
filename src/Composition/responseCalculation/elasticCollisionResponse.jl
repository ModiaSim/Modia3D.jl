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


resultantCoefficientOfRestitution(cor, abs_vreln, vsmall; cor_min=0.01) =
    cor + (cor_min - cor)*exp(log(0.01)*(abs_vreln/vsmall))


function resultantDampingCoefficient(cor, abs_vreln, vsmall; cor_min=0.01, d_max=1000.0)
    @assert(cor >= 0.0 && cor <= 1.0)
    @assert(abs_vreln >= 0.0)
    @assert(vsmall  > 0.0)
    @assert(cor_min > 0.0)
    @assert(d_max   > 0.0)

    cor_res = resultantCoefficientOfRestitution(cor,abs_vreln,vsmall;cor_min=cor_min)
    return min(d_max, 8.0*(1.0 - cor_res)/(5*cor_res*regularize(abs_vreln,vsmall)))
end


function elasticContactPairCoefficients(obj1::Object3D, obj2::Object3D)
  if typeof(obj1.feature.shape) <: Modia3D.Shapes.Sphere && typeof(obj2.feature.shape) <: Modia3D.Shapes.Sphere
    r1 = obj1.feature.shape.diameter*0.5
    r2 = obj2.feature.shape.diameter*0.5
    mu_r_geo = r1*r2/(r1 + r2)
    n_geo = 1.5
    c_geo = 4/3*sqrt(mu_r_geo)
  elseif typeof(obj1.feature.shape) <: Modia3D.Shapes.Sphere && typeof(obj2.feature.shape) != Modia3D.Shapes.Sphere
    mu_r_geo = obj1.feature.shape.diameter*0.5
    n_geo = 1.5
    c_geo = 4/3*sqrt(mu_r_geo)
  elseif typeof(obj1.feature.shape) != Modia3D.Shapes.Sphere && typeof(obj2.feature.shape) <: Modia3D.Shapes.Sphere
    mu_r_geo = obj2.feature.shape.diameter*0.5
    n_geo = 1.5
    c_geo = 4/3*sqrt(mu_r_geo)
  else
    mu_r_geo = 1.0
    n_geo = 1.0
    c_geo = 1.0
  end
  return (c_geo, n_geo, mu_r_geo)
end


"""
   resonseMaterial = contactStart(matPair::Modia3D.ElasticContactPairMaterial,
                                  obj1,obj2,rContact,contactNormal)

Return a `responseMaterial::ElasticContactPairResponseMaterial` object
at the start of a collision.
"""
function contactStart(matPair::Shapes.ElasticContactPairMaterial,
                      obj1::Object3D,
                      obj2::Object3D,
                      rContact::Frames.Vector3D,
                      contactNormal::Frames.Vector3D,
                      elasticContactReductionFactor::Float64)
    # Compute spring constant
    name1 = obj1.feature.contactMaterial
    name2 = obj2.feature.contactMaterial
    mat1  = Shapes.solidMaterialPalette[name1]
    mat2  = Shapes.solidMaterialPalette[name2]
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
      d_res = Modia3D.resultantDampingCoefficient(matPair.coefficientOfRestitution, abs(delta_dot_start), matPair.vsmall)

      # Determine other coefficients
      (c_geo, n_geo, mu_r_geo) = elasticContactPairCoefficients(obj1,obj2)
      responseMaterial = ElasticContactPairResponseMaterial(c_res, c_geo, n_geo, d_res,
                              matPair.slidingFrictionCoefficient,
                              matPair.rotationalResistanceCoefficient, mu_r_geo,
                              matPair.vsmall, matPair.wsmall)
    end
    return responseMaterial
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
function responseCalculation(material::ElasticContactPairResponseMaterial, obj1::Object3D, obj2::Object3D,
                             rContact::Frames.Vector3D, e_n::Frames.Vector3D,
                             s::Float64, time, file, sim)::Tuple{Frames.Vector3D,Frames.Vector3D,Frames.Vector3D,Frames.Vector3D}
    # Material
    c_res    = material.c_res
    c_geo    = material.c_geo
    n_geo    = material.n_geo
    d_res    = material.d_res
    mu_k     = material.mu_k
    mu_r     = material.mu_r
    mu_r_geo = material.mu_r_geo
    vsmall   = material.vsmall
    wsmall   = material.wsmall

    ### signed velocity and relative velocity ####
    # Contact points and distances to local part frame (in world frame)
    r_rel1 = rContact - obj1.r_abs
    r_rel2 = rContact - obj2.r_abs

    # Velocities and angular velocities of contact frames in world frame
    w1 = obj1.R_abs'*obj1.w
    w2 = obj2.R_abs'*obj2.w
    v1 = obj1.v0 + cross(w1,r_rel1)
    v2 = obj2.v0 + cross(w2,r_rel2)

    # Velocities and angular velocities in normal and tangential direction
    w_rel   = w2 - w1
    e_w_reg = w_rel/Modia3D.regularize(norm(w_rel),wsmall)
    v_rel   = v2 - v1

    # delta_dot ... signed relative velocity in normal direction
    # v_t       ... relative velocity vector in tangential direction
    # delta     ... signed distance
    delta_dot = dot(v_rel,e_n)
    v_t       = v_rel - delta_dot*e_n
    e_t_reg   = v_t/Modia3D.regularize(norm(v_t),vsmall)
    delta     = -s

    if c_geo != 1.0 && n_geo != 1.0
      delta_comp = delta * sqrt(abs(delta))
    else
      delta_comp = delta
    end
    fn = -max(0.0, c_res * c_geo * delta_comp * (1 - d_res*delta_dot) )
    ft = -mu_k*fn*e_t_reg
    f1 = fn * e_n + ft
    f2 = -f1

    tau = -mu_r * mu_r_geo * fn * e_w_reg
    t1  = cross(r_rel1,f1) + tau
    t2  = cross(r_rel2,f2) - tau

    return (f1,f2,t1,t2)
end

responseCalculation(material::Nothing, obj1::Object3D, obj2::Object3D,
                    rContact::Frames.Vector3D, e_n::Frames.Vector3D,
                    s::Float64, time, file) =
                    (Modia3D.ZeroVector3D, Modia3D.ZeroVector3D, Modia3D.ZeroVector3D, Modia3D.ZeroVector3D)
