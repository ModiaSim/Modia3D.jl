# License for this file: MIT (expat)
# Copyright 2017-2018, DLR Institute of System Dynamics and Control
#
# This file is part of module
#   Modia3D.Composition (Modia3D/Composition/_module.jl)
#
# Functions that have to be provided from an object used for contact pair response

contactStart(material::Modia3D.AbstractContactMaterial, obj1, obj2, rContact, contactNormal) =
             error("Function contactStart not defined for material ", String(typeof(material)))

contactEnd(material::Modia3D.AbstractContactMaterial, obj1, obj2) =
             error("Function contactEnd not defined for material ", String(typeof(material)))

responseCalculation(material::Modia3D.AbstractContactMaterial, obj1, obj2, rContact, contactNormal, distanceWithHysteresis, time) =
             error("Function responseCalculation not defined for material ", String(typeof(material)))


# Utility functions

"""
    responseMaterial = contactStart(obj1, obj2, rContact, contactNormal)

Return a response material object at contact start.
"""
function contactStart(obj1::Object3D, obj2::Object3D, rContact::SVector{3,Float64},
                      contactNormal::SVector{3,Float64})
    if typeof(obj1.data.contactMaterial) == Solids.ElasticContactMaterial2
        name1 = obj1.data.contactMaterial.name
        name2 = obj2.data.contactMaterial.name
    else
        name1 = obj1.data.contactMaterial
        name2 = obj2.data.contactMaterial
    end
    material = Solids.getContactPairMaterial(name1, name2)
    return contactStart(material,
                        obj1, obj2, rContact, contactNormal)
end


"""
    delta_dot = normalRelativeVelocityAtContact(obj1, obj2, rContact, contactNormal)

Return the relative velocity in normal direction `contactNormal` at
contact point `rContact` of the two penetrating objects `obj1, obj2`.
"""
function normalRelativeVelocityAtContact(obj1::Object3D, obj2::Object3D,
                                         rContact::ModiaMath.Vector3D, e_n::ModiaMath.Vector3D)
  r_rel1 = rContact - obj1.r_abs
  r_rel2 = rContact - obj2.r_abs
  dynamics1 = obj1.dynamics
  dynamics2 = obj2.dynamics
  w1 = obj1.R_abs'*dynamics1.w
  w2 = obj2.R_abs'*dynamics2.w
  v1 = dynamics1.v0 + cross(w1,r_rel1)
  v2 = dynamics2.v0 + cross(w2,r_rel2)
  v_rel = v2 - v1
  delta_dot = dot(v_rel,e_n)
  return delta_dot
end
