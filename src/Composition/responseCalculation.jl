# License for this file: MIT (expat)
# Copyright 2017-2018, DLR Institute of System Dynamics and Control
#
# This file is part of module
#   Modia3D.Composition (Modia3D/Composition/_module.jl)
#
# Functions that have to be provided from an object used for contact pair response

#contactStart(material::Modia3D.AbstractContactMaterial, obj1, obj2, rContact, contactNormal, elasticContactReductionFactor) =
#             error("Function contactStart not defined for material ", String(typeof(material)))

#contactEnd(material::Modia3D.AbstractContactMaterial, obj1, obj2) =
#             error("Function contactEnd not defined for material ", String(typeof(material)))

#responseCalculation(material::Modia3D.AbstractContactMaterial, obj1, obj2, rContact, contactNormal, distanceWithHysteresis, time) = error("Function responseCalculation not defined for material ", String(typeof(material)))


# Utility functions

"""
    responseMaterial = contactStart(obj1, obj2, rContact, contactNormal)

Return a response material object at contact start.
"""
function contactStart(obj1::Object3D{F}, obj2::Object3D{F}, rContact::SVector{3,F},
                      contactNormal::SVector{3,F}, elasticContactReductionFactor::F,
                      maximumContactDamping::F) where F <: Modia3D.VarFloatType
    material = Shapes.getContactPairMaterial(obj1, obj2)
    return contactStart(material, obj1, obj2, rContact, contactNormal, elasticContactReductionFactor, maximumContactDamping)
end


"""
    delta_dot = normalRelativeVelocityAtContact(obj1, obj2, rContact, contactNormal)

Return the relative velocity in normal direction `contactNormal` at
contact point `rContact` of the two penetrating objects `obj1, obj2`.
"""
function normalRelativeVelocityAtContact(obj1::Object3D{F}, obj2::Object3D{F},
                                         rContact::SVector{3,F}, e_n::SVector{3,F}) where F <: Modia3D.VarFloatType
  r_rel1 = rContact - obj1.r_abs
  r_rel2 = rContact - obj2.r_abs
  w1     = obj1.R_abs'*obj1.w
  w2     = obj2.R_abs'*obj2.w
  v1     = obj1.v0 + cross(w1,r_rel1)
  v2     = obj2.v0 + cross(w2,r_rel2)
  v_rel  = v2 - v1
  delta_dot = dot(v_rel,e_n)
  return delta_dot
end
