contactStart(matPair::Shapes.NoContactPairMaterial, obj1::Object3D{F}, obj2::Object3D{F}, rContact::SVector{3,F}, contactNormal::SVector{3,F}, elasticContactReductionFactor::F) where F <: AbstractFloat =
matPair

contactStart(matPair::Shapes.ObserverContactPairMaterial, obj1::Object3D{F}, obj2::Object3D{F}, rContact::SVector{3,F}, contactNormal::SVector{3,F}, elasticContactReductionFactor::F) where F <: AbstractFloat = matPair


contactStart(matPair::Shapes.ImpulseContactPairMaterial, obj1::Object3D{F}, obj2::Object3D{F}, rContact::SVector{3,F}, contactNormal::SVector{3,F}, elasticContactReductionFactor::F) where F <: AbstractFloat =
error("contactStart is not yet implemented for ImpulseContactPairMaterial.")

contactStart(matPair::Shapes.WheelRailContactPairMaterial, obj1::Object3D{F}, obj2::Object3D{F}, rContact::SVector{3,F}, contactNormal::SVector{3,F}, elasticContactReductionFactor::F) where F <: AbstractFloat =
error("contactStart is not yet implemented for WheelRailContactPairMaterial.")



responseCalculation(material::Shapes.NoContactPairMaterial, obj1::Object3D{F}, obj2::Object3D{F}, rContact::SVector{3,F}, e_n::SVector{3,F}, s::F, time, file, sim) where F <: AbstractFloat  =
(Modia3D.ZeroVector3D(F), Modia3D.ZeroVector3D(F), Modia3D.ZeroVector3D(F), Modia3D.ZeroVector3D(F))

function responseCalculation(material::Shapes.ObserverContactPairMaterial, obj1::Object3D{F}, obj2::Object3D{F}, rContact::SVector{3,F}, e_n::SVector{3,F}, s::F, time, file, sim) where F <: AbstractFloat
  if material.printAlarm && ModiaLang.isEvent(sim)
    println("At time event ", time, " two observer objects ", Modia3D.fullName(obj1), " and ", Modia3D.fullName(obj2), " are colliding.")
  end
  return (Modia3D.ZeroVector3D(F), Modia3D.ZeroVector3D(F), Modia3D.ZeroVector3D(F), Modia3D.ZeroVector3D(F))
end

responseCalculation(material::Shapes.ImpulseContactPairMaterial, obj1::Object3D{F}, obj2::Object3D{F}, rContact::SVector{3,F}, e_n::SVector{3,F}, s::F, time, file, sim) where F <: AbstractFloat  =
(Modia3D.ZeroVector3D(F), Modia3D.ZeroVector3D(F), Modia3D.ZeroVector3D(F), Modia3D.ZeroVector3D(F))

responseCalculation(material::Shapes.WheelRailContactPairMaterial, obj1::Object3D{F}, obj2::Object3D{F}, rContact::SVector{3,F}, e_n::SVector{3,F}, s::F, time, file, sim) where F <: AbstractFloat  =
(Modia3D.ZeroVector3D(F), Modia3D.ZeroVector3D(F), Modia3D.ZeroVector3D(F), Modia3D.ZeroVector3D(F))


contactEnd(mat::Shapes.NoContactPairMaterial,obj1,obj2)::Nothing        = nothing
contactEnd(mat::Shapes.ObserverContactPairMaterial,obj1,obj2)::Nothing  = nothing
contactEnd(mat::Shapes.ImpulseContactPairMaterial,obj1,obj2)::Nothing   = nothing
contactEnd(mat::Shapes.WheelRailContactPairMaterial,obj1,obj2)::Nothing = nothing
