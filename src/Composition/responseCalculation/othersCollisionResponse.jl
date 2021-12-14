contactStart(matPair::Shapes.NoContactPairMaterial, obj1::Object3D, obj2::Object3D, rContact::SVector{3,Float64}, contactNormal::SVector{3,Float64}, elasticContactReductionFactor::Float64) =
matPair

contactStart(matPair::Shapes.ObserverContactPairMaterial, obj1::Object3D, obj2::Object3D, rContact::SVector{3,Float64}, contactNormal::SVector{3,Float64}, elasticContactReductionFactor::Float64) = matPair


contactStart(matPair::Shapes.ImpulseContactPairMaterial, obj1::Object3D, obj2::Object3D, rContact::SVector{3,Float64}, contactNormal::SVector{3,Float64}, elasticContactReductionFactor::Float64) =
error("contactStart is not yet implemented for ImpulseContactPairMaterial.")

contactStart(matPair::Shapes.WheelRailContactPairMaterial, obj1::Object3D, obj2::Object3D, rContact::SVector{3,Float64}, contactNormal::SVector{3,Float64}, elasticContactReductionFactor::Float64) =
error("contactStart is not yet implemented for WheelRailContactPairMaterial.")



responseCalculation(material::Shapes.NoContactPairMaterial, obj1::Object3D, obj2::Object3D, rContact::SVector{3,Float64}, e_n::SVector{3,Float64}, s::Float64, time, file, sim) =
(Modia3D.ZeroVector3D(Float64), Modia3D.ZeroVector3D(Float64), Modia3D.ZeroVector3D(Float64), Modia3D.ZeroVector3D(Float64))

function responseCalculation(material::Shapes.ObserverContactPairMaterial, obj1::Object3D, obj2::Object3D, rContact::SVector{3,Float64}, e_n::SVector{3,Float64}, s::Float64, time, file, sim)
  if material.printAlarm && ModiaLang.isEvent(sim)
    println("At time event ", time, " two observer objects ", Modia3D.fullName(obj1), " and ", Modia3D.fullName(obj2), " are colliding.")
  end
  return (Modia3D.ZeroVector3D(Float64), Modia3D.ZeroVector3D(Float64), Modia3D.ZeroVector3D(Float64), Modia3D.ZeroVector3D(Float64))
end

responseCalculation(material::Shapes.ImpulseContactPairMaterial, obj1::Object3D, obj2::Object3D, rContact::SVector{3,Float64}, e_n::SVector{3,Float64}, s::Float64, time, file, sim) =
(Modia3D.ZeroVector3D(Float64), Modia3D.ZeroVector3D(Float64), Modia3D.ZeroVector3D(Float64), Modia3D.ZeroVector3D(Float64))

responseCalculation(material::Shapes.WheelRailContactPairMaterial, obj1::Object3D, obj2::Object3D, rContact::SVector{3,Float64}, e_n::SVector{3,Float64}, s::Float64, time, file, sim) =
(Modia3D.ZeroVector3D(Float64), Modia3D.ZeroVector3D(Float64), Modia3D.ZeroVector3D(Float64), Modia3D.ZeroVector3D(Float64))


contactEnd(mat::Shapes.NoContactPairMaterial,obj1,obj2)::Nothing        = nothing
contactEnd(mat::Shapes.ObserverContactPairMaterial,obj1,obj2)::Nothing  = nothing
contactEnd(mat::Shapes.ImpulseContactPairMaterial,obj1,obj2)::Nothing   = nothing
contactEnd(mat::Shapes.WheelRailContactPairMaterial,obj1,obj2)::Nothing = nothing
