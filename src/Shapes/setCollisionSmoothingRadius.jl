setCollisionSmoothingRadius(shape, collisionSmoothingRadius::F) where F <: Modia3D.VarFloatType = F(NaN)


function setCollisionSmoothingRadius(shape::Box{F}, collisionSmoothingRadius::F) where F <: Modia3D.VarFloatType
    @assert(collisionSmoothingRadius >= 0.0)
    collisionSmoothingRadius2 = min(collisionSmoothingRadius, F(0.1)*min(shape.lengthX, shape.lengthY, shape.lengthZ)) # at most 10% of the smallest edge length
    if collisionSmoothingRadius2 < collisionSmoothingRadius
      println("collisionSmoothingRadius in Box changed from $collisionSmoothingRadius to $collisionSmoothingRadius2")
    end
    @assert(shape.lengthX >= 2*collisionSmoothingRadius2)
    @assert(shape.lengthY >= 2*collisionSmoothingRadius2)
    @assert(shape.lengthZ >= 2*collisionSmoothingRadius2)
    return collisionSmoothingRadius2
end

function setCollisionSmoothingRadius(shape::Cylinder{F}, collisionSmoothingRadius::F) where F <: Modia3D.VarFloatType
    @assert(collisionSmoothingRadius >= 0.0)
    collisionSmoothingRadius = min(collisionSmoothingRadius, F(0.1)*min(shape.diameter, shape.length))  # at most 10% of the smallest edge length
    return collisionSmoothingRadius
end

function setCollisionSmoothingRadius(shape::Cone{F}, collisionSmoothingRadius::F) where F <: Modia3D.VarFloatType
    @assert(collisionSmoothingRadius >= 0.0)
    collisionSmoothingRadius2 = min(collisionSmoothingRadius, F(0.1)*min(shape.diameter, shape.length)) # at most 10% of the smallest edge length
    return collisionSmoothingRadius2
end

function setCollisionSmoothingRadius(shape::Beam{F}, collisionSmoothingRadius::F) where F <: Modia3D.VarFloatType
    @assert(collisionSmoothingRadius >= 0.0)
    collisionSmoothingRadius2 = min(collisionSmoothingRadius, F(0.1)*min(shape.length, shape.width, shape.thickness)) # at most 10% of the smallest edge length
    return collisionSmoothingRadius2
end
