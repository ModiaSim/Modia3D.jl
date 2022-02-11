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


setContactSphereRadius(shape::Nothing, contactSphereRadius::Nothing, ::Type{F}) where F <: Modia3D.VarFloatType = (false, F(NaN))

setContactSphereRadius(shape::Sphere{F}, contactSphereRadius::Nothing, ::Type{F}) where F <: Modia3D.VarFloatType = (false, F(shape.diameter * 0.5) )
setContactSphereRadius(shape::Sphere{F}, contactSphereRadius::F, ::Type{F}) where F <: Modia3D.VarFloatType = (false, contactSphereRadius)


setContactSphereRadius(shape::Ellipsoid{F}, contactSphereRadius::Nothing, ::Type{F}) where F <: Modia3D.VarFloatType = (false, F(min(shape.lengthX, shape.lengthY, shape.lengthZ)*0.5) )
setContactSphereRadius(shape::Ellipsoid{F}, contactSphereRadius::F, ::Type{F}) where F <: Modia3D.VarFloatType = (false, contactSphereRadius )


setContactSphereRadius(shape::Cylinder{F}, contactSphereRadius::Nothing, ::Type{F}) where F <: Modia3D.VarFloatType = (false, F(min(shape.diameter, shape.length)*0.5) )
setContactSphereRadius(shape::Cylinder{F}, contactSphereRadius::F, ::Type{F}) where F <: Modia3D.VarFloatType = (false, contactSphereRadius )


setContactSphereRadius(shape::Capsule{F}, contactSphereRadius::Nothing, ::Type{F}) where F <: Modia3D.VarFloatType = (false, F(shape.diameter*0.5) )
setContactSphereRadius(shape::Capsule{F}, contactSphereRadius::F, ::Type{F}) where F <: Modia3D.VarFloatType = (false, contactSphereRadius )


setContactSphereRadius(shape::Cone{F}, contactSphereRadius::Nothing, ::Type{F}) where F <: Modia3D.VarFloatType = (false, F((shape.diameter + shape.topDiameter)*0.25) )
setContactSphereRadius(shape::Cone{F}, contactSphereRadius::F, ::Type{F}) where F <: Modia3D.VarFloatType = (false, contactSphereRadius )


setContactSphereRadius(shape::Box{F}, contactSphereRadius::Nothing, ::Type{F}) where F <: Modia3D.VarFloatType = (true, F(min(shape.lengthX, shape.lengthY, shape.lengthZ)*0.5) )
setContactSphereRadius(shape::Box{F}, contactSphereRadius::F, ::Type{F}) where F <: Modia3D.VarFloatType = (true, contactSphereRadius )


setContactSphereRadius(shape::Beam{F}, contactSphereRadius::Nothing, ::Type{F}) where F <: Modia3D.VarFloatType = (true, F(min(shape.length, shape.width, shape.thickness)*0.5) )
setContactSphereRadius(shape::Beam{F}, contactSphereRadius::F, ::Type{F}) where F <: Modia3D.VarFloatType = (true, contactSphereRadius )


setContactSphereRadius(shape::FileMesh, contactSphereRadius::Nothing, ::Type{F}) where F <: Modia3D.VarFloatType = (false, F(shape.shortestEdge * 0.5) )
setContactSphereRadius(shape::FileMesh, contactSphereRadius::F, ::Type{F}) where F <: Modia3D.VarFloatType = (false, contactSphereRadius )
