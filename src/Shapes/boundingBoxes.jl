### -------------------------------------------------------------------------------
# supportPoint(solid, shape,r_abs,R_abs,e) - Return support point of shape shape at position r_abs/R_abs
#                                   in direction of unit vector e
#                                   function is called from mpr algorithm
# boundingBox!(solid, shape,r_abs,R_abs,AABB) - Update vertex positions of smallest AABB box that contains shape

"""
    r = supportPoint(solid, shape, r_abs, R_abs, e)

Return the absolute position vector `r` from world frame to *support point* of
solid `shape`, resolved in world frame in [m]. The support point is the point of solid `shape`
that is the most extreme in direction of unit vector `e`.

# Arguments
- `shape::Modia3D.AbstractGeometry`: Solid shape object.
- `r_abs::AbstractVector`: Absolute position vector of `shape` reference frame.
- `R_abs::AbstractMatrix`: Rotation matrix to rotate world frame in `shape` reference frame.
- `e::AbstractVector`: Unit vector pointing into the desired direction.
"""
@inline supportPoint_Box(shape::Box, r_abs::SVector{3,T}, R_abs::SMatrix{3,3,T,9}, e::SVector{3,T}, collisionSmoothingRadius::T) where {T} =
            r_abs + R_abs'*supportPoint_abs_Box(shape, R_abs*e, collisionSmoothingRadius) + e*collisionSmoothingRadius
#
@inline supportPoint_Cylinder( shape::Cylinder, r_abs::SVector{3,T}, R_abs::SMatrix{3,3,T,9}, e::SVector{3,T}, collisionSmoothingRadius::T) where {T} =
            r_abs + R_abs'*supportPoint_abs_Cylinder(shape, R_abs*e) + e*collisionSmoothingRadius

@inline supportPoint_Cone(shape::Cone, r_abs::SVector{3,T}, R_abs::SMatrix{3,3,T,9}, e::SVector{3,T}, collisionSmoothingRadius::T) where {T} =
            r_abs + R_abs'*supportPoint_abs_Cone(shape, R_abs*e) + e*collisionSmoothingRadius

@inline supportPoint_Capsule(shape::Capsule, r_abs::SVector{3,T}, R_abs::SMatrix{3,3,T,9}, e::SVector{3,T}) where {T} =
            r_abs + R_abs'*supportPoint_abs_Capsule(shape, R_abs*e)

@inline supportPoint_Beam(shape::Beam, r_abs::SVector{3,T}, R_abs::SMatrix{3,3,T,9}, e::SVector{3,T}, collisionSmoothingRadius::T) where {T} =
            r_abs + R_abs'*supportPoint_abs_Beam(shape, R_abs*e) + e*collisionSmoothingRadius

@inline supportPoint_Sphere(shape::Sphere, r_abs::SVector{3,T}, R_abs::SMatrix{3,3,T,9}, e::SVector{3,T}) where {T} =
            r_abs + (shape.diameter/2.0)*e

@inline supportPoint_Ellipsoid(shape::Ellipsoid, r_abs::SVector{3,T}, R_abs::SMatrix{3,3,T,9}, e::SVector{3,T}) where {T} =
            r_abs + R_abs'*supportPoint_abs_Ellipsoid(shape, R_abs*e)

@inline supportPoint_FileMesh(shape::FileMesh, r_abs::SVector{3,T}, R_abs::SMatrix{3,3,T,9}, e::SVector{3,T}) where {T} =
            r_abs + R_abs'*supportPoint_abs_FileMesh(shape, R_abs*e)


# supportPoint_abs(shape, e_abs) - Return support point of shape shape at the shape reference
#         frame of shape in direction of unit vector e_abs (resolved in shape reference frame = object3d frame)
# see e.g. [Gino v.d. Bergen, Collision Detection in Interactive 3D Environments, 2003]
# or [G. Snethen, Xenocollide: Complex collision made simple. In Game Programming Gems 7, 2008]
# or Andrea Neumayr took ideas for computing based on those books

# [Gino v.d. Bergen, p. 135]
@inline function supportPoint_abs_Ellipsoid(shape::Ellipsoid, e_abs::SVector{3,T}) where {T}
    @inbounds begin
        shapeLengthX = T(shape.lengthX)
        shapeLengthY = T(shape.lengthY)
        shapeLengthZ = T(shape.lengthZ)
        return SVector{3,T}( (shapeLengthX/2.0)^2*e_abs[1], (shapeLengthY/2.0)^2*e_abs[2], (shapeLengthZ/2.0)^2*e_abs[3] )/norm(SVector{3,T}((shapeLengthX/2.0)*e_abs[1], (shapeLengthY/2.0)*e_abs[2], (shapeLengthZ/2.0)*e_abs[3]) )
    end
end

# [Gino v.d. Bergen, p. 135]
@inline function supportPoint_abs_Box(shape::Box, e_abs::SVector{3,T},  collisionSmoothingRadius::T) where {T}
    @inbounds begin
        shapeLengthX = T(shape.lengthX)
        shapeLengthY = T(shape.lengthY)
        shapeLengthZ = T(shape.lengthZ)
        return SVector{3,T}(
            Basics.sign_eps(e_abs[1])*(shapeLengthX/2.0-collisionSmoothingRadius),
            Basics.sign_eps(e_abs[2])*(shapeLengthY/2.0-collisionSmoothingRadius),
            Basics.sign_eps(e_abs[3])*(shapeLengthZ/2.0-collisionSmoothingRadius) )
    end
end

# [Gino v.d. Bergen, p. 136, XenoCollide, p. 168, 169]
@inline function supportPoint_abs_Cylinder(shape::Cylinder, e_abs::SVector{3,T} ) where {T}
    @inbounds begin
        neps = sqrt(eps(T))
        shapeLength = T(shape.length)
        shapeDiameter = T(shape.diameter)
        if shape.axis == 1
            enorm = norm(SVector(e_abs[2], e_abs[3]))
            if enorm <= neps
                return Basics.sign_eps(e_abs[1])*SVector{3,T}(shapeLength/2.0, 0.0, 0.0)
            else
                return Basics.sign_eps(e_abs[1])*SVector{3,T}(shapeLength/2.0, 0.0, 0.0) + SVector(0.0, (shapeDiameter/2.0)*e_abs[2], (shapeDiameter/2.0)*e_abs[3])/enorm
            end
        elseif shape.axis == 2
            enorm = norm(SVector(e_abs[3], e_abs[1]))
            if enorm <= neps
                return Basics.sign_eps(e_abs[2])*SVector{3,T}(0.0, shapeLength/2.0, 0.0)
            else
                return Basics.sign_eps(e_abs[2])*SVector{3,T}(0.0, shapeLength/2.0, 0.0) + SVector((shapeDiameter/2.0)*e_abs[1], 0.0, (shapeDiameter/2.0)*e_abs[3])/enorm
            end
        else
            enorm = norm(SVector(e_abs[1], e_abs[2]))
            if enorm <= neps
                return Basics.sign_eps(e_abs[3])*SVector{3,T}(0.0, 0.0, shapeLength/2.0)
            else
                return Basics.sign_eps(e_abs[3])*SVector{3,T}(0.0, 0.0, shapeLength/2.0) + SVector((shapeDiameter/2.0)*e_abs[1], (shapeDiameter/2.0)*e_abs[2], 0.0)/enorm
            end
        end
    end
end

# G. Hippmann: Cylinder + Sphere as bottom and top
@inline function supportPoint_abs_Capsule(shape::Capsule, e_abs::SVector{3,T}) where {T}
    @inbounds begin
        shapeLength = T(shape.length)
        shapeDiameter = T(shape.diameter)
        if shape.axis == 1
            return Basics.sign_eps(e_abs[1])*SVector{3,T}(0.5*shapeLength, 0.0, 0.0) + 0.5*shapeDiameter*e_abs
        elseif shape.axis == 2
            return Basics.sign_eps(e_abs[2])*SVector{3,T}(0.0, 0.5*shapeLength, 0.0) + 0.5*shapeDiameter*e_abs
        else
            return Basics.sign_eps(e_abs[3])*SVector{3,T}(0.0, 0.0, 0.5*shapeLength) + 0.5*shapeDiameter*e_abs
        end
    end
end

# for cone: [Gino v.d. Bergen, p. 136]]
# for frustum of a cone: A. Neumayr, G. Hippmann
@inline function supportPoint_abs_Cone(shape::Cone, e_abs::SVector{3,T}) where {T}
    @inbounds begin
        neps = sqrt(eps(T))
        baseRadius = T(shape.diameter/2.0)
        rightCone = T(shape.topDiameter) == T(0.0)
        shapeLength = T(shape.length)
        if rightCone
            sin_phi = T(baseRadius/sqrt(baseRadius^2 + shapeLength^2))  # sin of base angle
        else
            topRadius = T(shape.topDiameter/2.0)
            diffRadius = T(baseRadius - topRadius)
            sin_phi = T(diffRadius/sqrt(diffRadius^2 + shapeLength^2))  # sin of base angle
        end
        if shape.axis == 1
            value = e_abs[1] / norm(SVector{3,T}(e_abs[1], e_abs[2], e_abs[3]))
            if value >= sin_phi
                if rightCone
                    return SVector{3,T}(shapeLength, 0.0, 0.0)  # apex is support point
                else  # frustum of a cone
                    enorm = norm(SVector(e_abs[2], e_abs[3]))
                    if enorm > neps
                        return SVector(shapeLength, 0.0, 0.0) + SVector(0.0, topRadius*e_abs[2], topRadius*e_abs[3]) / enorm  # point on top circle is support point
                    else
                        return SVector{3,T}(shapeLength, 0.0, 0.0)  # top circle center is support point
                    end
                end
            else
                enorm = norm(SVector(e_abs[2], e_abs[3]))
                if enorm > neps
                    return SVector{3,T}(0.0, baseRadius*e_abs[2], baseRadius*e_abs[3]) / enorm  # point on base circle is support point
                else
                    return SVector{3,T}(0.0, 0.0, 0.0)  # base circle center is support point
                end
            end
        elseif shape.axis == 2
            value = e_abs[2] / norm(SVector(e_abs[1], e_abs[2], e_abs[3]))
            if value >= sin_phi
                if rightCone
                    return SVector{3,T}(0.0, shapeLength, 0.0)  # apex is support point
                else  # frustum of a cone
                    enorm = norm(SVector(e_abs[3], e_abs[1]))
                    if enorm > neps
                        return SVector{3,T}(0.0, shapeLength, 0.0) + SVector{3,T}(topRadius*e_abs[1], 0.0, topRadius*e_abs[3]) / enorm  # point on top circle is support point
                    else
                        return SVector{3,T}(0.0, shapeLength, 0.0)  # top circle center is support point
                    end
                end
            else
                enorm = norm(SVector(e_abs[3], e_abs[1]))
                if enorm > neps
                    return SVector{3,T}(baseRadius*e_abs[1], 0.0, baseRadius*e_abs[3]) / enorm  # point on base circle is support point
                else
                    return SVector{3,T}(0.0, 0.0, 0.0)  # base circle center is support point
                end
            end
        else
            value = e_abs[3] / norm(SVector(e_abs[1], e_abs[2], e_abs[3]))
            if value >= sin_phi
                if rightCone
                    return SVector{3,T}(0.0, 0.0, shapeLength)  # apex is support point
                else  # frustum of a cone
                    enorm = norm(SVector(e_abs[1], e_abs[2]))
                    if enorm > neps
                        return SVector{3,T}(0.0, 0.0, shapeLength) + SVector(topRadius*e_abs[1], topRadius*e_abs[2], 0.0) / enorm  # point on top circle is support point
                    else
                        return SVector{3,T}(0.0, 0.0, shapeLength)  # top circle center is support point
                    end
                end
            else
                enorm = norm(SVector(e_abs[1], e_abs[2]))
                if enorm > neps
                    return SVector{3,T}(baseRadius*e_abs[1], baseRadius*e_abs[2], 0.0) / enorm  # point on base circle is support point
                else
                    return SVector{3,T}(0.0, 0.0, 0.0)  # base circle center is support point
                end
            end
        end
    end
end

# G. Hippmann: Outer half circles of beam
@inline function supportPoint_abs_Beam(shape::Beam, e_abs::SVector{3,T})::SVector{3,T} where {T}
    @inbounds begin
        neps = sqrt(eps(T))
        shapeLength = T(shape.length)
        shapeWidth = T(shape.width)
        shapeThickness = T(shape.thickness)

        if shape.axis == 1
            enorm = norm(SVector(e_abs[1], e_abs[2]))
            if enorm <= neps
                return SVector(Basics.sign_eps(e_abs[1])*shapeLength/2.0, 0.0, Basics.sign_eps(e_abs[3])*shapeThickness/2.0)
            else
                return SVector(Basics.sign_eps(e_abs[1])*shapeLength/2.0, 0.0, Basics.sign_eps(e_abs[3])*shapeThickness/2.0) + SVector(shapeWidth/2*e_abs[1], shapeWidth/2*e_abs[2], 0.0)/enorm
            end
        elseif shape.axis == 2
            enorm = norm(SVector(e_abs[2], e_abs[3]))
            if enorm <= neps
                return SVector(Basics.sign_eps(e_abs[1])*shapeThickness/2.0, Basics.sign_eps(e_abs[2])*shapeLength/2.0, 0.0)
            else
                return SVector(Basics.sign_eps(e_abs[1])*shapeThickness/2.0, Basics.sign_eps(e_abs[2])*shapeLength/2.0, 0.0) + SVector(0.0, shapeWidth/2*e_abs[2], shapeWidth/2*e_abs[3])/enorm
            end
        else
            enorm = norm(SVector(e_abs[3], e_abs[1]))
            if enorm <= neps
                return SVector(0.0, Basics.sign_eps(e_abs[2])*shapeThickness/2.0, Basics.sign_eps(e_abs[3])*shapeLength/2.0)
            else
                return SVector(0.0, Basics.sign_eps(e_abs[2])*shapeThickness/2.0, Basics.sign_eps(e_abs[3])*shapeLength/2.0) + SVector(shapeWidth/2*e_abs[1], 0.0, shapeWidth/2*e_abs[3])/enorm
            end
        end
    end
end

# [Gino v.d. Bergen, p. 131]
@inline function supportPoint_abs_FileMesh(shape::FileMesh, e_abs::SVector{3,T}) where {T}
    e_absVec = Vector{SVector{3,Float64}}(undef, 1)
    e_absVec[1] = e_abs
    (max_value, position) = findmax(broadcast(dot, shape.objPoints, e_absVec))
    return SVector{3,T}(shape.objPoints[position])
end


#------------------------------------------------------------------------------------
# boundingBox!(..) calculates the AABB (= Axis Aligned Bounding Box) for the geometries
# it also uses support points for finding the points which are furthest away in each x,y,z - direction
# it calles supportPoint_i
"""
    boundingBox!(shape, AABB, r_abs, R_abs; tight=true, scaleFactor=0.01)

Returns the *Axis Aligned Bounding Box* of solid `shape` in argument `AABB`.

# Arguments
- `shape::Modia3D.AbstractGeometry`: Solid shape object.
- `AABB::Modia3D.BoundingBox`: On return, update AABB with the actual *Axis Aligned Bounding Box* of solid `shape`.
- `r_abs::AbstractVector`: Absolute position vector of `shape` reference frame.
- `R_abs::AbstractMatrix`: Rotation matrix to rotate world frame in `shape` reference frame.
- `tight::Bool`: If true, return the tightest AABB. If false return an AABB that is
                scaleFactor bigger than the best fitting AABB
                (for example, scaleFactor=0.1 means that the returned AABB is 10 percent bigger
                than the best fitting AABB).
- `scaleFactor::Float64`: If `tight=false`, the returned AABB is `scaleFactor` bigger than the
                          best fitting AABB.
"""

#------------------------------------------------------------------------------------
# supportPoint_i are utility functions for boundingBox!(..)
# it is based on supportPoint_abs (its nearly the same as supportPoint(..))
@inline supportPoint_i_Box(shape::Box, r_absi::Float64, R_absi::SVector{3,Float64}, isign::Int, collisionSmoothingRadius) =
               r_absi + R_absi'*supportPoint_abs_Box(shape, isign*R_absi, collisionSmoothingRadius) + isign*collisionSmoothingRadius

@inline supportPoint_i_Cylinder(shape::Cylinder, r_absi::Float64, R_absi::SVector{3,Float64}, isign::Int, collisionSmoothingRadius) =
               r_absi + R_absi'*supportPoint_abs_Cylinder(shape, isign*R_absi) + isign*collisionSmoothingRadius

@inline supportPoint_i_Cone(shape::Cone, r_absi::Float64, R_absi::SVector{3,Float64}, isign::Int, collisionSmoothingRadius) =
               r_absi + R_absi'*supportPoint_abs_Cone(shape, isign*R_absi) + isign*collisionSmoothingRadius

@inline supportPoint_i_Capsule(shape::Capsule, r_absi::Float64, R_absi::SVector{3,Float64}, isign::Int) =
               r_absi + R_absi'*supportPoint_abs_Capsule(shape, isign*R_absi)
#
@inline supportPoint_i_Beam(shape::Beam, r_absi::Float64, R_absi::SVector{3,Float64}, isign::Int, collisionSmoothingRadius) =
               r_absi + R_absi'*supportPoint_abs_Beam(shape, isign*R_absi) + isign*collisionSmoothingRadius

@inline supportPoint_i_Ellipsoid(shape::Ellipsoid, r_absi::Float64, R_absi::SVector{3,Float64}, isign::Int) =
               r_absi + dot(R_absi, supportPoint_abs_Ellipsoid(shape, isign*R_absi))

@inline supportPoint_i_FileMesh(shape::FileMesh, r_absi::Float64, R_absi::SVector{3,Float64}, isign::Int) =
               r_absi + dot(R_absi, supportPoint_abs_FileMesh(shape, isign*R_absi))
