function supportPoint(obj::Composition.Object3D, e::SVector{3,Float64})
    shapeKind = obj.shapeKind
    solid::Modia3D.Solid = obj.feature
    collisionSmoothingRadius = solid.collisionSmoothingRadius

    if shapeKind == Modia3D.SphereKind
        #sphere::Modia3D.Sphere = obj.shape
        return Modia3D.supportPoint_Sphere(obj.shape, obj.r_abs, obj.R_abs, e)
    elseif shapeKind == Modia3D.EllipsoidKind
        #ellipsoid::Modia3D.Ellipsoid = obj.shape
        return Modia3D.supportPoint_Ellipsoid(obj.shape, obj.r_abs, obj.R_abs, e)
    elseif shapeKind == Modia3D.BoxKind
        #box::Modia3D.Box = obj.shape
        return Modia3D.supportPoint_Box(obj.shape, obj.r_abs, obj.R_abs, e, collisionSmoothingRadius)
    elseif shapeKind == Modia3D.CylinderKind
        #cylinder::Modia3D.Cylinder = obj.shape
        return Modia3D.supportPoint_Cylinder(obj.shape, obj.r_abs, obj.R_abs, e, collisionSmoothingRadius)
    elseif shapeKind == Modia3D.ConeKind
        #cone::Modia3D.Cone = obj.shape
        return Modia3D.supportPoint_Cone(obj.shape, obj.r_abs, obj.R_abs, e, collisionSmoothingRadius)
    elseif shapeKind == Modia3D.CapsuleKind
        #capsule::Modia3D.Capsule = obj.shape
        return Modia3D.supportPoint_Capsule(obj.shape, obj.r_abs, obj.R_abs, e)
    elseif shapeKind == Modia3D.BeamKind
        #beam::Modia3D.Beam = obj.shape
        return Modia3D.supportPoint_Beam(obj.shape, obj.r_abs, obj.R_abs, e, collisionSmoothingRadius)
    elseif shapeKind == Modia3D.FileMeshKind
        #fileMesh::Modia3D.FileMesh = obj.shape
        return Modia3D.supportPoint_FileMesh(obj.shape, obj.r_abs, obj.R_abs, e)
    else
        error("not supported shape for support points")
    end
end


function boundingBox!(obj::Composition.Object3D, AABB::Basics.BoundingBox; tight::Bool=true, scaleFactor::Float64=0.01)
    shapeKind = obj.shapeKind
    solid::Modia3D.Solid = obj.feature
    collisionSmoothingRadius = solid.collisionSmoothingRadius

    if shapeKind == Modia3D.SphereKind
        sphere::Modia3D.Sphere = obj.shape
        @inbounds begin
            r = sphere.diameter/2
            if tight    # best fitting AABB which is possible
                AABB.x_min = obj.r_abs[1] - r
                AABB.x_max = obj.r_abs[1] + r
                AABB.y_min = obj.r_abs[2] - r
                AABB.y_max = obj.r_abs[2] + r
                AABB.z_min = obj.r_abs[3] - r
                AABB.z_max = obj.r_abs[3] + r
            else
                # scales each axis with scaleFactor 0.1 means 10 percent bigger than best fitting AABB and so on
                @assert(scaleFactor > 0.0)
                scale = r*scaleFactor/2
                AABB.x_min = obj.r_abs[1] - r - scale
                AABB.x_max = obj.r_abs[1] + r + scale
                AABB.y_min = obj.r_abs[2] - r - scale
                AABB.y_max = obj.r_abs[2] + r + scale
                AABB.z_min = obj.r_abs[3] - r - scale
                AABB.z_max = obj.r_abs[3] + r + scale
            end
            return AABB
        end
    elseif shapeKind == Modia3D.EllipsoidKind
        ellipsoid::Modia3D.Ellipsoid = obj.shape
        xmin = Modia3D.supportPoint_i_Ellipsoid(ellipsoid, obj.r_abs[1], SVector(obj.R_abs[:,1]), -1)
        xmax = Modia3D.supportPoint_i_Ellipsoid(ellipsoid, obj.r_abs[1], SVector(obj.R_abs[:,1]), +1)
        ymin = Modia3D.supportPoint_i_Ellipsoid(ellipsoid, obj.r_abs[2], SVector(obj.R_abs[:,2]), -1)
        ymax = Modia3D.supportPoint_i_Ellipsoid(ellipsoid, obj.r_abs[2], SVector(obj.R_abs[:,2]), +1)
        zmin = Modia3D.supportPoint_i_Ellipsoid(ellipsoid, obj.r_abs[3], SVector(obj.R_abs[:,3]), -1)
        zmax = Modia3D.supportPoint_i_Ellipsoid(ellipsoid, obj.r_abs[3], SVector(obj.R_abs[:,3]), +1)
    elseif shapeKind == Modia3D.BoxKind
        box::Modia3D.Box = obj.shape
        xmin = Modia3D.supportPoint_i_Box(box, obj.r_abs[1], SVector(obj.R_abs[:,1]), -1, collisionSmoothingRadius)
        xmax = Modia3D.supportPoint_i_Box(box, obj.r_abs[1], SVector(obj.R_abs[:,1]), +1, collisionSmoothingRadius)
        ymin = Modia3D.supportPoint_i_Box(box, obj.r_abs[2], SVector(obj.R_abs[:,2]), -1, collisionSmoothingRadius)
        ymax = Modia3D.supportPoint_i_Box(box, obj.r_abs[2], SVector(obj.R_abs[:,2]), +1, collisionSmoothingRadius)
        zmin = Modia3D.supportPoint_i_Box(box, obj.r_abs[3], SVector(obj.R_abs[:,3]), -1, collisionSmoothingRadius)
        zmax = Modia3D.supportPoint_i_Box(box, obj.r_abs[3], SVector(obj.R_abs[:,3]), +1, collisionSmoothingRadius)
    elseif shapeKind == Modia3D.CylinderKind
        cylinder::Modia3D.Cylinder = obj.shape
        xmin = Modia3D.supportPoint_i_Cylinder(cylinder, obj.r_abs[1], SVector(obj.R_abs[:,1]), -1, collisionSmoothingRadius)
        xmax = Modia3D.supportPoint_i_Cylinder(cylinder, obj.r_abs[1], SVector(obj.R_abs[:,1]), +1, collisionSmoothingRadius)
        ymin = Modia3D.supportPoint_i_Cylinder(cylinder, obj.r_abs[2], SVector(obj.R_abs[:,2]), -1, collisionSmoothingRadius)
        ymax = Modia3D.supportPoint_i_Cylinder(cylinder, obj.r_abs[2], SVector(obj.R_abs[:,2]), +1, collisionSmoothingRadius)
        zmin = Modia3D.supportPoint_i_Cylinder(cylinder, obj.r_abs[3], SVector(obj.R_abs[:,3]), -1, collisionSmoothingRadius)
        zmax = Modia3D.supportPoint_i_Cylinder(cylinder, obj.r_abs[3], SVector(obj.R_abs[:,3]), +1, collisionSmoothingRadius)
    elseif shapeKind == Modia3D.ConeKind
        cone::Modia3D.Cone = obj.shape
        xmin = Modia3D.supportPoint_i_Cone(cone, obj.r_abs[1], SVector(obj.R_abs[:,1]), -1, collisionSmoothingRadius)
        xmax = Modia3D.supportPoint_i_Cone(cone, obj.r_abs[1], SVector(obj.R_abs[:,1]), +1, collisionSmoothingRadius)
        ymin = Modia3D.supportPoint_i_Cone(cone, obj.r_abs[2], SVector(obj.R_abs[:,2]), -1, collisionSmoothingRadius)
        ymax = Modia3D.supportPoint_i_Cone(cone, obj.r_abs[2], SVector(obj.R_abs[:,2]), +1, collisionSmoothingRadius)
        zmin = Modia3D.supportPoint_i_Cone(cone, obj.r_abs[3], SVector(obj.R_abs[:,3]), -1, collisionSmoothingRadius)
        zmax = Modia3D.supportPoint_i_Cone(cone, obj.r_abs[3], SVector(obj.R_abs[:,3]), +1, collisionSmoothingRadius)
    elseif shapeKind == Modia3D.CapsuleKind
        capsule::Modia3D.Capsule = obj.shape
        xmin = Modia3D.supportPoint_i_Capsule(capsule, obj.r_abs[1], SVector(obj.R_abs[:,1]), -1)
        xmax = Modia3D.supportPoint_i_Capsule(capsule, obj.r_abs[1], SVector(obj.R_abs[:,1]), +1)
        ymin = Modia3D.supportPoint_i_Capsule(capsule, obj.r_abs[2], SVector(obj.R_abs[:,2]), -1)
        ymax = Modia3D.supportPoint_i_Capsule(capsule, obj.r_abs[2], SVector(obj.R_abs[:,2]), +1)
        zmin = Modia3D.supportPoint_i_Capsule(capsule, obj.r_abs[3], SVector(obj.R_abs[:,3]), -1)
        zmax = Modia3D.supportPoint_i_Capsule(capsule, obj.r_abs[3], SVector(obj.R_abs[:,3]), +1)
    elseif shapeKind == Modia3D.BeamKind
        beam::Modia3D.Beam = obj.shape
        xmin = Modia3D.supportPoint_i_Beam(beam, obj.r_abs[1], SVector(obj.R_abs[:,1]), -1, collisionSmoothingRadius)
        xmax = Modia3D.supportPoint_i_Beam(beam, obj.r_abs[1], SVector(obj.R_abs[:,1]), +1, collisionSmoothingRadius)
        ymin = Modia3D.supportPoint_i_Beam(beam, obj.r_abs[2], SVector(obj.R_abs[:,2]), -1, collisionSmoothingRadius)
        ymax = Modia3D.supportPoint_i_Beam(beam, obj.r_abs[2], SVector(obj.R_abs[:,2]), +1, collisionSmoothingRadius)
        zmin = Modia3D.supportPoint_i_Beam(beam, obj.r_abs[3], SVector(obj.R_abs[:,3]), -1, collisionSmoothingRadius)
        zmax = Modia3D.supportPoint_i_Beam(beam, obj.r_abs[3], SVector(obj.R_abs[:,3]), +1, collisionSmoothingRadius)
    elseif shapeKind == Modia3D.FileMeshKind
        fileMesh::Modia3D.FileMesh = obj.shape
        xmin = Modia3D.supportPoint_i_FileMesh(fileMesh, obj.r_abs[1], SVector(obj.R_abs[:,1]), -1)
        xmax = Modia3D.supportPoint_i_FileMesh(fileMesh, obj.r_abs[1], SVector(obj.R_abs[:,1]), +1)
        ymin = Modia3D.supportPoint_i_FileMesh(fileMesh, obj.r_abs[2], SVector(obj.R_abs[:,2]), -1)
        ymax = Modia3D.supportPoint_i_FileMesh(fileMesh, obj.r_abs[2], SVector(obj.R_abs[:,2]), +1)
        zmin = Modia3D.supportPoint_i_FileMesh(fileMesh, obj.r_abs[3], SVector(obj.R_abs[:,3]), -1)
        zmax = Modia3D.supportPoint_i_FileMesh(fileMesh, obj.r_abs[3], SVector(obj.R_abs[:,3]), +1)
    else
        error("boundingBox! not supported for shape")
    end

    if tight    # best fitting AABB which is possible
        AABB.x_min = xmin
        AABB.x_max = xmax
        AABB.y_min = ymin
        AABB.y_max = ymax
        AABB.z_min = zmin
        AABB.z_max = zmax
    else
        # scales each axis with scaleFactor 0.1 means 10 percent bigger than best fitting AABB and so on
        @assert(scaleFactor > 0.0)
        scale = max(abs(xmax - xmin)*scaleFactor/2,abs(ymax - ymin)*scaleFactor/2,abs(zmax - zmin)*scaleFactor/2)
        AABB.x_min = xmin - scale
        AABB.x_max = xmax + scale
        AABB.y_min = ymin - scale
        AABB.y_max = ymax + scale
        AABB.z_min = zmin - scale
        AABB.z_max = zmax + scale
    end
    return AABB
end
