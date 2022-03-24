# License for this file: MIT (expat)
# Copyright 2017-2018, DLR Institute of System Dynamics and Control


# Collision detection algorithm based on the MPR algorithm

struct SupportPoint{T}
    p::SVector{3,T}    # support point
    n::SVector{3,T}    # support normal unit vector
    a::SVector{3,T}    # point on shapeA
    b::SVector{3,T}    # point on shapeB
    function SupportPoint{T}(p::SVector{3,T},n::SVector{3,T},a::SVector{3,T},b::SVector{3,T}) where {T}
        new(p,n,a,b)
    end
end


function getSupportPoint(shapeA::Modia3D.Composition.Object3D{F}, shapeB::Composition.Object3D{F}, n::SVector{3,T}; scale::T=T(1.0)) where {T,F}
    a = Modia3D.supportPoint(shapeA, n)
    b = Modia3D.supportPoint(shapeB, -n)
    return SupportPoint{T}((a-b).*scale,n,a,b)
end


function barycentric(r1::SupportPoint{T}, r2::SupportPoint{T}, r3::SupportPoint{T}, r4::SupportPoint{T}) where {T}
    r21 = r2.p - r1.p
    r31 = r3.p - r1.p
    r41 = r4.p - r1.p

    # Determine normal to triangle (and test that r4 is on the triangle plane)
    n = cross(r21,r31)
    e = normalize(n)
    #@assert(abs(dot(e,r41)) <= 1e-10)

    # Determine normals to r21 and e, as well as r31 and e
    n21 = cross(r21,e)
    n31 = cross(r31,e)

    # Determine barycentric coordinates b21, b31:
    #   n21*(r41 = b21*r21 + b31*r31)    ->  n21*r21 = 0
    #   n31*(r41 = b21*r21 + b31*r31)    ->  n31*r31 = 0
    b21 = dot(n31,r41) / dot(n31,r21)
    b31 = dot(n21,r41) / dot(n21,r31)

    a = r1.a + b21*(r2.a - r1.a) + b31*(r3.a - r1.a)
    b = r1.b + b21*(r2.b - r1.b) + b31*(r3.b - r1.b)
    r4 = SupportPoint{T}(r4.p, r4.n, a, b)


    ### only works if r4.n goes through zero
    #=
    b1 = dot(cross(r2.p,r3.p),r4.n)
    b2 = dot(cross(r3.p,r1.p),r4.n)
    b3 = dot(cross(r1.p,r2.p),r4.n)
    b_ges = b1 + b2 + b3

    r4.a .= (b1*r1.a + b2*r2.a + b3*r3.a) / b_ges
    r4.b .= (b1*r1.b + b2*r2.b + b3*r3.b) / b_ges
    =#
end


###########      Phase 1, Minkowski Portal Refinement      ###################
@inline getCentroid(obj::Composition.Object3D)::SVector{3,Modia3D.MPRFloatType} = SVector{3,Modia3D.MPRFloatType}(obj.r_abs + obj.R_abs'*obj.centroid)


# checks if centers of shapeA and shapeB are overlapping
# belongs to construction of r0
function checkCentersOfShapesOverlapp(r0::SupportPoint{T}, shapeA::Composition.Object3D{F}, shapeB::Composition.Object3D{F}) where {T,F}
    if norm(r0.p) <= Modia3D.nepsType(T)
        error("MPR: Too large penetration (prerequisite of MPR violated). Centers are overlapping. Look at $(Modia3D.fullName(shapeA)) and $(Modia3D.fullName(shapeB)).")
    end
end


function checkIfShapesArePlanar(r0::SupportPoint{T},r1::SupportPoint{T},r2::SupportPoint{T},n2::SVector{3,T},
                                shapeA::Composition.Object3D{F},shapeB::Composition.Object3D{F}) where {T,F}
    r3 = SupportPoint{T}

    # r3 is in the direction of plane normal that contains triangle r0-r1-r2
    n3 = cross(r1.p-r0.p, r2.p-r0.p)
    neps = Modia3D.nepsType(T)
    # the triangle r0-r1-r2 has degenerated into a line segment
    if norm(n3) <= neps
        # change search direction for r2
        # because we are still interested in distances if shapes are not intersecting
        n2 = -n2
        r2 = getSupportPoint(shapeA, shapeB, n2)
        n3 = cross(r1.p-r0.p, r2.p-r0.p)  # new normal to the triangle plane (r0-r1-r2_new)
        if norm(n3) <= neps
            # Shape is purely planar. Computing the shortest distance for a planar shape
            # requires an MPR 2D algorithm (using lines instead of triangles as portals).
            # However, this is not implemented and therefore the shortest distance cannot be computed
            error("MPR: Shapes are planar and MPR2D is not supported. norm(cross(r1.p-r0.p, r2.p-r0.p)). Look at $(Modia3D.fullName(shapeA)) and $(Modia3D.fullName(shapeB)).")
        end
    end

    if dot(n3,r0.p) >= 0.0
        n3 = -n3
    end

    # check if portal triangle r1-r2-r3 has degenerated into a line segment <--> points r1,r2,r3 are on the same line
    r3 = getSupportPoint(shapeA, shapeB, Basics.normalizeVector(n3))
    n3b = cross(r2.p-r1.p, r3.p-r1.p)
    if norm(n3b) <= neps
        # change search direction for r3
        r3 = getSupportPoint(shapeA, shapeB, -r3.n)
        if norm(cross(r2.p-r1.p, r3.p-r1.p)) <= neps
            # Shape is purely planar. Computing the shortest distance for a planar shape
            # requires an MPR 2D algorithm (using lines instead of triangles as portals).
            # However, this is not implemented and therefore the shortest distance cannot be computed
            error("MPR: Shapes are planar and MPR2D is not supported. r1, r2, r3 are on the same ray. norm(cross(r2.p-r1.p, r3.p-r1.p)) <= neps. Look at $(Modia3D.fullName(shapeA)) and $(Modia3D.fullName(shapeB)).")
        end
    end

    return (r2, r3, n2, n3)
end


###########      Phase 2, Minkowski Portal Refinement      ###################
# loop around to "ensure" the tetrahedron r0,r1,r2 and r3 encloses the origin
# stimmt so nicht wirklich, muss ich nochmal nachlesen!!!
# Der Ursprung muss nicht enthalten sein!!!
function tetrahedronEncloseOrigin(r0::SupportPoint{T}, r1::SupportPoint{T},
            r2::SupportPoint{T}, r3::SupportPoint{T},
            niter_max::Int64,
            shapeA::Composition.Object3D{F}, shapeB::Composition.Object3D{F}, scale::T) where {T,F}
    r1org = r1
    r2org = r2
    r3org = r3
    aux = Modia3D.ZeroVector3D(T)
    neps = Modia3D.nepsType(T)
    success = false
    for i in 1:niter_max
        aux = cross(r1.p-r0.p,r3.p-r0.p)
        if dot(aux,r0.p) > neps
            r2 = r3
            r3 = getSupportPoint(shapeA,shapeB,Basics.normalizeVector(aux), scale=scale)
            continue
        end
        aux = cross(r3.p-r0.p,r2.p-r0.p)
        if dot(aux,r0.p) > neps
            r1 = r3
            r3 = getSupportPoint(shapeA,shapeB,Basics.normalizeVector(aux), scale=scale)
            continue
        end
        success = true
        break
    end
    if success != true
        error("MPR (phase 2): Max. number of iterations (mprIterMax = $niter_max) is reached. Please, increase mprIterMax. Look at shapes: $(Modia3D.fullName(shapeA)) and $(Modia3D.fullName(shapeB)).")
    end
    return (r1, r2, r3)
end


###########      Phase 3, Minkowski Portal Refinement      ###################
# construction of r4
function constructR4(r0::SupportPoint{T},r1::SupportPoint{T},r2::SupportPoint{T},r3::SupportPoint{T},
                     shapeA::Composition.Object3D{F},shapeB::Composition.Object3D{F}, scale::T) where {T,F}
    r4 = SupportPoint{T}
    n4 = cross(r2.p-r1.p, r3.p-r1.p)
    neps = Modia3D.nepsType(T)
    if norm(n4) <= neps
        r3 = getSupportPoint(shapeA, shapeB, -r3.n, scale=scale) # change search direction
        n4 = cross(r2.p-r1.p, r3.p-r1.p)
        if norm(n4) <= neps
            # Shape is purely planar. Computing the shortest distance for a planar shape
            # requires an MPR 2D algorithm (using lines instead of triangles as portals).
            # However, this is not implemented and therefore the shortest distance cannot be computed
            error("MPR: Shapes are planar and MPR2D is not supported. norm(n4). Look at $(Modia3D.fullName(shapeA)) and $(Modia3D.fullName(shapeB)).")
        end
    end
    if dot(n4,r0.p) >= 0.0
        n4 = -n4
    end
    r4 = getSupportPoint(shapeA, shapeB, Basics.normalizeVector(n4), scale=scale)

    return (r3, r4, n4)
end

# computes twice a parallelepipedial product (Spatprodukt)
isNextPortal(r0::SupportPoint, r1::SupportPoint, r2::SupportPoint, r4::SupportPoint) = dot( cross( (r2.p-r0.p), (r4.p-r0.p) ), r0.n ) <= 0.0 &&
                            dot( cross( (r4.p-r0.p), (r1.p-r0.p) ), r0.n ) <= 0.0


# Construction of three baby tetrahedrons and decide which one will replace
# the current tetrahedron r1r2 - r2r3 - r3r1 - current tetrahedron edges
# leads to 3 new tetrahedrons:
#   r1r2r4
#   r2r3r4
#   r3r1r4
function createBabyTetrahedrons(r0::SupportPoint{T}, r1::SupportPoint{T}, r2::SupportPoint{T}, r3::SupportPoint{T}, r4::SupportPoint{T}) where {T}
    nextPortal = true
    if isNextPortal(r0,r1,r2,r4)
        r3 = r4
    elseif isNextPortal(r0,r2,r3,r4)
        r1 = r4
    elseif isNextPortal(r0,r3,r1,r4)
        r2 = r4
    else
        nextPortal = false # the signs of all tried combinations of parallelepipedial products are not negative
    end
    return (nextPortal, r1, r2, r3)
end


scaleVector(scale, ri) = ri.*scale


function skalarization(r0::SupportPoint{T}, r1::SupportPoint{T}, r2::SupportPoint{T}, r3::SupportPoint{T}) where {T}
    x = maximum([abs.(r0.p) abs.(r1.p) abs.(r2.p) abs.(r3.p)])
    scale = 1/x
    r0 = SupportPoint{T}(scaleVector(scale, r0.p), r0.n, r0.a, r0.b)
    r1 = SupportPoint{T}(scaleVector(scale, r1.p), r1.n, r1.a, r1.b)
    r2 = SupportPoint{T}(scaleVector(scale, r2.p), r2.n, r2.a, r2.b)
    r3 = SupportPoint{T}(scaleVector(scale, r3.p), r3.n, r3.a, r3.b)
    return (r0, r1, r2, r3, scale)
end

function finalTC2(r1::SupportPoint{T}, r2::SupportPoint{T}, r3::SupportPoint{T}, r4::SupportPoint{T}) where {T}
    #println("TC 2")
    #if !analyzeFinalPortal(r1.p, r2.p, r3.p, r4.p)
        # error("shapeA = ", shapeA, " shapeB = ", shapeB)
    #end
    distance = -dot(r4.n, r4.p)
    r4 = barycentric(r1, r2, r3, r4)
    return distance, r1, r2, r3, r4
end


function finalTC3(r0::SupportPoint{T}, r1::SupportPoint{T}, r2::SupportPoint{T}, r3::SupportPoint{T}, r4::SupportPoint{T}) where {T}
    #println("TC 3")

    #doesRayIntersectPortal(r1.p,r2.p,r3.p, r4.p)
    #if !analyzeFinalPortal(r1.p, r2.p, r3.p, r4.p)
        # error("shapeA = ", shapeA, " shapeB = ", shapeB)
    #end

    (r4p, distance) = signedDistanceToPortal(r0.p, r1.p, r2.p, r3.p)
    r4 = SupportPoint{T}(r4p, r4.n, r4.a, r4.b)
    r4 = barycentric(r1, r2, r3, r4)
    return distance, r1, r2, r3, r4
end


function terminateMPR(r0::SupportPoint{T}, r1::SupportPoint{T}, r2::SupportPoint{T},
                      r3::SupportPoint{T}, r4::SupportPoint{T}, isTC2::Bool, isTC3::Bool)::Tuple{T, SupportPoint{T}, SupportPoint{T}, SupportPoint{T}, SupportPoint{T}} where {T}
    if isTC2
        (distance,r1,r2,r3,r4) = finalTC2(r1, r2, r3, r4)
        return distance, r1, r2, r3, r4
    end
    if isTC3
        (distance,r1,r2,r3,r4) = finalTC3(r0, r1, r2, r3, r4)
        return distance, r1, r2, r3, r4
    end
end


function phase3(r0::SupportPoint{T}, r1::SupportPoint{T}, r2::SupportPoint{T}, r3::SupportPoint{T}, niter_max::Int64, tol_rel::T, shapeA::Composition.Object3D{F}, shapeB::Composition.Object3D{F}, scale::T) where {T,F}
    r1org = r1
    r2org = r2
    r3org = r3
    new_tol = T(42.0)
    isTC2 = false
    isTC3 = false
    r1_new::SupportPoint = r0
    r2_new::SupportPoint = r0
    r3_new::SupportPoint = r0
    r4_new::SupportPoint = r0
    for i in 1:niter_max
        ### Phase 3.1: construct r4 ###
        # Find support point using the tetrahedron face
        (r3,r4,n4) = constructR4(r0,r1,r2,r3,shapeA,shapeB, scale)


        ### Phase 3.2: check if r4 is close to the origin ###
        # check if the new point r4 is already on the plane of the triangle r1,r2,r3,
        # we're already as close as we can get to the origin
        TC2 = norm(cross(r4.p,r4.n))    # TC2
        TC3 = abs(dot(r4.p-r1.p, r4.n)) # TC3
        ## TERMINATION CONDITION 2 ##
        if TC2 < tol_rel
            return finalTC2(r1,r2,r3,r4)

        ## TERMINATION CONDITION 3 ##
        elseif TC3 < tol_rel
            return finalTC3(r0, r1, r2, r3, r4)
        else
            if TC2 < new_tol
                new_tol = TC2
                isTC2 = true
                isTC3 = false
                r1_new = r1
                r2_new = r2
                r3_new = r3
                r4_new = r4
            end
            if TC3 < new_tol
                new_tol = TC3
                isTC2 = false
                isTC3 = true
                r1_new = r1
                r2_new = r2
                r3_new = r3
                r4_new = r4
            end
        end

        #### Phase 3.3: construct baby tetrahedrons with r1,r2,r3,r4 and create a new portal ###
        # Construction of three baby tetrahedrons
        (nextPortal, r1,r2,r3) = createBabyTetrahedrons(r0,r1,r2,r3,r4)

        if !nextPortal # createBabyTetrahedrons failed
            @warn("MPR (phase 3): Numerical issues with distance computation between $(Modia3D.fullName(shapeA)) and $(Modia3D.fullName(shapeB)). tol_rel increased locally for this computation to $new_tol.")
            return terminateMPR(r0, r1_new, r2_new, r3_new, r4_new, isTC2, isTC3)
        end
    end
    @warn("MPR (phase 3): Max. number of iterations (mprIterMax = $niter_max) is reached. Please, increase mprIterMax. tol_rel increased locally for this computation to $new_tol. Look at shapes $(Modia3D.fullName(shapeA)) and $(Modia3D.fullName(shapeB)).")
    return terminateMPR(r0, r1_new, r2_new, r3_new, r4_new, isTC2, isTC3)
end

# MPR - Minkowski Portal Refinement algorithm
# construction of points r0 is in the interior of Minkowski Difference and points r1,r2,r3,r4 are on the boundary of Minkowski Difference
# Phase 1
#   Construct a tetrahedron such one point is in the interior r0 and three points r1,r2,r3 are on the boundary of the Minkowski Difference
#   Construction of r0 and of initial portal triangle points r1, r2, r3
#   Phase 1.1: construction of r0
#   Phase 1.2: construction of initial r1
#     Termination Condition 1
#   Phase 1.3: construction of initial r2
#   Phase 1.4: construction of initial r3
# Phase 2
#   make sure the origin ray passes through portal triangle r1-r2-r3, otherwise it is not sure that the portal's normal points in the right direction
#   if the origin ray passes through the portal, --> the normal of all other triangles of the tetrahedron point away from the origin ray
#   replace one of the points in the portal with a new one until the ray from r0 passes through the portal
#   loop around to "ensure" the tetrahedron r0,r1,r2 and r3 encloses the origin
# Phase 3
#   search iteratively in the direction of the portal's normal
#   construct a new portal, such the origin passes still the portal, and it is closer to the surface of the Minkowski Difference
#   than the old portal
#   Phase 3.1: construct r4,
#   Phase 3.2: check if r4 is close to the origin
#     Termination Condition 2
#     Termination Condition 3
#   Phase 3.3: construct baby tetrahedrons with r1,r2,r3,r4 and create a new portal
function mprGeneral(ch::Composition.ContactDetectionMPR_handler{T,F}, shapeA::Composition.Object3D{F}, shapeB::Modia3D.Composition.Object3D{F}) where {T,F}
    tol_rel = ch.tol_rel
    niter_max = ch.niter_max
    neps = Modia3D.nepsType(T)
    r0 = SupportPoint{T}
    r1 = SupportPoint{T}
    r2 = SupportPoint{T}

    ###########      Phase 1, Minkowski Portal Refinement      ###################
    # Construction of r0 and initial portal triangle points r1, r2, r3

    ### Phase 1.1: construction of r0 ###
    # r0 is a point inside the Minkowski Difference (centroidA-centroidB)
    # the direction of the origin ray r0 is -r0.p
    centroidA = getCentroid(shapeA)
    centroidB = getCentroid(shapeB)
    if isnan(centroidA[1]) || isnan(centroidB[1]) ||
       isnan(centroidA[2]) || isnan(centroidB[2]) ||
       isnan(centroidA[3]) || isnan(centroidB[3])
        error("MPR: One of the absolute position or translation is NaN. Look at $(Modia3D.fullName(shapeA)): r_abs = $(shapeA.r_abs), R_abs = $(shapeA.R_abs) and $(Modia3D.fullName(shapeB)): r_abs = $(shapeB.r_abs), R_abs = $(shapeB.R_abs).")
    end
    r0 = SupportPoint{T}(centroidA-centroidB, -(centroidA-centroidB), SVector{3,T}(0.0,0.0,0.0), SVector{3,T}(0.0,0.0,0.0))
    # check if centers of shapes are overlapping
    checkCentersOfShapesOverlapp(r0, shapeA, shapeB)

    ### Phase 1.2: construction of initial r1 ###
    # r1 is the farthest point in the direction to the origin
    # first portal point should point in the direction of the origin ray (-r0.p)
    # therefore choose search direction -r0.p
    r1 = getSupportPoint(shapeA, shapeB, Basics.normalizeVector(r0.n))

    ### Phase 1.3: construction of initial r2 ###
    n2 = cross(r0.n, r1.p)
    n2abs = norm(n2)
    ## TERMINATION CONDITION 1 ##
    if n2abs <= neps
        # r0 || r1, r0 ist parallel zu r1
        # centers of shapes (r0.p and r1.p) are on the same line segment
        # the origin of Minkowski Difference is on the line from r0 with direction r1-r0
        # e.g. any collision/or distance between two spheres
        #println("TC 1")
        distance = dot(r1.p,normalize(r0.p))
        return (distance, r1.a, r1.b, r1.n, false, Modia3D.ZeroVector3D(T), Modia3D.ZeroVector3D(T), Modia3D.ZeroVector3D(T), Modia3D.ZeroVector3D(T), Modia3D.ZeroVector3D(T), Modia3D.ZeroVector3D(T) )
    else
        # normalize n2
        n2 = n2/n2abs
    end
    r2 = getSupportPoint(shapeA, shapeB, n2)

    ### Phase 1.4: construction of initial r3 ###
    # r3 is in the direction of plane normal that contains triangle r0-r1-r2
    (r2, r3, n2, n3) = checkIfShapesArePlanar(r0, r1, r2, n2, shapeA, shapeB)

    (r0,r1,r2,r3,scale) = skalarization(r0,r1,r2,r3)


    ###########      Phase 2, Minkowski Portal Refinement      ###################
    # loop around to "ensure" the tetrahedron r0,r1,r2 and r3 encloses the origin
    (r1,r2,r3) = tetrahedronEncloseOrigin(r0,r1,r2,r3,niter_max,shapeA,shapeB, scale)
    # doesRayIntersectPortal(r1.p,r2.p,r3.p, r0.p) # Portal.A, Portal.B, Portal.C, point,

    ###########      Phase 3, Minkowski Portal Refinement      ###################
    (distance, r1, r2, r3, r4) = phase3(r0, r1, r2, r3, niter_max, tol_rel, shapeA, shapeB, scale)
    return ( distance, r4.a, r4.b, r4.n, true, r1.a, r1.b, r2.a, r2.b, r3.a, r3.b )
end


function distanceTwoSpheres(ch::Composition.ContactDetectionMPR_handler{T,F}, shapeA::Composition.Object3D{F}, shapeB::Modia3D.Composition.Object3D{F},
    sphereA::Shapes.Sphere{F}, sphereB::Shapes.Sphere{F}) where {T,F}
    neps = Modia3D.nepsType(T)
    radiusA = T(sphereA.diameter*0.5)
    radiusB = T(sphereB.diameter*0.5)
    centroidSphereA = getCentroid(shapeA)
    centroidSphereB = getCentroid(shapeB)
    n = centroidSphereB - centroidSphereA
    distanceCentroids = norm(n)
    if distanceCentroids <= neps
        error("Centers of two spheres are overlapping. Look at $(Modia3D.fullName(shapeA)) and $(Modia3D.fullName(shapeB)).")
    else
        normal = n/distanceCentroids
    end
    distance = distanceCentroids - radiusA - radiusB
    contactPointShapeA = centroidSphereA + normal*radiusA
    contactPointShapeB = centroidSphereB - normal*radiusB
    return (distance, contactPointShapeA, contactPointShapeB, normal, false, Modia3D.ZeroVector3D(T), Modia3D.ZeroVector3D(T), Modia3D.ZeroVector3D(T), Modia3D.ZeroVector3D(T), Modia3D.ZeroVector3D(T), Modia3D.ZeroVector3D(T) )
end


function mpr(ch::Composition.ContactDetectionMPR_handler{T,F}, shapeA::Composition.Object3D{F}, shapeB::Modia3D.Composition.Object3D{F}) where {T,F}
    shapeKindA = shapeA.shapeKind
    shapeKindB = shapeB.shapeKind

    if shapeKindA == Modia3D.SphereKind && shapeKindB == Modia3D.SphereKind
        sphereA::Shapes.Sphere{F} = shapeA.shape
        sphereB::Shapes.Sphere{F} = shapeB.shape
        (distance, contactPoint1, contactPoint2, normal, supportPointsDefined,
        support1A, support1B, support2A, support2B, support3A, support3B) = distanceTwoSpheres(ch, shapeA, shapeB, sphereA, sphereB)
    else
        (distance, contactPoint1, contactPoint2, normal, supportPointsDefined,
        support1A, support1B, support2A, support2B, support3A, support3B) = mprGeneral(ch, shapeA, shapeB)

        if Modia3D.contactPointIsLocallyBijectiveToNormal(shapeA) && Modia3D.contactPointIsLocallyBijectiveToNormal(shapeB)
            algebraicSign = sign(distance)
            contactPoint1 = Modia3D.supportPoint(shapeA, normal)
            contactPoint2 = Modia3D.supportPoint(shapeB, -normal)
            distance = algebraicSign * norm(contactPoint2 - contactPoint1)
        elseif Modia3D.contactPointIsLocallyBijectiveToNormal(shapeA) && !Modia3D.contactPointIsLocallyBijectiveToNormal(shapeB)
            contactPoint1 = Modia3D.supportPoint(shapeA, normal)
            contactPoint2 = contactPoint1 + distance*normal
        elseif !Modia3D.contactPointIsLocallyBijectiveToNormal(shapeA) && Modia3D.contactPointIsLocallyBijectiveToNormal(shapeB)
            contactPoint2 = Modia3D.supportPoint(shapeB, -normal)
            contactPoint1 = contactPoint2 + distance*-normal
        end
    end
    return (F(distance), SVector{3,F}(contactPoint1), SVector{3,F}(contactPoint2), SVector{3,F}(normal), supportPointsDefined,
    SVector{3,F}(support1A), SVector{3,F}(support1B), SVector{3,F}(support2A), SVector{3,F}(support2B), SVector{3,F}(support3A), SVector{3,F}(support3B) )
end
