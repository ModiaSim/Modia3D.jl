const MinBVLength = 1.0e-6  # min. axial length of BVs

struct ContactSurfaceFace{F <: Modia3D.VarFloatType}
    vertexIndices::SVector{3,UInt64}
    baryCentre::SVector{3,F}
    normalVector::SVector{3,F}
    area::F
end

struct BVTree{F <: Modia3D.VarFloatType}
    minBound::SVector{3,F}
    maxBound::SVector{3,F}
    faceIndex::UInt64
    lowerBV::Union{BVTree{F}, Nothing}
    upperBV::Union{BVTree{F}, Nothing}
end

struct ContactSurface{F <: Modia3D.VarFloatType}
    vertexCoordinates::Vector{SVector{3,F}}
    faces::Vector{ContactSurfaceFace{F}}

    faceBVTree::Union{BVTree{F}, Nothing}
    penLineBVTree::Union{BVTree{F}, Nothing}

    function ContactSurface{F}(vertices::Vector{SVector{3,Float64}}, faces::Vector{SVector{3,Int64}},
                               flipNormals::Bool, generateFaceBVTree::Bool, generatePenLineBVTree::Bool, maxPenetration::F) where F <: Modia3D.VarFloatType

        vertexCoordinates = Vector{SVector{3,F}}()
        for vertex in vertices
            push!(vertexCoordinates, SVector{3,F}(vertex))
        end

        surfaceFaces = Vector{ContactSurfaceFace{F}}()
        for face in faces
            if flipNormals
                vertexIndices = SVector{3,UInt64}(face[3], face[2], face[1])
            else
                vertexIndices = SVector{3,UInt64}(face[1], face[2], face[3])
            end
            (baryCentre, normalVector, area) = accessoryFaceData(vertexCoordinates, vertexIndices)
            surfaceFace = ContactSurfaceFace{F}(vertexIndices, baryCentre, normalVector, area)
            push!(surfaceFaces, surfaceFace)
        end

        faceBVTree = nothing
        if generateFaceBVTree
            faceBVTree = createBVTree(vertexCoordinates, surfaceFaces, F(0.0))
        end

        penLineBVTree = nothing
        if generatePenLineBVTree
            penLineBVTree = createBVTree(vertexCoordinates, surfaceFaces, maxPenetration)
        end

        new(vertexCoordinates, surfaceFaces, faceBVTree, penLineBVTree)
    end
end

@enum CEStatus begin
    inactive  # CE is not active
    active    # CE is active
    invalid   # CE penetration is invalid (wrong target surface orientation)
end

mutable struct ContactElement{F <: Modia3D.VarFloatType}
    status::CEStatus            # status
    position::SVector{3,F}      # position vector E->CE w.r.t. E
    normalVector::SVector{3,F}  # normal vector w.r.t. E
    penetration::F              # penetration
    area::F                     # area
    normalForce::F              # normal force
end

mutable struct ContactState{F <: Modia3D.VarFloatType}
    A_EF::SMatrix{3,3,F,9}                              # transformation matrix: E_v = A_EF * F_v
    r_EF::SVector{3,F}                                  # position vector E->F w.r.t. E
    w_EF::SVector{3,F}                                  # ang. velocity vector E->F w.r.t. E
    v_EF::SVector{3,F}                                  # trans. velocity vector E->F w.r.t. E
    surfaceFMaxBBDirections::Vector{SVector{3,Int}}     # unit cube F vertex coordinates at max. boundaries w.r.t. E
    surfaceFVertexCoordinates::Vector{SVector{3,F}}     # cached surface F vertex coordinates w.r.t. E
    surfaceFVertexCoordinatesCache::Vector{Bool}        # surfaceFVertexCoordinates cache status
    surfaceFBaryCentres::Vector{SVector{3,F}}           # cached surface F barycentres w.r.t. E
    surfaceFBaryCentresCache::Vector{Bool}              # surfaceFBaryCentres cache status
    surfaceFNormalVectors::Vector{SVector{3,F}}         # cached surface F normal vectors w.r.t. E
    surfaceFNormalVectorsCache::Vector{Bool}            # surfaceFNormalVectors cache status
    surfaceEContactElements::Vector{ContactElement{F}}  # surface E contact elements
    surfaceFContactElements::Vector{ContactElement{F}}  # surface F contact elements
    workVertexCoordinatesE::Vector{SVector{3,F}}        # work array: face of surface E vertex coordinates
    workVertexCoordinatesF::Vector{SVector{3,F}}        # work array: face of surface E vertex coordinates
    numBVChecks::UInt64                                 # BV check counter
    numPenChecks::UInt64                                # penetration check counter
    numPenetrations::UInt64                             # penetration counter
    maxPenetration::F                                   # max. penetration
end


function accessoryFaceData(surfaceVertexCoordinates::Vector{SVector{3,F}}, faceVertexIndices::SVector{3,UInt64}) where F <: Modia3D.VarFloatType

    v1 = surfaceVertexCoordinates[faceVertexIndices[1]]
    v2 = surfaceVertexCoordinates[faceVertexIndices[2]]
    v3 = surfaceVertexCoordinates[faceVertexIndices[3]]

    v12 = v2 - v1
    v13 = v3 - v1
    v12xv13 = cross(v12, v13)

    baryCentre = SVector{3,F}((v1 + v2 + v3)/F(3.0))

    normalVector = SVector{3,F}(normalize(v12xv13))

    area = F(0.5)*norm(v12xv13)

    return (baryCentre, normalVector, area)

end


"""
    force = PolygonalContactModel(; obj1::Object3D, obj2::Object3D,
        task::Int = 0,
        surfaceE_FlipFaces::Bool = false,
        surfaceF_FlipFaces::Bool = false,
        maxPenetration::Real = 0.0,
        bodyE_YoungsModulus::Real = 0.0,
        bodyF_YoungsModulus::Real = 0.0,
        bodyE_PoissonsRatio::Real = 0.0,
        bodyF_PoissonsRatio::Real = 0.0,
        bodyE_LayerDepth::Real = 0.0,
        bodyF_LayerDepth::Real = 0.0,
        expansionDampingMode::Int = 0,
        compressionArealDampingFactor::Real = 0.0,
        expansionArealDampingFactor::Real = 0.0,
        dampingTransitionMode::Int = 0,
        dampingTransitionDepth::Real = 0.0,
        frictionCoefficient::Real = 0.0,
        frictionRegularisationVelocity::Real = 0.0 )

Return a `force` acting as [Polygonal Contact Model](http://pcm.hippmann.org/)
(PCM) between `obj1::`[`Object3D`](@ref) (PCM body E) and
`obj2::`[`Object3D`](@ref) (PCM body F). Both, `obj1` and `obj2` must be of
feature `Solid` with shape type `FileMesh`.

# Arguments

- `task` defines the operation mode of PCM:
  - 1 = 2-pass base/target
  - 2 = base/target = contact surface E/F
  - 3 = base/target = contact surface F/E
- `surfaceE_FlipFaces` flips the faces (orientation) of contact surface E
- `surfaceF_FlipFaces` flips the faces (orientation) of contact surface F
- `bodyE_YoungsModulus` [N/m^2] defines the Young's modulus of the elasic foundation of body E (`bodyE_YoungsModulus` = 0 -> rigid)
- `bodyF_YoungsModulus` [N/m^2] defines the Young's modulus of the elasic foundation of body F (`bodyF_YoungsModulus` = 0 -> rigid)
- `bodyE_PoissonsRatio` [-] defines the Poison's ratio of the elasic foundation of body E (0 < `bodyE_PoissonsRatio` < 0.45)
- `bodyF_PoissonsRatio` [-] defines the Poison's ratio of the elasic foundation of body F (0 < `bodyF_PoissonsRatio` < 0.45)
- `bodyE_LayerDepth` [m] defines the depth of the elasic foundation of body E (`bodyE_LayerDepth` = 0 -> rigid)
- `bodyF_LayerDepth` [m] defines the depth of the elasic foundation of body F (`bodyF_LayerDepth` = 0 -> rigid)
- `maxPenetration` defines the max. penetration to be considered
- `expansionDampingMode` [-] defines the mode of the expansion damping:
  - 0 = no tensile contact force
  - 1 = no expansion damping
  - 2 = full expansion damping
- `compressionArealDampingFactor` [Ns/m^3] defines the areal damping factor (``F_d :=`` `compressionArealDampingFactor` ``* v_n * A``) of the elastic foundation during compression (approaching bodies)
- `expansionArealDampingFactor` [Ns/m^3] defines the areal damping factor (``F_d :=`` `expansionArealDampingFactor` * ``v_n * A``) of the elastic foundation during expansion (departing bodies)
- `dampingTransitionMode` defines the operation mode of the damping transition:
  - 0: none
  - 1: linear transition of depth `dampingTransitionDepth`
- `dampingTransitionDepth` [m] defines the depth of the damping transition
- `frictionCoefficient` [-] defines the sliding friction coefficient (``F_t :=`` `frictionCoefficient` ``* F_n``) of the contact
- `frictionRegularizationVelocity` [m/s] defines the upper limit of the static friction regularization (``v_t <`` `frictionRegularisationVelocity` -> ``F_t := v_t/`` `frictionRegularizationVelocity` ``*`` `frictionCoefficient` ``* F_n``)

# Results

- `numBVChecks` is the number of bounding volume checks.
- `numPenChecks` is the number of penetration checks.
- `numPenetrations` is the number of penetrations.
- `numContactElements` is the number of contact elements.
- `contactPatchArea` is the contact patch area.
- `weightedPenetration` is the area-weighted penetration.
- `maxPenetration` is the maximum penetration.
- `minNormalVelocity` is the minimum normal velocity.
- `weightedNormalVelocity` ist the area-weighted normal velocity.
- `maxNormalVelocity` is the maximum normal velocity.
- `minTangentialVelocity` is the minimum tangential velocity.
- `weightedTangentialVelocity` is the area-weighted tangential velocity.
- `maxTangentialVelocity` is the maximum tangential velocity.
- `weightedPressure` is the area-weighted normal contact pressure.
- `maxPressure` is the maximum normal contact pressure.
- `weightedTraction` is the area-weighted contact traction.
- `maxTraction` is the maximum contact traction.
- `weightedPosition` is the area-weighted contact position vector with
  respect to `obj1`, resolved in `obj1`.
- `weightedNormal` is the area-weighted contact normal vector, resolved
  in `obj1`.
- `forceVector` is the force vector acting on `obj2`, resolved in `obj1`.
  At `obj1` the same force vector is applied in inverse direction. In
  addition a compensation torque is applied at `obj1` to satisfy torque
  balance.
- `torqueVector` is the torque vector acting on `obj2`, resolved in
  `obj1`. At `obj1` the same torque vector is applied in inverse
  direction.

# Notes on usage

## Base-target surfaces and discretization
For optimal robustness and efficiency, the concept of PCM's contact detection should be regarded. It is based on a base-target approach, i.e. one of the contact surfaces is defined as base and the other one as target. Based on this assignment the PCM contact detection encompasses the following steps:

1. All polygons of the base surface for which the barycenter is located on the inner side of the target surface are processed as contact elements.
2. For each contact element the intersection point of the base polygon normal at the barycenter and the target surface is computed.
3. The distance between the base polygon barycenter and the intersection point is used as penetration of the contact element.
4. A spring-damper force law is applied along the penetration line of the contact element.
5. A regularized friction force law is applied perpendicular to the penetration line of the contact element.
6. The total contact force is calculated as the sum of all contact element forces.

As a consequence of this procedure, the polygonal discretization of the base surface defines the scanning resolution of the contact patch. Just like every discretization it should be chosen with care for an optimal trade-off between accuracy and efficiency of the contact evaluation. In particular surfaces with large polygons, which e.g. may occur in case of planar regions, are not suitable as base surface. This problem might be solved by remeshing the surface accordingly or by interchanging the base-target assignment. (As a matter of principle, large polygons are not an issue for target surfaces.)

## Sharp edges
The PCM discretization of the contact patch can lead to bad results in case of sharp edges of the target surface, i.e. angles of 90° or more between adjacent polygons. The problem is sketched in 2D in the figure regarding two boxes sliding along each other.

![PCM sharp edges](../../resources/images/pcm_scanning_problem.svg)

In case a) there are sharp edges of the target surface resulting in discontinuous penetrations at the contact elements of the base surface: When the front edge of the upper box passes a contact element, its penetration suddenly jumps to a non-zero value, and at the rear edge penetration jumps back to zero again. This leads to unphysical behavior because potential energy conservation of the elastic force law is violated.

This problem can be addressed in two different ways: Sharp edges of the target surface might be sanitized by chamfering (case b)) or the base-target assignment might be interchanged (case c)), such that discontinuous penetrations do not occur anymore.

## Maximum penetration
The maximum penetration `maxPenetration` is an important parameter which must be chosen with care. Since larger penetrations are not detected by the algorithm, the value must be greater than the maximum penetration of the whole simulation. Note that iterative solvers with step-size control can induce contact evaluations with larger penetrations than those visible in the results. As a consequence, some safety reserve (e.g. 50 %) should be provided and a step-size limit (e.g. 10 ms) should be defined. PCM prints the maximum penetration at the end of the simulation so that the chosen parameter value can be checked after simulation.

Note that using larger maximum penetration values than necessary does not affect the results but causes additional computation effort caused by unnecessarily large bounding volumes and handling of invalid penetration line intersections.

## Stiffness and damping parameters
For elastic normal contact forces PCM implements the elastic foundation model. The idea is to cover the contacting bodies with thin elastic layers which do not propagate deformation or stress in tangential direction. Contact elements are used to discretize the contact patch. Therefore, the normal direction and the area of the base polygon represent a subdivision of a combined elastic layer in which constant normal pressure is assumed. Thus, the elastic force law of a contact element is equivalent to a linear spring with the stiffness given by the product of the contact element area and the elastic layer stiffness. According to the elastic foundation model, the layer stiffness of each body is defined by a quotient of a material parameter (elastic modulus) and the thickness `LayerDepth` of the layer.

An advantage of this approach is that common material properties (Young's modulus and Poisson's radio) and the easily comprehensible layer depth are sufficient to define the elastic contact properties. However, in most applications there is no comparatively soft layer on the surfaces and finding suitable values for the layer depth can be difficult.

Note that the layer depth appears in the denominator of the layer stiffness, i.e. its reciprocal acts like a scaling factor and therefore has great influence of the contact stiffness. So, if the elastic contact properties are crucial for the system dynamics, experiments may be used to tune the layer stiffness.

The same holds for the damping characteristics. Deduced from the stiffness force law, the damping force of a PCM contact element is calculated with an area related layer damping coefficient `ArealDampingFactor`. Similar to the layer depth, this parameter cannot be specified directly according to material and geometry data, but requires test simulations to find plausible or verified values. However, this is typical for damping and restitution coefficients in multibody system dynamics. Note that `dampingTransitionMode` and `expansionDampingMode` allow to avoid discontinuous and tensile contact forces.

"""
mutable struct PolygonalContactModel{F <: Modia3D.VarFloatType} <: Modia3D.AbstractForceElement

    path::String

    obj1::Object3D{F}
    obj2::Object3D{F}

    task::Int
    surfaceE_FlipFaces::Bool
    surfaceF_FlipFaces::Bool
    bodyE_YoungsModulus::F
    bodyF_YoungsModulus::F
    bodyE_PoissonsRatio::F
    bodyF_PoissonsRatio::F
    bodyE_LayerDepth::F
    bodyF_LayerDepth::F
    maxPenetration::F
    expansionDampingMode::Int
    compressionArealDampingFactor::F
    expansionArealDampingFactor::F
    dampingTransitionMode::Int
    dampingTransitionDepth::F
    frictionCoefficient::F
    frictionRegularizationVelocity::F

    name::String
    DetEPS::F
    DotEPS::F

    combinedLayerStiffness::F
    penetrationRatio::F

    surfaceE::ContactSurface{F}
    surfaceF::ContactSurface{F}
    state::ContactState{F}

    numBVChecksResultIndex::Int
    numPenChecksResultIndex::Int
    numPenetrationsResultIndex::Int
    numContactElementsResultIndex::Int
    contactPatchAreaResultIndex::Int
    weightedPenetrationResultIndex::Int
    maxPenetrationResultIndex::Int
    minNormalVelocityResultIndex::Int
    weightedNormalVelocityResultIndex::Int
    maxNormalVelocityResultIndex::Int
    minTangentialVelocityResultIndex::Int
    weightedTangentialVelocityResultIndex::Int
    maxTangentialVelocityResultIndex::Int
    weightedPressureResultIndex::Int
    maxPressureResultIndex::Int
    weightedTractionResultIndex::Int
    maxTractionResultIndex::Int
    weightedPositionResultIndex::Int
    weightedNormalResultIndex::Int
    forceVectorResultIndex::Int
    torqueVectorResultIndex::Int

    function PolygonalContactModel{F}(; path::String = "", obj1::Object3D{F}, obj2::Object3D{F},
        task::Int = 0,
        surfaceE_FlipFaces::Bool = false,
        surfaceF_FlipFaces::Bool = false,
        bodyE_YoungsModulus::Real = F(0.0),
        bodyF_YoungsModulus::Real = F(0.0),
        bodyE_PoissonsRatio::Real = F(0.0),
        bodyF_PoissonsRatio::Real = F(0.0),
        bodyE_LayerDepth::Real = F(0.0),
        bodyF_LayerDepth::Real = F(0.0),
        maxPenetration::Real = F(0.0),
        expansionDampingMode::Int = 0,
        compressionArealDampingFactor::Real = F(0.0),
        expansionArealDampingFactor::Real = F(0.0),
        dampingTransitionMode::Int = 0,
        dampingTransitionDepth::Real = F(0.0),
        frictionCoefficient::Real = F(0.0),
        frictionRegularizationVelocity::Real = F(0.0)) where F <: Modia3D.VarFloatType

        bodyE_YoungsModulus            = Modia3D.convertAndStripUnit(F, u"N/m^2"  , bodyE_YoungsModulus)
        bodyF_YoungsModulus            = Modia3D.convertAndStripUnit(F, u"N/m^2"  , bodyF_YoungsModulus)
        bodyE_LayerDepth               = Modia3D.convertAndStripUnit(F, u"m"      , bodyE_LayerDepth)
        bodyF_LayerDepth               = Modia3D.convertAndStripUnit(F, u"m"      , bodyF_LayerDepth)
        maxPenetration                 = Modia3D.convertAndStripUnit(F, u"m"      , maxPenetration)
        compressionArealDampingFactor  = Modia3D.convertAndStripUnit(F, u"N*s/m^3", compressionArealDampingFactor)
        expansionArealDampingFactor    = Modia3D.convertAndStripUnit(F, u"N*s/m^3", expansionArealDampingFactor)
        dampingTransitionDepth         = Modia3D.convertAndStripUnit(F, u"m"      , dampingTransitionDepth)
        frictionRegularizationVelocity = Modia3D.convertAndStripUnit(F, u"m/s"    , frictionRegularizationVelocity)

        name = "PCM " * Modia3D.fullName(obj1) * " - " * Modia3D.fullName(obj2)
        DetEPS = 5e3*eps(F)  # determinant limit for robust detection of inner triangle points in function calculatePenetration
        DotEPS = 5e3*eps(F)  # scalar product limit for robust detection of perpendicular triangle normals in function calculatePenetration

        return new(path, obj1, obj2, task, surfaceE_FlipFaces, surfaceF_FlipFaces, bodyE_YoungsModulus, bodyF_YoungsModulus, bodyE_PoissonsRatio, bodyF_PoissonsRatio, bodyE_LayerDepth, bodyF_LayerDepth, maxPenetration, expansionDampingMode, compressionArealDampingFactor, expansionArealDampingFactor, dampingTransitionMode, dampingTransitionDepth, frictionCoefficient, frictionRegularizationVelocity, name, DetEPS, DotEPS)
    end

end
PolygonalContactModel(; kwargs...) = PolygonalContactModel{Float64}(; kwargs...)


function initializeForceElement(model::Modia.InstantiatedModel{F,TimeType}, force::PolygonalContactModel{F}) where {F <: Modia3D.VarFloatType, TimeType <: AbstractFloat}

    force.obj1.hasForceElement = true
    force.obj2.hasForceElement = true

    checkParameters!(force)
    generateSurfaces!(force)
    createState!(force)

    force.numBVChecksResultIndex                = Modia.new_w_segmented_variable!(model, force.path*".numBVChecks"               , Int(0))
    force.numPenChecksResultIndex               = Modia.new_w_segmented_variable!(model, force.path*".numPenChecks"              , Int(0))
    force.numPenetrationsResultIndex            = Modia.new_w_segmented_variable!(model, force.path*".numPenetrations"           , Int(0))
    force.numContactElementsResultIndex         = Modia.new_w_segmented_variable!(model, force.path*".numContactElements"        , Int(0))
    force.contactPatchAreaResultIndex           = Modia.new_w_segmented_variable!(model, force.path*".contactPatchArea"          , F(0), "m^2")
    force.weightedPenetrationResultIndex        = Modia.new_w_segmented_variable!(model, force.path*".weightedPenetration"       , F(0), "m")
    force.maxPenetrationResultIndex             = Modia.new_w_segmented_variable!(model, force.path*".maxPenetration"            , F(0), "m")
    force.minNormalVelocityResultIndex          = Modia.new_w_segmented_variable!(model, force.path*".minNormalVelocity"         , F(0), "m/s")
    force.weightedNormalVelocityResultIndex     = Modia.new_w_segmented_variable!(model, force.path*".weightedNormalVelocity"    , F(0), "m/s")
    force.maxNormalVelocityResultIndex          = Modia.new_w_segmented_variable!(model, force.path*".maxNormalVelocity"         , F(0), "m/s")
    force.minTangentialVelocityResultIndex      = Modia.new_w_segmented_variable!(model, force.path*".minTangentialVelocity"     , F(0), "m/s")
    force.weightedTangentialVelocityResultIndex = Modia.new_w_segmented_variable!(model, force.path*".weightedTangentialVelocity", F(0), "m/s")
    force.maxTangentialVelocityResultIndex      = Modia.new_w_segmented_variable!(model, force.path*".maxTangentialVelocity"     , F(0), "m/s")
    force.weightedPressureResultIndex           = Modia.new_w_segmented_variable!(model, force.path*".weightedPressure"          , F(0), "Pa")
    force.maxPressureResultIndex                = Modia.new_w_segmented_variable!(model, force.path*".maxPressure"               , F(0), "Pa")
    force.weightedTractionResultIndex           = Modia.new_w_segmented_variable!(model, force.path*".weightedTraction"          , F(0), "Pa")
    force.maxTractionResultIndex                = Modia.new_w_segmented_variable!(model, force.path*".maxTraction"               , F(0), "Pa")
    force.weightedPositionResultIndex           = Modia.new_w_segmented_variable!(model, force.path*".weightedPosition"          , SVector{3,F}(0, 0, 0), "m")
    force.weightedNormalResultIndex             = Modia.new_w_segmented_variable!(model, force.path*".weightedNormal"            , SVector{3,F}(0, 0, 0))
    force.forceVectorResultIndex                = Modia.new_w_segmented_variable!(model, force.path*".forceVector"               , SVector{3,F}(0, 0, 0), "N")
    force.torqueVectorResultIndex               = Modia.new_w_segmented_variable!(model, force.path*".torqueVector"              , SVector{3,F}(0, 0, 0), "N*m")

    return nothing

end

function evaluateForceElement(model::Modia.InstantiatedModel{F,TimeType}, force::PolygonalContactModel{F}, time::TimeType) where {F <: Modia3D.VarFloatType, TimeType <: AbstractFloat}

    # initialise contact evaluation
    initialiseEvaluation!(force)

    if force.task == 1 || force.task == 2
        # detect penetrations and generate contact elements for base/target surface E/F
        generateContactElements!(force, true)
    end

    if force.task == 1 || force.task == 3
        # detect penetrations and generate contact elements for base/target surface F/E
        generateContactElements!(force, false)
    end

    (force12, torque12, results) = calculateForceTorque!(force)

    applyFrameForcePair!(force.obj2, force.obj1, force12; frameCoord=force.obj1)
    applyFrameTorquePair!(force.obj2, force.obj1, torque12; frameCoord=force.obj1)

    if Modia.storeResults(model)
        Modia.copy_w_segmented_value_to_result(model, force.numBVChecksResultIndex, Int(results[1]))
        Modia.copy_w_segmented_value_to_result(model, force.numPenChecksResultIndex, Int(results[2]))
        Modia.copy_w_segmented_value_to_result(model, force.numPenetrationsResultIndex, Int(results[3]))
        Modia.copy_w_segmented_value_to_result(model, force.numContactElementsResultIndex, Int(results[4]))
        Modia.copy_w_segmented_value_to_result(model, force.contactPatchAreaResultIndex, results[5])
        Modia.copy_w_segmented_value_to_result(model, force.weightedPenetrationResultIndex, results[6])
        Modia.copy_w_segmented_value_to_result(model, force.maxPenetrationResultIndex, results[7])
        Modia.copy_w_segmented_value_to_result(model, force.minNormalVelocityResultIndex, results[8])
        Modia.copy_w_segmented_value_to_result(model, force.weightedNormalVelocityResultIndex, results[9])
        Modia.copy_w_segmented_value_to_result(model, force.maxNormalVelocityResultIndex, results[10])
        Modia.copy_w_segmented_value_to_result(model, force.minTangentialVelocityResultIndex, results[11])
        Modia.copy_w_segmented_value_to_result(model, force.weightedTangentialVelocityResultIndex, results[12])
        Modia.copy_w_segmented_value_to_result(model, force.maxTangentialVelocityResultIndex, results[13])
        Modia.copy_w_segmented_value_to_result(model, force.weightedPressureResultIndex, results[14])
        Modia.copy_w_segmented_value_to_result(model, force.maxPressureResultIndex, results[15])
        Modia.copy_w_segmented_value_to_result(model, force.weightedTractionResultIndex, results[16])
        Modia.copy_w_segmented_value_to_result(model, force.maxTractionResultIndex, results[17])
        Modia.copy_w_segmented_value_to_result(model, force.weightedPositionResultIndex, SVector{3,F}(results[18:20]))
        Modia.copy_w_segmented_value_to_result(model, force.weightedNormalResultIndex, SVector{3,F}(results[21:23]))
        Modia.copy_w_segmented_value_to_result(model, force.forceVectorResultIndex, -force12)
        Modia.copy_w_segmented_value_to_result(model, force.torqueVectorResultIndex, -torque12)
    end

    return nothing

end

function terminateForceElement(force::PolygonalContactModel{F}) where F <: Modia3D.VarFloatType

    if force.state.maxPenetration > F(0.9)*force.maxPenetration
        Printf.@printf "%s: Max. penetration = %.2e [m] is close to user-defined limit = %.2e [m].\n" force.name force.state.maxPenetration force.maxPenetration
    else
        Printf.@printf "%s: Max. penetration = %.2e [m].\n" force.name force.state.maxPenetration
    end

    return nothing

end


function checkParameters!(force::PolygonalContactModel{F}) where F <: Modia3D.VarFloatType

    if force.task >= 1 && force.task <= 3
        if force.maxPenetration <= F(0.0)
            error("$(force.name): Max. penetration = $(force.maxPenetration) [m] is not positive.")
        end
        if force.bodyE_YoungsModulus < F(0.0)
            error("$(force.name): Young's modulus of Body E = $(force.bodyE_YoungsModulus) [Pa] is negative.")
        end
        if force.bodyF_YoungsModulus < F(0.0)
            error("$(force.name): Young's modulus of Body F = $(force.bodyF_YoungsModulus) [Pa] is negative.")
        end
        if force.bodyE_YoungsModulus == F(0.0) && force.bodyF_YoungsModulus == F(0.0)
            error("$(force.name): Both Young's moduli are zero.")
        end
        if force.bodyE_PoissonsRatio < F(0.0) || force.bodyE_PoissonsRatio > F(0.5)
            error("$(force.name): Poisson's ratio of Body E = $(force.bodyE_PoissonsRatio) [-] is out of range.")
        end
        if force.bodyF_PoissonsRatio < F(0.0) || force.bodyF_PoissonsRatio > F(0.5)
            error("$(force.name): Poisson's ratio of Body F = $(force.bodyF_PoissonsRatio) [-] is out of range.")
        end
        if force.bodyE_LayerDepth < F(0.0)
            error("$(force.name): Layer depth of Body E = $(force.bodyE_LayerDepth) [m] is negative.")
        end
        if force.bodyF_LayerDepth < F(0.0)
            error("$(force.name): Layer depth of Body F = $(force.bodyF_LayerDepth) [m] is negative.")
        end
        if force.bodyE_LayerDepth == F(0.0) && force.bodyF_LayerDepth == F(0.0)
            error("$(force.name): Both layer depths are zero.")
        end
        if force.bodyE_YoungsModulus > F(0.0) && force.bodyE_LayerDepth > F(0.0)
            if ( force.bodyE_PoissonsRatio > F(0.45) )
                error("$(force.name): Poisson's ratio of Body E = $(force.bodyE_PoissonsRatio) [-] is out of range [0; 0.45] of elastic foundation model.")
            end
            layerStiffnessE = (F(1.0) - force.bodyE_PoissonsRatio) / (F(1.0) + force.bodyE_PoissonsRatio) / (F(1.0) - F(2.0) * force.bodyE_PoissonsRatio)
            layerStiffnessE = layerStiffnessE * force.bodyE_YoungsModulus / force.bodyE_LayerDepth
        else
            layerStiffnessE = F(0.0)
        end
        if force.bodyF_YoungsModulus > F(0.0) && force.bodyF_LayerDepth > F(0.0)
            if ( force.bodyF_PoissonsRatio > F(0.45) )
                error("$(force.name): Poisson's ratio of Body F = $(force.bodyF_PoissonsRatio) [-] is out of range [0; 0.45] of elastic foundation model.")
            end
            layerStiffnessF = (F(1.0) - force.bodyF_PoissonsRatio) / (F(1.0) + force.bodyF_PoissonsRatio) / (F(1.0) - F(2.0) * force.bodyF_PoissonsRatio)
            layerStiffnessF = layerStiffnessF * force.bodyF_YoungsModulus / force.bodyF_LayerDepth
        else
            layerStiffnessF = F(0.0)
        end

        # calculate combined layer stiffness
        if ( layerStiffnessE * layerStiffnessF != F(0.0) )
            force.combinedLayerStiffness = layerStiffnessE * layerStiffnessF / (layerStiffnessE + layerStiffnessF)
        elseif ( layerStiffnessE != F(0.0) )
            force.combinedLayerStiffness = layerStiffnessE
        else
            force.combinedLayerStiffness = layerStiffnessF
        end

        # calculate ratio of penetrations
        force.penetrationRatio = layerStiffnessF / (layerStiffnessE + layerStiffnessF)

        if force.compressionArealDampingFactor < F(0.0)
            error("$(force.name): Compression areal damping factor = $(force.compressionArealDampingFactor) [Ns/m] is negative.")
        end
        if force.expansionDampingMode == 1
            force.expansionArealDampingFactor = F(0.0)
        elseif force.expansionArealDampingFactor < F(0.0)
            error("$(force.name): Expansion areal damping factor = $(force.expansionArealDampingFactor) [Ns/m] is negative.")
        end
        if force.dampingTransitionMode == 1
            if force.dampingTransitionDepth < F(0.0)
                error("$(force.name): Damping transition depth = $(force.dampingTransitionDepth) [m] is negative.")
            elseif force.dampingTransitionDepth < F(1.0e-10)
                force.dampingTransitionDepth = F(1.0e-4)
                warn("$(force.name): Set damping transition depth := $(force.dampingTransitionDepth) [m].")
            end
        else
            force.dampingTransitionDepth = F(0.0)
        end
        if force.frictionCoefficient < F(0.0)
            error("$(force.name): Friction coefficient = $(force.frictionCoefficient) [-] is negative.")
        end
        if force.frictionRegularizationVelocity < F(1.0e-10)
            force.frictionRegularizationVelocity = F(1.0e-3)
            warn("$(force.name): Set friction regularisation velocity := $(force.frictionRegularizationVelocity) [m/s].")
        end
    else
        error("$(force.name): Operation mode = $(force.task) is out of range.")
    end

    return nothing

end

function generateSurfaces!(force::PolygonalContactModel{F}) where F <: Modia3D.VarFloatType

    if typeof(force.obj1.feature) <: Modia3D.Shapes.Solid
        if typeof(force.obj1.shape) <: Modia3D.Shapes.FileMesh
            generatePenLineBVTree = force.task == 1 || force.task == 2
            generateFaceBVTree = force.task == 1 || force.task == 3
            force.surfaceE = ContactSurface{F}(force.obj1.shape.objPoints, force.obj1.shape.facesIndizes,
                                               force.surfaceE_FlipFaces, generateFaceBVTree, generatePenLineBVTree, force.maxPenetration)
        else
            error("$(force.name): Shape of object 1 is not of type FileMesh.")
        end
    else
        error("$(force.name): Object 1 is not of type Solid.")
    end

    if typeof(force.obj2.feature) <: Modia3D.Shapes.Solid
        if typeof(force.obj2.shape) <: Modia3D.Shapes.FileMesh
            generatePenLineBVTree = force.task == 1 || force.task == 3
            generateFaceBVTree = force.task == 1 || force.task == 2
            force.surfaceF = ContactSurface{F}(force.obj2.shape.objPoints, force.obj2.shape.facesIndizes,
                                               force.surfaceF_FlipFaces, generateFaceBVTree, generatePenLineBVTree, force.maxPenetration)
        else
            error("$(force.name): Shape of object 2 is not of type FileMesh.")
        end
    else
        error("$(force.name): Object 2 is not of type Solid.")
    end

    return nothing

end

function createBVTree(surfaceVertexCoordinates::Vector{SVector{3,F}}, surfaceFaces::Vector{ContactSurfaceFace{F}}, maxPenetration::F) where F <: Modia3D.VarFloatType

    faceIndices = collect(UInt64, 1:length(surfaceFaces))

    return createBVKnot(surfaceVertexCoordinates, surfaceFaces, faceIndices, 0, maxPenetration)

end

function createBVKnot(surfaceVertexCoordinates::Vector{SVector{3,F}}, surfaceFaces::Vector{ContactSurfaceFace{F}}, faceIndices::Vector{UInt64}, level::Int, maxPenetration::F) where F <: Modia3D.VarFloatType

    faceIndicesLowerBV = Vector{UInt64}()
    faceIndicesUpperBV = Vector{UInt64}()
    minLowerBV = SVector{3,F}(fill(Inf, 3))
    maxLowerBV = SVector{3,F}(fill(-Inf, 3))
    minUpperBV = SVector{3,F}(fill(Inf, 3))
    maxUpperBV = SVector{3,F}(fill(-Inf, 3))
    volLowerBV = F(0.0)
    volUpperBV = F(0.0)

    if length(faceIndices) > 1
        # determine axis of greatest expanse
        axis = 0
        minBC = fill(F(Inf), 3)
        maxBC = fill(F(-Inf), 3)
        for iface in 1:length(faceIndices)
            faceIndex = faceIndices[iface]
            face = surfaceFaces[faceIndex]
            minBC = min.(minBC, face.baryCentre)
            maxBC = max.(maxBC, face.baryCentre)
        end
        dvec = maxBC - minBC
        if dvec[1] > dvec[2]
            if dvec[1] > dvec[3]
                axis = 1
            else
                axis = 3
            end
        else
            if dvec[2] > dvec[3]
                axis = 2
            else
                axis = 3
            end
        end

        # sort coordinates of barycentres along axis
        sort!(faceIndices, lt=(i, j) -> surfaceFaces[i].baryCentre[axis] != surfaceFaces[j].baryCentre[axis] ? isless(surfaceFaces[i].baryCentre[axis], surfaceFaces[j].baryCentre[axis]) : isless(i, j))
#       sort!(faceIndices, by=idx -> surfaceFaces[idx].baryCentre[axis])
    end

    # distribute faces to lower and upper BV
    for icount in 1:length(faceIndices)
        iface = 0
        if isodd(icount)
            iface = div(icount+1, 2)                          # odd icount -> count forwards 1, 2,...
        else
            iface = length(faceIndices) - div(icount, 2) + 1  # even icount -> count backwards nface, nface-1,...
        end
        faceIndex = faceIndices[iface]
        face = surfaceFaces[faceIndex]

        # calculate element BV
        minElem = SVector{3,F}(fill(Inf, 3))
        maxElem = SVector{3,F}(fill(-Inf, 3))
        if maxPenetration == F(0.0)
            # calculate face BV
            for vertexIndex in face.vertexIndices
                minElem = min.(minElem, surfaceVertexCoordinates[vertexIndex])
                maxElem = max.(maxElem, surfaceVertexCoordinates[vertexIndex])
            end
        else
            # calculate penetration line BV
            maxPenPoint = face.baryCentre - maxPenetration*face.normalVector
            minElem = min.(face.baryCentre, maxPenPoint)
            maxElem = max.(face.baryCentre, maxPenPoint)
        end

        # distribute face among lower and upper BV
        if icount == 1
            # first face initialises lower BV
            minLowerBV = minElem
            maxLowerBV = maxElem
            volLowerBV = (maxElem[1] - minElem[1])*(maxElem[2] - minElem[2])*(maxElem[3] - minElem[3])
            push!(faceIndicesLowerBV, faceIndex)
        elseif icount == 2
            # last face initialises upper BV
            minUpperBV = minElem
            maxUpperBV = maxElem
            volUpperBV = (maxElem[1] - minElem[1])*(maxElem[2] - minElem[2])*(maxElem[3] - minElem[3])
            push!(faceIndicesUpperBV, faceIndex)
        else
            # add face to BV with lower increase of volume
            minLowerBVCandidate = min.(minLowerBV, minElem)
            maxLowerBVCandidate = max.(maxLowerBV, maxElem)
            dvecLowerBVCandidate = maxLowerBVCandidate - minLowerBVCandidate
            volLowerBVCandidate = dvecLowerBVCandidate[1]*dvecLowerBVCandidate[2]*dvecLowerBVCandidate[3]
            minUpperBVCandidate = min.(minUpperBV, minElem)
            maxUpperBVCandidate = max.(maxUpperBV, maxElem)
            dvecUpperBVCandidate = maxUpperBVCandidate - minUpperBVCandidate
            volUpperBVCandidate = dvecUpperBVCandidate[1]*dvecUpperBVCandidate[2]*dvecUpperBVCandidate[3]

            addLower = true
            if length(faceIndices) > 1
                dvolLower = volLowerBVCandidate - volLowerBV
                dvolUpper = volUpperBVCandidate - volUpperBV
                if dvolLower < dvolUpper
                    addLower = true
                elseif dvolLower > dvolUpper
                    addLower = false
                elseif length(faceIndicesLowerBV) > length(faceIndicesUpperBV)
                    addLower = false
                else
                    addLower = true
                end
            end

            # add face and extend BV
            if addLower
                minLowerBV = minLowerBVCandidate
                maxLowerBV = maxLowerBVCandidate
                volLowerBV = volLowerBVCandidate
                push!(faceIndicesLowerBV, faceIndex)
            else
                minUpperBV = minUpperBVCandidate
                maxUpperBV = maxUpperBVCandidate
                volUpperBV = volUpperBVCandidate
                push!(faceIndicesUpperBV, faceIndex)
            end
        end
    end

    minBV = SVector{3,F}(fill(Inf, 3))
    maxBV = SVector{3,F}(fill(-Inf, 3))
    faceBV = UInt64(0)
    lowerBV = nothing
    upperBV = nothing
    if length(faceIndices) > 1
        # more than one face -> continue recursion
        minBV = min.(minLowerBV, minUpperBV)
        maxBV = max.(maxLowerBV, maxUpperBV)
        # recursively call for lower BV
        lowerBV = createBVKnot(surfaceVertexCoordinates, surfaceFaces, faceIndicesLowerBV, level+1, maxPenetration)
        # recursively call for upper BV
        upperBV = createBVKnot(surfaceVertexCoordinates, surfaceFaces, faceIndicesUpperBV, level+1, maxPenetration)
    else
        # one face -> stop recursion
        minBV = minLowerBV
        maxBV = maxLowerBV
        faceBV = faceIndicesLowerBV[1]
    end
    # avoid zero volume BV
    minBound = fill(F(Inf), 3)
    maxBound = fill(F(-Inf), 3)
    for iaxis in 1:3
        dlen = maxBV[iaxis] - minBV[iaxis]
        if dlen < MinBVLength
            minBound[iaxis] = minBV[iaxis] - F(0.5)*(MinBVLength - dlen)
            maxBound[iaxis] = maxBV[iaxis] + F(0.5)*(MinBVLength - dlen)
        else
            minBound[iaxis] = minBV[iaxis]
            maxBound[iaxis] = maxBV[iaxis]
        end
    end

    return BVTree{F}(SVector{3,F}(minBound), SVector{3,F}(maxBound), faceBV, lowerBV, upperBV)

end

function createState!(force::PolygonalContactModel{F}) where F <: Modia3D.VarFloatType

    A_EF = zeros(SMatrix{3,3,F,9})
    r_EF = zeros(SVector{3,F})
    w_EF = zeros(SVector{3,F})
    v_EF = zeros(SVector{3,F})
    surfaceFMaxBBDirections = Vector{SVector{3,Int}}(undef, 3)
    surfaceFVertexCoordinates = Vector{SVector{3,F}}(undef, length(force.surfaceF.vertexCoordinates))
    surfaceFVertexCoordinatesCache = Vector{Bool}(undef, length(force.surfaceF.vertexCoordinates))
    surfaceFBaryCentres = Vector{SVector{3,F}}(undef, length(force.surfaceF.faces))
    surfaceFBaryCentresCache = Vector{Bool}(undef, length(force.surfaceF.faces))
    surfaceFNormalVectors = Vector{SVector{3,F}}(undef, length(force.surfaceF.faces))
    surfaceFNormalVectorsCache = Vector{Bool}(undef, length(force.surfaceF.faces))
    surfaceEContactElements = Vector{ContactElement{F}}()
    if force.task == 1 || force.task == 2
        for iface in 1:length(force.surfaceE.faces)
            contactElement = ContactElement{F}(inactive, zeros(SVector{3,F}), zeros(SVector{3,F}), F(0.0), F(0.0), F(0.0))
            push!(surfaceEContactElements, contactElement)
        end
    end
    surfaceFContactElements = Vector{ContactElement{F}}()
    if force.task == 1 || force.task == 3
        for iface in 1:length(force.surfaceF.faces)
            contactElement = ContactElement{F}(inactive, zeros(SVector{3,F}), zeros(SVector{3,F}), F(0.0), F(0.0), F(0.0))
            push!(surfaceFContactElements, contactElement)
        end
    end
    workVertexCoordinatesE = Vector{SVector{3,F}}(undef, 3)
    workVertexCoordinatesF = Vector{SVector{3,F}}(undef, 3)

    force.state = ContactState{F}(A_EF, r_EF, w_EF, v_EF, surfaceFMaxBBDirections,
                                  surfaceFVertexCoordinates, surfaceFVertexCoordinatesCache,
                                  surfaceFBaryCentres, surfaceFBaryCentresCache,
                                  surfaceFNormalVectors, surfaceFNormalVectorsCache,
                                  surfaceEContactElements,
                                  surfaceFContactElements,
                                  workVertexCoordinatesE,
                                  workVertexCoordinatesF,
                                  0, 0, 0, F(0.0))

    return nothing

end

function initialiseEvaluation!(force::PolygonalContactModel{F}) where F <: Modia3D.VarFloatType

    # get contact state input
    force.state.A_EF = measFrameRotation(force.obj1; frameOrig=force.obj2)
    force.state.r_EF = measFramePosition(force.obj2; frameOrig=force.obj1, frameCoord=force.obj1)
    force.state.w_EF = measFrameRotVelocity(force.obj2; frameOrig=force.obj1, frameCoord=force.obj1)
    force.state.v_EF = measFrameTransVelocity(force.obj2; frameOrig=force.obj1, frameCoord=force.obj1, frameObsrv=force.obj1)

    # reset contact state cache
    force.state.surfaceFBaryCentresCache[:] .= false
    force.state.surfaceFNormalVectorsCache[:] .= false
    force.state.surfaceFVertexCoordinatesCache[:] .= false

    # initialise counters
    force.state.numBVChecks = 0
    force.state.numPenChecks = 0
    force.state.numPenetrations = 0

    return nothing

end

function generateContactElements!(force::PolygonalContactModel{F}, baseE::Bool) where F <: Modia3D.VarFloatType

    # determine extremal vertices of BV cuboids
    unitCube = SMatrix{3,8,F}(-1.0, -1.0, -1.0,  # vertex 1 of unit cube
                               1.0, -1.0, -1.0,  # vertex 2 of unit cube
                              -1.0,  1.0, -1.0,  # vertex 3 of unit cube
                               1.0,  1.0, -1.0,  # vertex 4 of unit cube
                              -1.0, -1.0,  1.0,  # vertex 5 of unit cube
                               1.0, -1.0,  1.0,  # vertex 6 of unit cube
                              -1.0,  1.0,  1.0,  # vertex 7 of unit cube
                               1.0,  1.0,  1.0)  # vertex 8 of unit cube
    vertexPosMax = zeros(SVector{3,F})
    @inbounds for ivertex in 1:8
        vertexPos = force.state.A_EF * unitCube[:,ivertex]  # E_vertexPos := A_EF * F_vertexPos
        for iaxis in 1:3
            if vertexPos[iaxis] > vertexPosMax[iaxis]
                vertexPosMax = setindex(vertexPosMax, vertexPos[iaxis], iaxis)
                force.state.surfaceFMaxBBDirections[iaxis] = SVector{3,Int}(view(unitCube, 1:3, ivertex))
            end
        end
    end

    if baseE
        # reset contact elements of base surface E
        for contactElement in force.state.surfaceEContactElements
            contactElement.status = inactive
        end
        # surface E is base -> penetration line BVs for E and face BVs for F
        surfaceEBVTree = force.surfaceE.penLineBVTree
        surfaceFBVTree = force.surfaceF.faceBVTree
    else
        # reset contact elements of base surface F
        for contactElement in force.state.surfaceFContactElements
            contactElement.status = inactive
        end
        # surface F is base -> face BVs for E and penetration line BVs for F
        surfaceEBVTree = force.surfaceE.faceBVTree
        surfaceFBVTree = force.surfaceF.penLineBVTree
    end

    # check for collisions using OOBB-tree algorithm
    detectCollision!(force, surfaceEBVTree, surfaceFBVTree, baseE)

    return nothing

end

function detectCollision!(force::PolygonalContactModel{F}, surfaceEBVTree::BVTree{F}, surfaceFBVTree::BVTree{F}, baseE::Bool) where F <: Modia3D.VarFloatType

    force.state.numBVChecks += 1  # increment number of BV checks

    # check for collision of bounding boxes E and F
    @inbounds for iaxisE in 1:3
        # determine bounding box F vertex coordinates with extremal coordinates in iaxisE-direction of E
        minVertexF = zeros(SVector{3,F})
        maxVertexF = zeros(SVector{3,F})
        for iaxisF in 1:3
            if force.state.surfaceFMaxBBDirections[iaxisE][iaxisF] > 0
                maxVertexF = setindex(maxVertexF, surfaceFBVTree.maxBound[iaxisF], iaxisF)
                minVertexF = setindex(minVertexF, surfaceFBVTree.minBound[iaxisF], iaxisF)
            else
                maxVertexF = setindex(maxVertexF, surfaceFBVTree.minBound[iaxisF], iaxisF)
                minVertexF = setindex(minVertexF, surfaceFBVTree.maxBound[iaxisF], iaxisF)
            end
        end

        # transform vertex coordinates F->E
        minBoundF = force.state.r_EF[iaxisE] + view(force.state.A_EF, iaxisE, :)' * minVertexF

        if minBoundF >= surfaceEBVTree.maxBound[iaxisE]
            # bounding boxes E and F do not intersect -> no collision
            return nothing
        end

        # transform vertex coordinates F->E
        maxBoundF = force.state.r_EF[iaxisE] + view(force.state.A_EF, iaxisE, :)' * maxVertexF

        if maxBoundF <= surfaceEBVTree.minBound[iaxisE]
            # bounding boxes E and F do not intersect -> no collision
            return nothing
        end
    end

    # bounding boxes E and F intersect -> check for collision of children
    if isnothing(surfaceEBVTree.lowerBV)
        # E is leaf
        if isnothing(surfaceFBVTree.lowerBV)
            # E and F are leafs -> check for intersection base penetration line - target face
            detectPenetration!(force, surfaceEBVTree.faceIndex, surfaceFBVTree.faceIndex, baseE)
        else
            # E is leaf, F is not leaf -> recursively call for E and children of F
            detectCollision!(force, surfaceEBVTree, surfaceFBVTree.lowerBV, baseE)
            if !isnothing(surfaceFBVTree.upperBV)
                detectCollision!(force, surfaceEBVTree, surfaceFBVTree.upperBV, baseE)
            end
        end
    else
        if isnothing(surfaceFBVTree.lowerBV)
            # F is leaf, E is not leaf -> recursively call for F and children of E
            detectCollision!(force, surfaceEBVTree.lowerBV, surfaceFBVTree, baseE)
            if !isnothing(surfaceEBVTree.upperBV)
                detectCollision!(force, surfaceEBVTree.upperBV, surfaceFBVTree, baseE)
            end
        else
            # E and F are not leafs -> recursively call for children of E and F
            detectCollision!(force, surfaceEBVTree.lowerBV, surfaceFBVTree.lowerBV, baseE)
            if !isnothing(surfaceEBVTree.upperBV)
                detectCollision!(force, surfaceEBVTree.upperBV, surfaceFBVTree.lowerBV, baseE)
                if !isnothing(surfaceFBVTree.upperBV)
                    detectCollision!(force, surfaceEBVTree.lowerBV, surfaceFBVTree.upperBV, baseE)
                    detectCollision!(force, surfaceEBVTree.upperBV, surfaceFBVTree.upperBV, baseE)
                end
            elseif !isnothing(surfaceFBVTree.upperBV)
                detectCollision!(force, surfaceEBVTree.lowerBV, surfaceFBVTree.upperBV, baseE)
            end
        end
    end

    return nothing

end

function detectPenetration!(force::PolygonalContactModel{F}, surfaceEFaceIndex::UInt64, surfaceFFaceIndex::UInt64, baseE::Bool) where F <: Modia3D.VarFloatType

    @inbounds begin
        force.state.numPenChecks += 1  # increment number of penetration checks

        for ivertex in 1:3
            vertexIndex = force.surfaceE.faces[surfaceEFaceIndex].vertexIndices[ivertex]
            force.state.workVertexCoordinatesE[ivertex] = force.surfaceE.vertexCoordinates[vertexIndex]
        end

        for ivertex in 1:3
            vertexIndex = force.surfaceF.faces[surfaceFFaceIndex].vertexIndices[ivertex]
            if force.state.surfaceFVertexCoordinatesCache[vertexIndex]
                force.state.workVertexCoordinatesF[ivertex] = force.state.surfaceFVertexCoordinates[vertexIndex]
            else
                vertF = force.state.r_EF + force.state.A_EF * force.surfaceF.vertexCoordinates[vertexIndex]
                force.state.workVertexCoordinatesF[ivertex] = vertF
                force.state.surfaceFVertexCoordinates[vertexIndex] = vertF
                force.state.surfaceFVertexCoordinatesCache[vertexIndex] = true
            end
        end

        baryCentreE = force.surfaceE.faces[surfaceEFaceIndex].baryCentre
        if force.state.surfaceFBaryCentresCache[surfaceFFaceIndex]
            baryCentreF = force.state.surfaceFBaryCentres[surfaceFFaceIndex]
        else
            baryCentreF = force.state.r_EF + force.state.A_EF * force.surfaceF.faces[surfaceFFaceIndex].baryCentre
            force.state.surfaceFBaryCentres[surfaceFFaceIndex] = baryCentreF
            force.state.surfaceFBaryCentresCache[surfaceFFaceIndex] = true
        end

        normalVectorE = force.surfaceE.faces[surfaceEFaceIndex].normalVector
        if force.state.surfaceFNormalVectorsCache[surfaceFFaceIndex]
            normalVectorF = force.state.surfaceFNormalVectors[surfaceFFaceIndex]
        else
            normalVectorF = force.state.A_EF * force.surfaceF.faces[surfaceFFaceIndex].normalVector
            force.state.surfaceFNormalVectors[surfaceFFaceIndex] = normalVectorF
            force.state.surfaceFNormalVectorsCache[surfaceFFaceIndex] = true
        end

        if baseE
            # check for penetration base E - target F
            (status, penetration) = calculatePenetration(force.state.workVertexCoordinatesE, baryCentreE, normalVectorE,
                                                         force.state.workVertexCoordinatesF, baryCentreF, normalVectorF,
                                                         force.maxPenetration, force.DetEPS, force.DotEPS)

            if status != inactive
                # activate/update contact element
                contactElement = force.state.surfaceEContactElements[surfaceEFaceIndex]
                if contactElement.status == inactive || penetration < contactElement.penetration
                    contactElement.status = status
                    contactElement.normalVector = -normalVectorE
                    contactElement.position = baryCentreE - force.penetrationRatio*penetration*normalVectorE
                    contactElement.penetration = penetration
                    contactElement.area = force.surfaceE.faces[surfaceEFaceIndex].area
                    contactElement.normalForce = F(0.0)
                end
                force.state.numPenetrations += 1  # increment number of penetrations
            end
        else
            # check for penetration base F - target E
            (status, penetration) = calculatePenetration(force.state.workVertexCoordinatesF, baryCentreF, normalVectorF,
                                                         force.state.workVertexCoordinatesE, baryCentreE, normalVectorE,
                                                         force.maxPenetration, force.DetEPS, force.DotEPS)

            if status != inactive
                # activate/update contact element
                contactElement = force.state.surfaceFContactElements[surfaceFFaceIndex]
                if contactElement.status == inactive || penetration < contactElement.penetration
                    contactElement.status = status
                    contactElement.normalVector = normalVectorF
                    contactElement.position = baryCentreF - (F(1.0) - force.penetrationRatio)*penetration*normalVectorF
                    contactElement.penetration = penetration
                    contactElement.area = force.surfaceF.faces[surfaceFFaceIndex].area
                    contactElement.normalForce = F(0.0)
                end
                force.state.numPenetrations += 1  # increment number of penetrations
            end
        end
    end

    return nothing

end

@inline function calculatePenetration(baseVertexCoordinates::Vector{SVector{3,F}}  , baseBaryCentre::SVector{3,F}  , baseNormalVector::SVector{3,F},
                                      targetVertexCoordinates::Vector{SVector{3,F}}, targetBaryCentre::SVector{3,F}, targetNormalVector::SVector{3,F},
                                      maxPenetration::F, DetEPS::F, DotEPS::F) where F <: Modia3D.VarFloatType

    # initialisation
    status = inactive
    penetration = F(0.0)

    ntdnb = dot(targetNormalVector, baseNormalVector)  # ntdnb := nt * nb
    if abs(ntdnb) > DotEPS  # check if normals are perpendicular
        @inbounds begin
            # calculate cartesian base of base face
            e1 = normalize(baseVertexCoordinates[2] - baseVertexCoordinates[1])  # e1 := (v2 - v1)/norm(v2 - v1)
            e3 = baseNormalVector                                                # e3 := nb
            e2 = cross(e3, e1)                                                   # e2 := e3 x e1

            # project vertices of target face to plane e1e2
#           rvt = zeros(SMatrix{2,3,F,6})
#           for ivertex in 1:3
#               r_bv = targetVertexCoordinates[ivertex] - baseBaryCentre  # r_bv := r_Ev - r_Eb
#               rvt = setindex(rvt, dot(e1, r_bv), 1*ivertex)             # x_rvt := e1 * r_bv
#               rvt = setindex(rvt, dot(e2, r_bv), 2*ivertex)             # y_rvt := e2 * r_bv
#           end
            rvt = SMatrix{2,3,F,6}(
                dot(e1, targetVertexCoordinates[1] - baseBaryCentre),
                dot(e2, targetVertexCoordinates[1] - baseBaryCentre),
                dot(e1, targetVertexCoordinates[2] - baseBaryCentre),
                dot(e2, targetVertexCoordinates[2] - baseBaryCentre),
                dot(e1, targetVertexCoordinates[3] - baseBaryCentre),
                dot(e2, targetVertexCoordinates[3] - baseBaryCentre))

            # check if barycentre of base lies on the inside of projected target
            inside = true
            for ivertex1 in 1:3
                ivertex2 = mod1(ivertex1+1, 3)

                # vector between vertices of target
                if ntdnb < F(0.0)
                    # different orientation of base and target (valid penetration)
                    vt = view(rvt, :, ivertex1) - view(rvt, :, ivertex2)
                else
                    # same orientation of base and target (invalid penetration)
                    vt = view(rvt, :, ivertex2) - view(rvt, :, ivertex1)
                end

                # vector from 1st vertex of target to barycentre of base
                rb = -view(rvt, :, ivertex1)

                # determinant of [ vt rb ]
                determinant = vt[1]*rb[2] - vt[2]*rb[1]

                # determinant<0 -> barycentre of base lies on the outside of the current edge of target
                if determinant < -DetEPS
                    inside = false
                    break
                end
            end
        end

        if inside
            # calculate intersection point in target
            dis = dot(targetNormalVector, targetBaryCentre)      # dis := nt * r_Et (addend for Hesse normal form)
            dis = dis - dot(targetNormalVector, baseBaryCentre)  # dis := nt * r_Et - nt * r_Eb
            dis = dis / ntdnb                                    # dis := dis / (nt * nb)
            if dis < F(0.0)  # check if intersection in target lies on the inner side of base
                # check max. penetration
                if dis >= -maxPenetration
                    # intersection detected
                    penetration = -dis
                    if ntdnb < F(0.0)  # check if normals point in opposite directions
                        status = active
                    else
                        status = invalid
                    end
                end
            end
        end
    end

    return (status, penetration)

end

function calculateForceTorque!(force::PolygonalContactModel{F}) where F <: Modia3D.VarFloatType

    force12 = zeros(SVector{3,F})
    torque12 = zeros(SVector{3,F})
    results = zeros(SVector{23,F})

    numContactElements = 0
    area = F(0.0)
    penwgh = F(0.0)
    penmax = F(0.0)
    vnmin = F(0.0)
    vnmax = F(0.0)
    vtmin = F(0.0)
    vtmax = F(0.0)
    vnwgh = F(0.0)
    vtwgh = F(0.0)
    pnwgh = F(0.0)
    pnmax = F(0.0)
    qtwgh = F(0.0)
    qtmax = F(0.0)
    ftot = F(0.0)
    r_C = zeros(SVector{3,F})
    n_C = zeros(SVector{3,F})
    cparea = F(0.0)

    for surfaceContactElements in (force.state.surfaceEContactElements, force.state.surfaceFContactElements)
        for contactElement in surfaceContactElements
            if contactElement.status == active
                numContactElements += 1

                # kinematics
                r_FCE = contactElement.position - force.state.r_EF  # r_FCE := r_ECE - r_EF

                v_CE = force.state.v_EF + cross(force.state.w_EF, r_FCE)  # v_CE := v_EF + w_EF x r_FCE

                vn = dot(contactElement.normalVector, v_CE)  # vn := n_CE * v_CE
                v_n = vn * contactElement.normalVector       # v_n := vn * n_CE

                v_t = v_CE - v_n  # v_t := v_CE - v_n
                vt = norm(v_t)    # vt := abs(v_t)

                # additional results
                area += contactElement.area

                penwgh += contactElement.penetration * contactElement.area
                penmax = max(penmax, contactElement.penetration)

                if numContactElements == 1
                    vnmin = vn
                    vnmax = vn
                    vtmin = vt
                    vtmax = vt
                else
                    vnmin = min(vnmin, vn)
                    vnmax = max(vnmax, vn)
                    vtmin = min(vtmin, vt)
                    vtmax = max(vtmax, vt)
                end
                vnwgh += vn * contactElement.area
                vtwgh += vt * contactElement.area

                # calculate normal force
                fc = force.combinedLayerStiffness * contactElement.area * contactElement.penetration  # Fc := c * A * u

                arealDampingFactor = F(0.0)
                if vn > F(0.0)
                    # compression
                    arealDampingFactor = force.compressionArealDampingFactor
                else
                    # expansion
                    if force.expansionDampingMode != 1
                        arealDampingFactor = force.expansionArealDampingFactor
                    end
                end

                fd = arealDampingFactor * contactElement.area * vn  # Fd := d_A * A * vn

                if force.dampingTransitionMode == 1 && contactElement.penetration < force.dampingTransitionDepth
                    fd *= contactElement.penetration / force.dampingTransitionDepth  # linear damping transition
                end

                fn = fc + fd

                if force.expansionDampingMode == 0
                    fn = max(fn, F(0.0))  # avoid tension (caused by damping force)
                end

                f_n = fn * contactElement.normalVector  # f_n := fn * n

                # additional results
                pnwgh += fn
                pnmax = max(pnmax, fn/contactElement.area)

                # calculate tangential force
                ft = F(0.0)
                f_t = zeros(SVector{3,F})
                if vt > F(1.0e-37)
                    ft = force.frictionCoefficient * fn  # ft := mu * fn
                    if vt < force.frictionRegularizationVelocity
                        rv = vt / force.frictionRegularizationVelocity
                        ft = ft * rv * (F(2.0) - rv)  # linear friction regularisation
                    end

                    f_t = ft * normalize(v_t)  # f_t := ft * v_t / vt
                end

                # additional results
                qtwgh += ft
                qtmax = max(qtmax, ft/contactElement.area)

                # add force to resulting vector wrench
                frc = f_n + f_t          # total force of contact element
                rxf = cross(r_FCE, frc)  # torque resulting from relocating force vector
                force12 += frc
                torque12 += rxf

                # additional results
                fabs = norm(frc)
                ftot += fabs
                r_C += contactElement.position * fabs
                n_C += contactElement.normalVector * fabs

                if force.task == 1
                    contactElement.normalForce = F(0.5) * fn
                else
                    contactElement.normalForce = fn
                end
            end
        end
    end
    force.state.maxPenetration = max(force.state.maxPenetration, penmax)

    # 2-pass base/target: halve force and torque
    if force.task == 1
        force12 *= F(0.5)
        torque12 *= F(0.5)
        cparea = F(0.5) * area
    else
        cparea = area
    end

    # additional results
    if area > F(0.0) && ftot > F(0.0)
        results = SVector{23,F}(
            F(force.state.numBVChecks),
            F(force.state.numPenChecks),
            F(force.state.numPenetrations),
            F(numContactElements),
            cparea,          # contact patch area
            penwgh / area,   # weighted penetration
            penmax,          # maximum penetration
            vnmin,           # minimum normal velocity
            vnwgh / area,    # weighted normal velocity
            vnmax,           # maximum normal velocity
            vtmin,           # minimum tangential velocity
            vtwgh / area,    # weighted tangential velocity
            vtmax,           # maximum tangential velocity
            pnwgh / area,    # weighted contact pressure
            pnmax,           # maximum contact pressure
            qtwgh / area,    # weighted contact traction
            qtmax,           # maximum contact traction
            r_C[1] / ftot,   # weighted contact x-position
            r_C[2] / ftot,   # weighted contact y-position
            r_C[3] / ftot,   # weighted contact z-position
            n_C[1] / ftot,   # weighted contact x-normal
            n_C[2] / ftot,   # weighted contact y-normal
            n_C[3] / ftot    # weighted contact z-normal
        )
    end

    return (force12, torque12, results)

end
