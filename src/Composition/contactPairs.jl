
#-------------------------------------- Default Contact Detection -------------------------------

"""
    ContactPairs(nzmax, collSuperObjs, noCPairs, dummyObject3D)

Generate a new ContactPairs structure used for communication between the Object3D handler and a ContactDetection handler.

- `DummyObject3D::Modia3D.AbstractObject3DFeature`: A dummy Object3D that can be used in the struct as element of a vector of Object3Ds
  to fill the vector with a dummy value of the correct type.
"""
mutable struct ContactPairs
    # Solid shapes used in contact detection (provided by Object3D handler)
    allowedToMove::Vector{Union{Bool,Nothing}}
    dummyObject3D::Modia3D.AbstractObject3DFeature     # Dummy Object3D for non-used elements of z-Vector.

    # Dimensions
    ne::Int                                         # length(collSuperObjs)
    nz::Int                                         # length(z)
    nzContact::Int                                  # length(z | z has contact) length of z where zi has contact

    function ContactPairs(world::Composition.Object3D, AABB::Vector{Vector{Basics.BoundingBox}}, superObjs::Vector{SuperObjsRow},
                        allowedToMove::Vector{Union{Bool,Nothing}}, visualizeBoundingBox::Bool, nVisualContSupPoints::Int,
                        visualizeContactPoints::Bool, visualizeSupportPoints::Bool, defaultContactSphereDiameter::Float64)
        @assert(length(superObjs) > 0)
        @assert(nVisualContSupPoints > 0)
        dummyObject3D = Composition.emptyObject3DFeature

        lengthCollSuperObjs = length(superObjs)
        if lengthCollSuperObjs <= 1
            error("There's nothing to collide. All Object3Ds are rigidly connected.")
        end

        nzContact = 0
        nz = 2

        if lengthCollSuperObjs > 2
            nz = lengthCollSuperObjs
        end

        if visualizeContactPoints
            addContactVisuObjToWorld!(world, nVisualContSupPoints, defaultContactSphereDiameter)
        end
        if visualizeSupportPoints
            addSupportVisuObjToWorld!(world, nVisualContSupPoints, defaultContactSphereDiameter)
        end

        if visualizeBoundingBox
            addAABBVisuToWorld!(world, AABB)
        end

        new(allowedToMove, dummyObject3D, lengthCollSuperObjs, nz, nzContact)
    end
end

initializeContactDetection!(ch::Modia3D.AbstractContactDetection, collSuperObjs::Vector{Vector{Modia3D.AbstractObject3DFeature}}, noCPairs::Vector{Vector{Int64}}) = error("No contact detection handler defined.")
selectContactPairsWithEvent!(ch::Modia3D.AbstractContactDetection)   = error("No contact detection handler defined.")
selectContactPairsNoEvent!(ch::Modia3D.AbstractContactDetection)     = error("No contact detection handler defined.")
getDistances!(ch::Modia3D.AbstractContactDetection)                  = error("No contact detection handler defined.")
setComputationFlag(ch::Modia3D.AbstractContactDetection)             = error("No contact detection handler defined.")
closeContactDetection!(ch::Modia3D.AbstractContactDetection)         = error("No contact detection handler defined.")
