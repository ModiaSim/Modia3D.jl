# ------------------------------ GrippingPair ----------------------------------
# is used in referencePathInternal
mutable struct GrippingPair{F <: Modia3D.VarFloatType}
    gripStatus::GripStatus                 # actual grip status
    movableObj::Composition.Object3D{F}    # must be part of a movable unit
    # is set for Attach and ReleaseAndAttach
    robotOrDepot::Composition.Object3D{F}  # must be part of a gripper unit or part of a depot (like bottom or other movable unit)
    GrippingPair{F}(gripStatus::GripStatus, movableObj::Composition.Object3D{F}) where {F <: Modia3D.VarFloatType} = new(gripStatus, movableObj)
    GrippingPair{F}(gripStatus::GripStatus, movableObj::Composition.Object3D{F}, robotOrDepot::Composition.Object3D{F}) where {F <: Modia3D.VarFloatType}  = new(gripStatus, movableObj, robotOrDepot)
end


# ------------------------ utility functions -----------------------------------
areObjectsClose(scene::Composition.Scene{F}, robotOrDepot::Composition.Object3D{F}, movableObj::Composition.Object3D{F}) where F <: Modia3D.VarFloatType =
    (norm(robotOrDepot.r_abs - movableObj.r_abs) < scene.options.gap, norm(robotOrDepot.r_abs - movableObj.r_abs))

function getSuperObjsMovable(scene::Composition.Scene{F}, pos::Int64) where F <: Modia3D.VarFloatType
    superObjs::Vector{Composition.SuperObjsRow{F}} = scene.superObjs
    return superObjs[pos].superObjMovable.superObj
end

function getSuperObjsCollision(scene::Composition.Scene{F}, pos::Int64) where F <: Modia3D.VarFloatType
    superObjs::Vector{Composition.SuperObjsRow{F}} = scene.superObjs
    return superObjs[pos].superObjCollision.superObj
end


# ----------------------- checkGrippingFeatures --------------------------------
# check if most features are fullfiled to change parent object
# check 1: both Object3Ds must be lockable
# check 2: the so-called movableObj must belong to a movable unit
# check 3: robotOrDepot Object3D must belong to a gripper unit for gripping / releasing
# check 4: robotOrDepot Object3D must belong to a depot or movable unit for releasing and set down (it's not allowed to be part of a gripper unit)
function checkGrippingFeatures(scene::Composition.Scene{F}, gripPair::GrippingPair{F}) where F <: Modia3D.VarFloatType
    allowedToChange = true
    movableObj::Composition.Object3D{F} = gripPair.movableObj

    # movableObj must be lockable
    if !movableObj.interactionManner.lockable
        error("movableObj is not lockable")
        #printWarnLockable(robotOrDepot, movableObj)
        allowedToChange = false
        return false
    end

    # the so-called movableObj must belong to a movable unit
    movableObjIsMovable = Composition.objectHasMovablePos(movableObj)
    if !movableObjIsMovable
        error("das movable obj gehört nicht zu movable")
        printWarnMovableUnit(movableObj)
        allowedToChange = false
        return false
    end


    if gripPair.gripStatus == ReleaseAndSetDown ||  gripPair.gripStatus == Grip
        if isdefined(gripPair, :robotOrDepot)
            robotOrDepot::Composition.Object3D{F} = gripPair.robotOrDepot
            if !robotOrDepot.interactionManner.lockable
                error("For ActionAttach and ActionAttachAndRelease the second Object3D must be lockable")
                #printWarnLockable(robotOrDepot, movableObj)
                allowedToChange = false
                return false
            end

            if allowedToChange
                (objsAreClose, distBetweenObjs) = areObjectsClose(scene, robotOrDepot, movableObj)
                if objsAreClose
                    return true     # all checks for dynamic changing structure are successful
                else
                    printWarnNotCloseGripOrRelease(robotOrDepot, movableObj, distBetweenObjs)
                    return false
                end
            end
        else
            @error("For ActionAttach and ActionReleaseAndAttach a second lockabe Object3D must be defined.")
            return false
        end
    end
    return true
end

# ----------------------- changeParentOfMovableUnit! ---------------------------
# change actual parent of movable unit to the parent of gripper or depot unit,
# or to the movable unit itself (Step 3, Step 5)
# all massProperties (mass, center of mass, inertia tensor) of movable unit
# are substracted from old parent and added to the new parent (Step 4, Step 6)
# for Grip and ReleaseAndSetDown it is the same procedure
#   the parent of robotOrDepot will be the new parent of whole movable unit (Step 3)
# Step 1: check if it is allowed to change (because all formal checks where ok)
# Step 2: check if both Object3Ds are close enough (user defined gap) the whole
#         movable unit gets a new parent
# Step 3: decide which Object3D will be the new parent
#         (parent of movable unit or parent of robotOrDepot unit)
#         --> this affects: AABB and collisionSuperObjs
# Step 4: substract massProperties of movable unit from old parent
# Step 5: after this step all Object3Ds of the movable unit
#         are direct children of the new parent
# Step 6: add massProperties of movable unit to new parent
function changeParentOfMovableUnit!(scene::Composition.Scene{F}, world::Composition.Object3D{F}, gripPair::GrippingPair{F}) where F <: Modia3D.VarFloatType
    # parent of movable unit is changed
    movableObjs = getSuperObjsMovable(scene,
    gripPair.movableObj.interactionManner.movablePos)
    movableObjRoot::Composition.Object3D{F} = movableObjs[1]

    # new parent is world
    if gripPair.gripStatus == Release
        # new parent is world
        Basics.deleteItem(movableObjRoot.parent.children, movableObjRoot)
        Composition.changeJointFromFixToFreeMotion!(world, movableObjRoot)
        return nothing
    end

    # new parent is world
    if gripPair.gripStatus == Delete
        Composition.changeParent(world, movableObjRoot)
        Composition.changeJointFromFreeMotionToFix!(world, movableObjRoot)
        for obj in movableObjs
            obj.visualMaterial.transparency = 1.0
            obj.feature = Composition.emptyObject3DFeature
            obj.hasMass = false
        end
        return nothing
    end

    # new parent is robotOrDepot
    if gripPair.gripStatus == ReleaseAndSetDown ||  gripPair.gripStatus == Grip
        if isdefined(gripPair, :robotOrDepot)
            robotOrDepot::Composition.Object3D{F} = gripPair.robotOrDepot
        else
            @error("For ActionAttach and ActionReleaseAndAttach a second lockabe Object3D must be defined.")
            return nothing
        end
    end

    if gripPair.gripStatus == ReleaseAndSetDown
        robotOrDepotPos = robotOrDepot.interactionManner.originPos
        newParent = Composition.getRootObj(scene.buffer[robotOrDepotPos])
        Composition.changeParent(newParent, movableObjRoot)
        return nothing
    end

    if gripPair.gripStatus == Grip
        # new parent is parent of robotOrDepot (for Grip || ReleaseAndSetDown)
        robotOrDepotPos = robotOrDepot.interactionManner.originPos
        newParent = Composition.getRootObj(scene.buffer[robotOrDepotPos])
        Composition.changeParent(newParent, movableObjRoot)

        if gripPair.gripStatus == Grip && Composition.isFree(movableObjRoot)
            Composition.changeJointFromFreeMotionToFix!(newParent, movableObjRoot)
        end
        return nothing
    end
    return nothing
end


# -------------------------- several warnings ----------------------------------
printWarnLockable(robotOrDepot::Composition.Object3D{F}, movableObj::Composition.Object3D{F}) where F <: Modia3D.VarFloatType =
    @warn("Only if both objects are lockable gripping/releasing can take place. Set interactionBehavior = Modia3D.Lockable. Please, check out $(Modia3D.fullName(robotOrDepot)), $(Modia3D.fullName(movableObj)).")

printWarnMovableUnit(movableObj::Composition.Object3D{F}) where F <: Modia3D.VarFloatType =
    @warn("Only movable units can be gripped, released or released and set down. To build up a movable unit choose a parent Object3D and set interactionBehavior = Modia3D.Movable. All its children will belong to this movable unit. Please, check out the parent of $(Modia3D.fullName(movableObj))." )

printWarnGripperUnit(robotOrDepot::Composition.Object3D{F}, rootObjrobotOrDepot::Composition.Object3D{F}, rootCollObj::Composition.Object3D{F}) where F <: Modia3D.VarFloatType =
    @warn("Only gripping units are allowed to grip or to release a movable unit. To build up a gripping unit choose a parent Object3D and set interactionBehavior = Modia3D.Gripper. All its children will belong to this gripping unit.  Please, check out: $(Modia3D.fullName(robotOrDepot)), $(Modia3D.fullName(rootObjrobotOrDepot)), $(Modia3D.fullName(rootCollObj))." )

printWarnGripperUnit(robotOrDepot::Composition.Object3D{F}, rootObjrobotOrDepot::Composition.Object3D{F}) where F <: Modia3D.VarFloatType =
    @warn("Only gripping units are allowed to grip or to release a movable unit. To build up a gripping unit choose a parent Object3D and set interactionBehavior = Modia3D.Gripper. All its children will belong to this gripping unit.  Please, check out: $(Modia3D.fullName(robotOrDepot)), $(Modia3D.fullName(rootObjrobotOrDepot))." )

printWarnReleaseAndSetDown(robotOrDepot::Composition.Object3D{F}) where F <: Modia3D.VarFloatType =
    @warn("The actual Object3D is part of a gripping unit. Therefore it would grip instead of release it. Check out: $(Modia3D.fullName(robotOrDepot)).")

printWarnCannotRelease(movableObj::Composition.Object3D{F}) where F <: Modia3D.VarFloatType =
    @warn("The movable unit cannot be released, because it is not attached to the gripper unit. Please, check out $(Modia3D.fullName(movableObj)).")

printWarnNotCloseGripOrRelease(robotOrDepot::Composition.Object3D{F}, movableObj::Composition.Object3D{F}, dist::F) where F <: Modia3D.VarFloatType =
    @warn("Sorry, actual distance (= $dist) between $(Modia3D.fullName(robotOrDepot)) and $(Modia3D.fullName(movableObj)) is not close enough to grip or release. Adjust reference path or gap in SceneOptions.")
