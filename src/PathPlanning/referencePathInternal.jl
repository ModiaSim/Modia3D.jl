struct ArbitraryMotion
    # e.g. ptpJointSpace!
    func::Function
    inputArg1       # e.g. positions, waitingPeriod, robotOrDepot
    inputArg2       # e.g. nothing, movablePart
    inputArg3       # e.g. nothing, waitingPeriod
end


mutable struct ReferencePath{F <: Modia3D.VarFloatType, TimeType}
    names::Vector{AbstractString}     # names are set by the user
    initPos::Vector{Float64}          # initial positions
    position::Vector{Float64}         # current reference position
    velocity::Vector{Float64}         # current reference velocity
    acceleration::Vector{Float64}     # current reference acceleration
    v_max::Vector{Float64}            # max velocity is set by the user
    a_max::Vector{Float64}            # max acceleration is set by the user

    function ReferencePath{F, TimeType}(; names::AbstractVector,
                position = zeros(size(names,1)),
                v_max    = ones(size(names,1)),
                a_max    = ones(size(names,2))) where {F <: Modia3D.VarFloatType, TimeType <: AbstractFloat}
        new(names, position, position, zeros(size(names,1)), zeros(size(names,1)), v_max, a_max)
    end
end

mutable struct ModelActions{F <: Modia3D.VarFloatType, TimeType}
    path::String
    scene::Composition.Scene{F}
    instantiatedModel::Modia.SimulationModel{F,TimeType} # SimulationModel of model
    refMotion::Vector{ArbitraryMotion} # stores explicit robot functions
    nextEventTime::Float64

    referencePath::ReferencePath{F,TimeType}
    ptpPath::PTP_path{F}  # current PTP_path definition
    function ModelActions{F, TimeType}(; path, world, instantiatedModel,
                actions, startTime = 0.0) where {F <: Modia3D.VarFloatType, TimeType <: AbstractFloat}
        modelActions = new(path, world.feature, instantiatedModel, ArbitraryMotion[], startTime)
        actions(modelActions)
        return modelActions
    end
end

function addReferencePath(modelActions::ModelActions{F, TimeType};
    names::AbstractVector,
    position = zeros(size(names,1)),
    v_max    = ones(size(names,1)),
    a_max    = ones(size(names,2))) where {F <: Modia3D.VarFloatType, TimeType <: AbstractFloat}

    modelActions.referencePath = ReferencePath{F,TimeType}(names=names,
    position = position,
    v_max    = v_max,
    a_max    = a_max)
    return nothing
end

getRefPathPosition(referencePath::ModelActions{F,TimeType}, index::Int64) where {F <: Modia3D.VarFloatType, TimeType <: AbstractFloat} = referencePath.referencePath.position[index]
getRefPathPosition(; referencePath::ModelActions{F,TimeType}, index::Int64) where {F <: Modia3D.VarFloatType, TimeType <: AbstractFloat} = referencePath.referencePath.position[index]

getRefPathInitPosition(referencePath::ModelActions{F,TimeType}, index::Int64) where {F <: Modia3D.VarFloatType, TimeType <: AbstractFloat} = referencePath.initPos[index]
getRefPathInitPosition(; referencePath::ModelActions{F,TimeType}, index::Int64) where {F <: Modia3D.VarFloatType, TimeType <: AbstractFloat} = referencePath.initPos[index]


## -------------------- utility functions ------------------------
function setAttributesReferencePath!(ref::ModelActions{F,TimeType}, waitingPeriod::Float64) where {F <: Modia3D.VarFloatType, TimeType <: AbstractFloat}
    ref.nextEventTime = waitingPeriod
    Modia.setNextEvent!(ref.instantiatedModel, ref.nextEventTime)
    return nothing
end

function setAttributesReferencePathRestart!(ref::ModelActions{F,TimeType}, waitingPeriod::Float64) where {F <: Modia3D.VarFloatType, TimeType <: AbstractFloat}
    ref.nextEventTime = waitingPeriod
    Modia.setNextEvent!(ref.instantiatedModel, ref.nextEventTime)
    if Modia.isEvent(ref.instantiatedModel) && Modia.getTime(ref.instantiatedModel) >= 0.0
        Modia.setFullRestartEvent!(ref.instantiatedModel)
    end
    return nothing
end


function getComponent(referencePath::ModelActions{F,TimeType}, path::String) where {F <: Modia3D.VarFloatType, TimeType <: AbstractFloat}
    obj::Composition.Object3D{F} = getValue(referencePath.instantiatedModel, path)
    return obj
end


#--------------  Functions that can be executed at an event instant -------
### ------------------ ptpJointSpace! -------------------------------------
function ptpJointSpace!(ref::ModelActions{F,TimeType}, positions::Matrix{Float64}, dummy1, dummy2) where {F <: Modia3D.VarFloatType, TimeType <: AbstractFloat}
    pos1 = size(transpose(ref.referencePath.position))
    pos2 = size(positions)
    if pos1[2] != pos2[2]
        error("... from ptpJointSpace: Please check out dimensions of input $positions (= $pos2). Its dimensions should be (n, $(pos1[2])), n > 0.")
    end
    ref.ptpPath = Modia.PTP_path{F}(ref.referencePath.names,
        positions = [transpose(ref.referencePath.position); positions],
        startTime = Modia.getTime(ref.instantiatedModel),
        v_max     = ref.referencePath.v_max,
        a_max     = ref.referencePath.a_max)
    setAttributesReferencePath!(ref, Modia.pathEndTime(ref.ptpPath))
    return nothing
end

ptpJointSpace!(ref::ModelActions{F,TimeType}, positions::Vector{Float64}, dummy1, dummy2) where {F <: Modia3D.VarFloatType, TimeType <: AbstractFloat} =
    ptpJointSpace!(ref, reshape(positions, :, 1), dummy1, dummy2)

ptpJointSpace!(ref::ModelActions{F,TimeType}, positions::Float64, dummy1, dummy2) where {F <: Modia3D.VarFloatType, TimeType <: AbstractFloat} =
    ptpJointSpace!(ref, reshape([positions], :, 1), dummy1, dummy2)



###------------------------- ActionWait! ----------------------------------
function ActionWait!(ref::ModelActions{F,TimeType}, waitingPeriod::Float64, dummy1, dummy2) where {F <: Modia3D.VarFloatType, TimeType <: AbstractFloat}
    @assert(waitingPeriod >= 0.0)
    setAttributesReferencePath!(ref, Modia.getTime(ref.instantiatedModel) + waitingPeriod)
    return nothing
end

function robotEventAfterPeriod!(ref::ModelActions{F,TimeType}, waitingPeriod::Float64, dummy1, dummy2) where {F <: Modia3D.VarFloatType, TimeType <: AbstractFloat}
    @assert(waitingPeriod >= 0.0)
    setAttributesReferencePathRestart!(ref, Modia.getTime(ref.instantiatedModel) + waitingPeriod)
    return nothing
end

###--------------------- ActionReleaseAndAttach! -------------------------
function ActionReleaseAndAttach!(ref::ModelActions{F,TimeType}, robotOrDepot::String, movableObj::String, waitingPeriod::Float64) where {F <: Modia3D.VarFloatType, TimeType <: AbstractFloat}
    @assert(waitingPeriod >= 0.0) # waitingPeriod must be positive
    ref.scene.gripPair = Modia3D.GrippingPair{F}( Modia3D.ReleaseAndSetDown, getComponent(ref, movableObj), getComponent(ref, robotOrDepot))

    setAttributesReferencePathRestart!(ref, Modia.getTime(ref.instantiatedModel) + waitingPeriod)
    return nothing
end


###------------------------- ActionAttach! ----------------------------------
function ActionAttach!(ref::ModelActions{F,TimeType}, robotOrDepot::String, movableObj::String, waitingPeriod::Float64) where {F <: Modia3D.VarFloatType, TimeType <: AbstractFloat}
    @assert(waitingPeriod >= 0.0) # waitingPeriod must be bigger than 0.1 sec
    ref.scene.gripPair = Modia3D.GrippingPair{F}(Modia3D.Grip, getComponent(ref, movableObj), getComponent(ref, robotOrDepot))
    setAttributesReferencePathRestart!(ref, Modia.getTime(ref.instantiatedModel) + waitingPeriod)
    return nothing
end


###------------------------- ActionRelease! -------------------------------
function ActionRelease!(ref::ModelActions{F,TimeType}, movableObj::String, waitingPeriod::Float64, nothing) where {F <: Modia3D.VarFloatType, TimeType <: AbstractFloat}
    @assert(waitingPeriod >= 0.0) # waitingPeriod must be positive
    ref.scene.gripPair = Modia3D.GrippingPair{F}(Modia3D.Release, getComponent(ref, movableObj))

    setAttributesReferencePathRestart!(ref, Modia.getTime(ref.instantiatedModel) + waitingPeriod)
    return nothing
end


###------------------------- ActionDelete! -------------------------------
function ActionDelete!(ref::ModelActions{F,TimeType}, movableObj::String, waitingPeriod::Float64, nothing) where {F <: Modia3D.VarFloatType, TimeType <: AbstractFloat}
    @assert(waitingPeriod >= 0.0) # waitingPeriod must be positive
    ref.scene.gripPair = Modia3D.GrippingPair{F}(Modia3D.Delete, getComponent(ref, movableObj))

    setAttributesReferencePathRestart!(ref, Modia.getTime(ref.instantiatedModel) + waitingPeriod)
    return nothing
end

# ----------- warnings for ReleaseAndSetDown, Grip, Release -------------
printWarnReleaseAndSetDown(robotOrDepot, movableObj, waitingPeriod) =
    @warn("Function ActionReleaseAndAttach is for releasing and disjointing a movable unit from a gripper unit and setting it down and rigidly attaching it to a depot. The input argument 'robotOrDepot' must be a lockable Object3D and it is not allowed to be part of a gripper unit. The input argument 'movableObj' must be a lockable Object3D and it must belong to a movable unit. Check out ActionReleaseAndAttach(referencePath, robotOrDepot=$robotOrDepot, movableObj=$movableObj, waitingPeriod=$waitingPeriod).")


printWarnGrip(robotOrDepot, movableObj, waitingPeriod) =
    @warn("The function ActionAttach allows a gripper unit to grip a movable unit and rigidly attach both units during transportation. The input argument 'robotOrDepot' must be a lockable Object3D and it must belong to a gripper unit. The input argument 'movableObj' must be a lockable Object3D and it must belong to a movable unit. Check out ActionAttach(referencePath, robotOrDepot=$robotOrDepot, movableObj=$movableObj, waitingPeriod=$waitingPeriod).")


printWarnRelease(robotOrDepot, movableObj, waitingPeriod) =
    @warn("The function ActionRelease is for releasing and disjointing a movable unit from a gripper unit. Be aware the movable unit might fall down. The input argument 'robotOrDepot' must be a lockable Object3D and it must belong to a gripper unit. The input argument 'movableObj' must be a lockable Object3D and it must belong to a movable unit. Check out ActionRelease(referencePath, robotOrDepot=$robotOrDepot, movableObj=$movableObj, waitingPeriod=$waitingPeriod).")
