#-------------- Commands that can be used in referenceMotion programs -----
### ------------------------- ptpJointSpace -------------------------------

function ptpJointSpace(ref::ModelActions{F,TimeType}, positions::Union{Matrix{Float64}, Vector{Float64}, Float64}, iargs...; kwargs...) where {F <: Modia3D.VarFloatType, TimeType <: AbstractFloat}
    checkErrorPtpJointSpace(iargs, kwargs)
    push!(ref.refMotion, ArbitraryMotion(ptpJointSpace!, positions, nothing, nothing))
    return nothing
end
ptpJointSpace(;referencePath, positions) =
    ptpJointSpace(referencePath, positions)
ptpJointSpace(positions::Union{Matrix{Float64}, Vector{Float64}, Float64}, ref::ModelActions{F,TimeType}, iargs...; kwargs...) where {F <: Modia3D.VarFloatType, TimeType <: AbstractFloat} =
    ptpJointSpace(ref, positions, iargs...; kwargs...)
ptpJointSpace(ref::ModelActions{F,TimeType}, iargs...; kwargs...) where {F <: Modia3D.VarFloatType, TimeType <: AbstractFloat} =
    checkErrorPtpJointSpace(iargs, kwargs)

#
# --------- check errors with might occur by the user -------------------
function checkErrorPtpJointSpace(iargs, kwargs)
    if length(iargs) > 0
        error("... from ptpJointSpace: first input argument must be a referencePath, second input argument must be position. No further or less input arguments are allowed.")
    end
    if length(kwargs) > 0
        error("... from ptpJointSpace: no keyword arguments are allowed. You used $kwargs.")
    end
    return nothing
end

### ------------------- executeActions -----------------------
executeActions(; referencePath, instantiatedModel, worldName) = executeActions(referencePath, instantiatedModel, worldName)


function executeActions(modelActions::ModelActions{F,TimeType}) where {F <: Modia3D.VarFloatType, TimeType <: AbstractFloat}
    if Modia.isEvent(modelActions.instantiatedModel)
        if Modia.getTime(modelActions.instantiatedModel) < modelActions.nextEventTime
            # Reschedule next time event
            Modia.setNextEvent!(modelActions.instantiatedModel, modelActions.nextEventTime)
        else
            # At the current event instant a new command has to be processed
            # Execute next command in referenceMotion
            if !isempty(modelActions.refMotion)
                item = popfirst!(modelActions.refMotion)
                item.func(modelActions, item.inputArg1, item.inputArg2, item.inputArg3)
    end; end; end

    # only if a ptp path is defined getPosition! can be executed
    if isdefined(modelActions, :ptpPath)
        getPosition!(modelActions.ptpPath, Modia.getTime(modelActions.instantiatedModel),
            modelActions.referencePath.position,
            modelActions.referencePath.velocity,
            modelActions.referencePath.acceleration)
    end
    return modelActions
end



### ------------------------- ActionWait ------------------------------
function ActionWait(ref::ModelActions{F,TimeType}, waitingPeriod::Float64=0.0, iargs...;  kwargs...)::Nothing where {F <: Modia3D.VarFloatType, TimeType <: AbstractFloat}
    checkErrorExplicitRobotFunc("ActionWait", "waitingPeriod", iargs, kwargs, refPathExists=true)
    push!(ref.refMotion, ArbitraryMotion(ActionWait!, waitingPeriod, nothing, nothing))
    return nothing
end
ActionWait(iargs...; kwargs...)::Nothing =
    checkErrorExplicitRobotFunc("ActionWait", "waitingPeriod", iargs, kwargs, refPathExists=false)


### --------------------- ActionReleaseAndAttach -------------------------
function ActionReleaseAndAttach(ref::ModelActions{F,TimeType}, movablePart::String="", robotOrDepot::String="", iargs...; waitingPeriod::Float64=0.0, kwargs...)::Nothing where {F <: Modia3D.VarFloatType, TimeType <: AbstractFloat}
    checkErrorExplicitRobotFunc("ActionReleaseAndAttach", "robotOrDepot, movablePart, and waitingPeriod", iargs, kwargs, refPathExists=true)
    push!(ref.refMotion, ArbitraryMotion(ActionReleaseAndAttach!, robotOrDepot, movablePart, waitingPeriod))
    return nothing
end
ActionReleaseAndAttach(iargs...; kwargs...)::Nothing =
    checkErrorExplicitRobotFunc("ActionReleaseAndAttach", "robotOrDepot, movablePart, and waitingPeriod", iargs, kwargs, refPathExists=false)

function EventAfterPeriod(ref::ModelActions{F,TimeType}, waitingPeriod::Float64=0.0, iargs...;  kwargs...)::Nothing where {F <: Modia3D.VarFloatType, TimeType <: AbstractFloat}
    checkErrorExplicitRobotFunc("EventAfterPeriod", "waitingPeriod", iargs, kwargs, refPathExists=true)
    push!(ref.refMotion, ArbitraryMotion(robotEventAfterPeriod!, waitingPeriod, nothing, nothing))
    return nothing
end
EventAfterPeriod(iargs...; kwargs...)::Nothing =
    checkErrorExplicitRobotFunc("EventAfterPeriod", "waitingPeriod", iargs, kwargs, refPathExists=false)


### --------------------- ActionAttach ------------------------------------
function ActionAttach(ref::ModelActions{F,TimeType}, movablePart::String="", robotOrDepot::String="", iargs...; waitingPeriod::Float64=0.0, kwargs...)::Nothing where {F <: Modia3D.VarFloatType, TimeType <: AbstractFloat}
    checkErrorExplicitRobotFunc("ActionAttach", "robotOrDepot, movablePart, and waitingPeriod", iargs, kwargs, refPathExists=true)
    push!(ref.refMotion, ArbitraryMotion(ActionAttach!, robotOrDepot, movablePart, waitingPeriod))
    return nothing
end
ActionAttach(iargs...; kwargs...)::Nothing =
    checkErrorExplicitRobotFunc("ActionAttach", "robotOrDepot, movablePart, and waitingPeriod", iargs, kwargs, refPathExists=false)


### --------------------- ActionRelease -----------------------------------
function ActionRelease(ref::ModelActions{F,TimeType}, movablePart::String="", robotOrDepot::String="", iargs...; waitingPeriod::Float64=0.0, kwargs...)::Nothing where {F <: Modia3D.VarFloatType, TimeType <: AbstractFloat}
    checkErrorExplicitRobotFunc("ActionRelease", "robotOrDepot, movablePart, and waitingPeriod", iargs, kwargs, refPathExists=true)
    push!(ref.refMotion, ArbitraryMotion(ActionRelease!, movablePart, waitingPeriod, nothing))
    return nothing
end
ActionRelease(iargs...; kwargs...)::Nothing =
    checkErrorExplicitRobotFunc("ActionRelease", "robotOrDepot, movablePart, and waitingPeriod", iargs, kwargs, refPathExists=false)



### --------------------- ActionRelease -----------------------------------
function ActionDelete(ref::ModelActions{F,TimeType}, movablePart::String="", robotOrDepot::String="", iargs...; waitingPeriod::Float64=0.0, kwargs...)::Nothing where {F <: Modia3D.VarFloatType, TimeType <: AbstractFloat}
    checkErrorExplicitRobotFunc("ActionDelete", "robotOrDepot, movablePart, and waitingPeriod", iargs, kwargs, refPathExists=true)
    push!(ref.refMotion, ArbitraryMotion(ActionDelete!, movablePart, waitingPeriod, nothing))
    return nothing
end
ActionDelete(iargs...; kwargs...)::Nothing =
    checkErrorExplicitRobotFunc("ActionDelete", "robotOrDepot, movablePart, and waitingPeriod", iargs, kwargs, refPathExists=false)


function checkErrorExplicitRobotFunc(funcName::String, keywordNames::String, iargs, kwargs; refPathExists=false)::Nothing
    if refPathExists
        if length(iargs) > 0
            error("... from $funcName: first input argument must be a referencePath. No further input arguments are allowed.")
        end
    else
        error("... from $funcName: first input argument must be a referencePath. No further input arguments are allowed.")
    end
    if length(kwargs) > 0
        error("... from $funcName: use $keywordNames as keyword arguments. You used $kwargs.")
    end
    return nothing
end
