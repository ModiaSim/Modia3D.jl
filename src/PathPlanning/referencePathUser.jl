# is for scheduling robot reference motion (not a must at the moment (no channels))
function scheduleReferenceMotion(ref::ReferencePath,
                                 referenceMotion!::Function)
    @error("von scheduleReferenceMotion Zeile 4")
    referenceMotion!(ref)
end

scheduleReferenceMotion(; referencePath, referenceMotion) = scheduleReferenceMotion(referencePath, referenceMotion)


#-------------- Commands that can be used in referenceMotion programs -----
### ------------------------- ptpJointSpace -------------------------------

function ptpJointSpace(ref::ReferencePath, positions::Union{Matrix{Float64}, Vector{Float64}, Float64}, iargs...; kwargs...)
    checkErrorPtpJointSpace(iargs, kwargs)
    return push!(ref.refMotion, ArbitraryMotion(ptpJointSpace!, positions, nothing, nothing))
end
ptpJointSpace(;referencePath, positions) = ptpJointSpace(referencePath, positions)
ptpJointSpace(positions::Union{Matrix{Float64}, Vector{Float64}, Float64}, ref::ReferencePath, iargs...; kwargs...) =
ptpJointSpace(ref, positions, iargs...; kwargs...)
ptpJointSpace(ref::ReferencePath, iargs...; kwargs...) =
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

### ------------------- calculateRobotMovement -----------------------
# is called from computeSignal or computeForceElement


calculateRobotMovement(; referencePath, instantiatedModel) = calculateRobotMovement(referencePath, instantiatedModel)


function calculateRobotMovement(referencePath, instantiatedModel)
    referencePath.instantiatedModel = instantiatedModel
    if ModiaLang.isEvent(instantiatedModel)
        if ModiaLang.getTime(instantiatedModel) < referencePath.nextEventTime
            # Reschedule next time event
            ModiaLang.setNextEvent!(instantiatedModel, referencePath.nextEventTime)
        else
            # At the current event instant a new command has to be processed
            # Execute next command in referenceMotion
            if !isempty(referencePath.refMotion)
                item = popfirst!(referencePath.refMotion)
                item.func(referencePath, item.inputArg1, item.inputArg2, item.inputArg3)
    end; end; end

    # only if a ptp path is defined getPosition! can be executed
    t = ModiaLang.getTime(instantiatedModel)
    #if t >= 0.1 && t <= 0.2
    #    println("... time = ", t, ", ptpPath = ", referencePath.ptpPath)
    #end
    if !isnothing(referencePath.ptpPath)
        getPosition!(referencePath.ptpPath, ModiaLang.getTime(instantiatedModel),
            referencePath.position,
            referencePath.velocity,
            referencePath.acceleration)
        #println("... time = ", t, ", position = ", referencePath.position)
    end
    return referencePath
end
