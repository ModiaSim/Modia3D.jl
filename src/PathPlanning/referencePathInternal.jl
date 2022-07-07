struct ArbitraryMotion
    # e.g. ptpJointSpace!
    func::Function
    inputArg1       # e.g. positions, waitingPeriod, robotOrDepot
    inputArg2       # e.g. nothing, movablePart
    inputArg3       # e.g. nothing, waitingPeriod
end

mutable struct ReferencePath
    names::Vector{AbstractString}     # names are set by the user
    initPos::Vector{Float64}          # initial positions
    position::Vector{Float64}         # current reference position
    velocity::Vector{Float64}         # current reference velocity
    acceleration::Vector{Float64}     # current reference acceleration
    ptpPath::Union{PTP_path,Nothing}  # current PTP_path definition
    nextEventTime::Float64            # time at which the next event is triggered
    v_max::Vector{Float64}            # max velocity is set by the user
    a_max::Vector{Float64}            # max acceleration is set by the user
    refMotion::Vector{ArbitraryMotion} # stores explicit robot functions

    # both are updated in calculateRobotMovement
    instantiatedModel::Modia.SimulationModel     # SimulationModel of model
    # scene::Composition.Scene                       # scene

    function ReferencePath(names::AbstractVector;
                position = zeros(size(names,1)),
                v_max    = ones(size(names,1)),
                a_max    = ones(size(names,2)))
        new(names, position, position, zeros(size(names,1)), zeros(size(names,1)), nothing, 0.001, v_max, a_max, ArbitraryMotion[])
    end
end

ReferencePath(; names, position, v_max, a_max) = ReferencePath(names, position=position, v_max=v_max, a_max=a_max)

getRefPathPosition(referencePath::ReferencePath, index)::Float64 = referencePath.position[index]
getRefPathPosition(; referencePath::ReferencePath, index)::Float64 = referencePath.position[index]

getRefPathInitPosition(referencePath::ReferencePath, index)::Float64 = referencePath.initPos[index]
getRefPathInitPosition(; referencePath::ReferencePath, index)::Float64 = referencePath.initPos[index]


## -------------------- utility functions ------------------------
function setAttributesReferencePath!(ref::ReferencePath, waitingPeriod::Float64)
    ref.nextEventTime = waitingPeriod
    Modia.setNextEvent!(ref.instantiatedModel, ref.nextEventTime)
end


# getComponent(referencePath::ReferencePath, path::String) =
#    getPathComponent(referencePath.simulationState.model.model.assembly, path)


#--------------  Functions that can be executed at an event instant -------
### ------------------ ptpJointSpace! -------------------------------------
function ptpJointSpace!(ref::ReferencePath, positions::Matrix{Float64}, dummy1, dummy2)
    pos1 = size(transpose(ref.position))
    pos2 = size(positions)
    if pos1[2] != pos2[2]
        error("... from ptpJointSpace: Please check out dimensions of input $positions (= $pos2). Its dimensions should be (n, $(pos1[2])), n > 0.")
    end
    ref.ptpPath = PTP_path(ref.names,
    positions = [transpose(ref.position); positions],
    startTime = Modia.getTime(ref.instantiatedModel),
    v_max     = ref.v_max,
    a_max     = ref.a_max)
    setAttributesReferencePath!(ref, pathEndTime(ref.ptpPath))
end

ptpJointSpace!(ref::ReferencePath, positions::Vector{Float64}, dummy1, dummy2) =
    ptpJointSpace!(ref, reshape(positions, :, 1), dummy1, dummy2)

ptpJointSpace!(ref::ReferencePath, positions::Float64, dummy1, dummy2) =
    ptpJointSpace!(ref, reshape([positions], :, 1), dummy1, dummy2)
