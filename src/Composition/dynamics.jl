function getJointsAndForceElementsAndResultElementsAndObject3DsWithoutParents!(evaluatedParameters::AbstractDict,
                                                                               object3DWithoutParents::Vector{Object3D{F}},
                                                                               revoluteObjects::Vector{Object3D{F}},
                                                                               prismaticObjects::Vector{Object3D{F}},
                                                                               freeMotionObjects::Vector{Object3D{F}},
                                                                               hiddenJointObjects::Vector{Object3D{F}},
                                                                               forceElements::Vector{Modia3D.AbstractForceElement},
                                                                               resultElements::Vector{Modia3D.AbstractResultElement},
                                                                               path::String)::Nothing where F <: Modia3D.VarFloatType
    for (key,value) in evaluatedParameters
        #println("$path.$key = $value")
        if typeof(value) <: Object3D
            if value.parent === value
                if typeof(value.feature) <: Scene
                    push!(object3DWithoutParents, value)
                else
                    error("\n", value.path, " is an Object3D that has no parent, but no feature=Scene(..)!\n(an Object3D that has no parent must have feature=Scene(..))")
                end
            elseif typeof(value.feature) <: Scene
                error("\n", value.path, " is an Object3D that has feature=Scene(..) and has a parent (= ", value.parent.path, ")!\nThe Object3D with feature=Scene(..) is not allowed to have a parent!")
            end
            if typeof(value.joint) <: Modia3D.Composition.FreeMotion && value.joint.hiddenStates
                push!(hiddenJointObjects, value)
            end

        elseif typeof(value) <: Modia3D.Composition.Revolute
            push!(revoluteObjects, value.obj2)

        elseif typeof(value) <: Modia3D.Composition.Prismatic
            push!(prismaticObjects, value.obj2)

        elseif typeof(value) <: Modia3D.Composition.FreeMotion
            push!(freeMotionObjects, value.obj2)

        elseif typeof(value) <: Modia3D.AbstractForceElement
            push!(forceElements, value)

        elseif typeof(value) <: Modia3D.AbstractResultElement
            push!(resultElements, value)

        elseif typeof(value) <: OrderedDict
            value1::OrderedDict = value
            getJointsAndForceElementsAndResultElementsAndObject3DsWithoutParents!(value1, object3DWithoutParents, revoluteObjects, prismaticObjects,
                                                                                  freeMotionObjects, hiddenJointObjects, forceElements, resultElements,
                                                                                  path*"."*string(key))
        end
    end
    return nothing
end

multiBodyName(instantiatedModelName, mbsPath) = mbsPath == "" ? instantiatedModelName :
                                                                instantiatedModelName * "." * mbsPath

"""
    (world, jointObjects, forceElements) = checkMultibodySystemAndGetWorldAndJointsAndForceElements(
        instantiatedModelName, mbsRoot, mbsPath, FloatType)

Recursively traverse model mbsRoot and perform the following actions:

- Check that from all Object3Ds exactly one of them has no parent and `feature = SceneOptions(..)`.
  Return this parent as `world`.
- Check that only `world` has a `feature` entry that is a SceneOption.
- Return (world, revoluteObjects, prismaticObjects, freeMotionObjects, hiddenJointObjects, forceElements)
"""
function checkMultibodySystemAndGetWorldAndJointsAndForceElementsAndResultElements(instantiatedModelName, mbsRoot, mbsPath::String, FloatType::Type)
    object3DWithoutParents = Object3D{FloatType}[]
    revoluteObjects    = Object3D{FloatType}[]
    prismaticObjects   = Object3D{FloatType}[]
    freeMotionObjects  = Object3D{FloatType}[]
    hiddenJointObjects = Object3D{FloatType}[]
    forceElements      = Modia3D.AbstractForceElement[]
    resultElements     = Modia3D.AbstractResultElement[]
    getJointsAndForceElementsAndResultElementsAndObject3DsWithoutParents!(mbsRoot, object3DWithoutParents, revoluteObjects, prismaticObjects,
                                                                          freeMotionObjects, hiddenJointObjects, forceElements, resultElements,
                                                                          mbsPath)

    if length(object3DWithoutParents) == 0
        error("\n", multiBodyName(instantiatedModelName, mbsPath), ": There is either no Object3D or all of them have a parent!\n",
              "(Note, there must be exactly one Object3D that has no parent and feature=Scene(..).)")
    elseif length(object3DWithoutParents) > 1
        object3DNames = "   " * object3DWithoutParents[1].path
        for i = 2:length(object3DWithoutParents)
            object3DNames *= "\n   " * object3DWithoutParents[i].path
        end
        error("\n", instantiatedModelName, ": The following ", length(object3DWithoutParents), " Object3Ds have no parents and feature=Scene(..)\n",
              "(note, there must be exactly one Object3D that has no parent and feature=Scene(..)):\n", object3DNames, "\n")
    end
    world = object3DWithoutParents[1]
    return (world, revoluteObjects, prismaticObjects, freeMotionObjects, hiddenJointObjects, forceElements, resultElements)
end



function initAnalysis2!(world::Object3D{F}, timer::TimerOutputs.TimerOutput)::Scene{F} where F <: Modia3D.VarFloatType
    # use Scene(..) of world object
    if typeof(world.feature) <: Modia3D.Composition.Scene
        scene::Scene{F} = world.feature
    else
        error("Internal error of Modia3D: typeof(world.feature) = ", typeof(world.feature), ", but must be Modia3D.Composition.Scene")
    end
    scene.analysis = Modia3D.DynamicAnalysis
    scene.timer    = timer
    # rebuild tree
    Modia3D.Composition.chooseAndBuildUpTree(world, scene)

    if !scene.options.enableContactDetection
        scene.collide = false
    end

    return scene
end


"""
    initSegment_Model3D!(partiallyInstantiatedModel::Modia.InstantiatedModel,
                         modelPath::String, ID, parameters::AbstractDict; log=false)

Called once before initialization of a new simulation segment to instantiate/re-initialize a Model3D model
"""
function initSegment_Model3D!(partiallyInstantiatedModel::Modia.InstantiatedModel{F,TimeType},
                              modelPath::String, ID, parameters::AbstractDict; log=false)::Nothing where {F <: Modia3D.VarFloatType, TimeType <: AbstractFloat}
    TimerOutputs.@timeit partiallyInstantiatedModel.timer "Modia3D_0 initSegment_Model3D!" begin
        if log
            println("Modia3D.initSegment_Model3D! called for path=\"$modelPath\":")
        end

        mbsBuild::MultibodyBuild{F,TimeType} = Modia.get_instantiatedSubmodel(partiallyInstantiatedModel, ID)
        # hier Gripping Sachen

        if isnothing(mbsBuild.mbs) || partiallyInstantiatedModel.nsegments == 1
            firstSegment = true
            (world, revoluteObjects, prismaticObjects, freeMotionObjects, hiddenJointObjects, forceElements, resultElements) = checkMultibodySystemAndGetWorldAndJointsAndForceElementsAndResultElements(partiallyInstantiatedModel.modelName, parameters, modelPath, F)

            # Set timer in scene (so that timer is easily available in Modia3D functions)

            # Initialize force elements
            for force in forceElements
                initializeForceElement(partiallyInstantiatedModel, force)
            end

            # Construct MultibodyData
            scene = initAnalysis2!(world, partiallyInstantiatedModel.timer)
        else
            firstSegment = false
            world::Object3D{F} = mbsBuild.mbs.world
            scene::Scene{F} = mbsBuild.mbs.scene
            #instantiatedModel = mbsBuild.mbs.instantiatedModel
            #gripPair = scene.gripPair
            if isdefined(scene, :gripPair)
                if Modia3D.checkGrippingFeatures(scene, scene.gripPair)
                    Modia3D.changeParentOfMovableUnit!(scene, world, scene.gripPair)
                    if isdefined(scene.gripPair, :enableContactDetection)
                        scene.collide = scene.gripPair.enableContactDetection
                    end
                else
                    @error("Print warning für gripping features")
                    # printWarnGrip(robotOrDepot, movableObj, waitingPeriod)
                end
            end

            (worldDummy, revoluteObjects, prismaticObjects, freeMotionObjects, hiddenJointObjects, forceElements, resultElements) = checkMultibodySystemAndGetWorldAndJointsAndForceElementsAndResultElements(partiallyInstantiatedModel.modelName, parameters, modelPath, F)

            # Set timer in scene (so that timer is easily available in Modia3D functions)

            # Initialize force elements
            for force in forceElements
                initializeForceElement(partiallyInstantiatedModel, force)
            end

            # Construct MultibodyData
            # rebuild MKS
            if scene.options.useOptimizedStructure
                rebuild_superObjs!(scene, world)

                if isdefined(scene, :gripPair)
                    if Modia3D.checkGrippingFeatures(scene, scene.gripPair)
                        if isdefined(scene.gripPair, :enableContactDetection)
                            if scene.options.enableContactDetection && scene.collide
                                scene.collide = scene.gripPair.enableContactDetection
                            end
                        end
                    end
                end

            else
                error("Full restart is possible only if useOptimizedStructure is enabled in SceneOptions.")
            end
        end

        # Initialize result elements
        for result in resultElements
            initializeResultElement(partiallyInstantiatedModel, result)
        end

        scene.forceElements = forceElements
        scene.resultElements = resultElements
        if scene.options.enableContactDetection && scene.collide
            nz = 2
            zStartIndex = Modia.new_z_segmented_variable!(partiallyInstantiatedModel, nz)
            scene.zStartIndex = zStartIndex
        else
            nz = 0
            zStartIndex = 0
        end

        # objIndices[i,1]: Index of r_abs of Object3D i
        #           [i,2]: Index of R_abs of Object3D i
        objIndices = Matrix{Int}(undef, length(scene.updateVisuElements), 2)
        for (i,obj) in enumerate(scene.updateVisuElements)
            if typeof(obj.feature) == Modia3D.Composition.EmptyObject3DFeature
                objIndices[i,1] = 0
                objIndices[i,2] = 0
            else
                objIndices[i,1] = Modia.new_w_segmented_variable!(partiallyInstantiatedModel, obj.path*".r_abs", Modia3D.ZeroVector3D(F), "m")
                objIndices[i,2] = Modia.new_w_segmented_variable!(partiallyInstantiatedModel, obj.path*".R_abs", Modia3D.NullRotation(F), "")
            end
        end

        mbsBuild.mbs = MultibodyData{F,TimeType}(partiallyInstantiatedModel, modelPath, world, scene,
                                                 revoluteObjects, prismaticObjects, freeMotionObjects, hiddenJointObjects,
                                                 mbsBuild.revoluteIndices, mbsBuild.prismaticIndices, mbsBuild.freeMotionIndices,
                                                 objIndices, zStartIndex, nz)
        parameters[:qdd_hidden] = zeros(F, length(mbsBuild.mbs.hiddenGenForces))

        if log
            mbs = mbsBuild.mbs
            println("  Number of degrees of freedom: ", length(mbs.revoluteObjects) + length(mbs.prismaticObjects) + 6*length(mbs.hiddenJointObjects))
            println("  Number of revolute joints:    ", length(mbs.revoluteObjects))
            println("  Number of prismatic joints:   ", length(mbs.prismaticObjects))
            println("  Number of freeMotion joints:  ", length(mbs.freeMotionObjects))
            println("  Number of Object3Ds with fixedToParent=false: ", length(mbs.hiddenJointObjects))
        end

        if scene.visualize && firstSegment
            TimerOutputs.@timeit partiallyInstantiatedModel.timer "Modia3D_0 initializeVisualization" Modia3D.Composition.initializeVisualization(Modia3D.renderer[1], scene)
            if partiallyInstantiatedModel.options.log
                println(    "        Modia3D: nVisualShapes = ", length(scene.allVisuElements))
                if scene.options.enableContactDetection
                    println("                 mprTolerance  = ", scene.options.contactDetection.tol_rel)
                    println("                 contact_eps   = ", scene.options.contactDetection.contact_eps)
                end
            end
        end
    end
    return nothing
end



"""
    mbs = openModel3D!(instantiatedModel, modelPath::String, x, time)

Open Model3D:

- Get mbs data structure from buildDict.
- Copy hidden states into mbs data structure.
- Copy der(r):=v and der(rot):= f(w) into hidden derivatives.
- Return mbs.
"""
function openModel3D!(instantiatedModel::Modia.InstantiatedModel{F,TimeType}, modelPath::String, x::AbstractVector, time::TimeType)::MultibodyData{F,TimeType} where {F <: Modia3D.VarFloatType, TimeType <: AbstractFloat}
    # println("bin in openModel3D ", instantiatedModel.eventHandler.restart)
    mbsBuild::MultibodyBuild{F,TimeType} = instantiatedModel.buildDict[modelPath]

    if isnothing(mbsBuild.mbs)
        error("Error in Modia3D.openModel3D!: initSegment_Model3D!(..) was not called.\nShould have been called during evaluation of the parameters for $modelPath!!!")
    end
    @assert(instantiatedModel === mbsBuild.mbs.instantiatedModel)
    mbs = mbsBuild.mbs
    mbs.time = time

    setStatesHiddenJoints!(instantiatedModel, mbs, x)
    return mbs
end


#=
"""
    getJointResiduals_method3!(mbs, args...)

Return joint accelerations vector computed with buildModia3D(model; method=3).
args... are the joint forces of 1D joints
"""
function getJointResiduals_method3!(mbs::MultibodyData{F}, args::Vararg{F,N})::Vector{F} where {F,N}
    # Store generalized forces in joints
    objects = mbs.jointObjects1
    @assert(length(args) == length(objects))

    for (i,obj) in enumerate(objects)
        jointKind = obj.jointKind

        if jointKind == RevoluteKind
            revolute     = mbs.revolute[obj.jointIndex]
            revolute.tau = args[i]

        elseif jointKind == PrismaticKind
            prismatic   = mbs.prismatic[obj.jointIndex]
            prismatic.f = args[i]

        else
           error("Bug in Modia3D.getJointResiduals_method3!: jointKind = $jointKind is not allowed")
        end
    end


    # Solve residual = M(q)*qdd + h(q,qd) - u
    if length(mbs.leq) == 0
        # Allocate memory for the linear equation system
        x_names = String[]
        for obj in objects
            if obj.jointKind == RevoluteKind
                name = "der("*obj.joint.path*".w)"
            else
                name = obj.joint.path
            end
            push!(x_names, name)
        end
        x_lengths = fill(1,length(objects))
        leq = Modia.LinearEquations{F}(x_names, x_lengths, length(objects), false)
        push!(mbs.leq, leq)
        push!(mbs.instantiatedModel.linearEquations, leq)  # Stored it also in instantiatedModel, in order to support DAE-mode
    else
        leq = mbs.leq[1]
    end
    leq.mode = -3
    m = mbs.instantiatedModel
    while Modia.LinearEquationsIteration(leq, m.isInitial, m.solve_leq, m.storeResult, m.time, m.timer)
        # Store generalized accelerations in the joints
        a = leq.x
        for (i,obj) in enumerate(objects)
            jointKind = obj.jointKind

            if jointKind == RevoluteKind
                revolute   = mbs.revolute[obj.jointIndex]
                revolute.a = a[i]

            elseif jointKind == PrismaticKind
                prismatic   = mbs.prismatic[obj.jointIndex]
                prismatic.a = a[i]

            else
                error("Bug in Modia3D.getJointResiduals_method3!: jointKind = $jointKind is not allowed")
            end
        end

        # Compute and copy the residuals
        leq.residuals .= Modia3D.getJointResiduals_method2!(mbs, leq; cacheWithJointForces=true)
    end

    # Return the joint accelerations
    return leq.x
end
=#

   #= Efficiency improvement, by computing only terms that are needed in the respective phase
    of Modia/src/EquationAndStateInfo.jl - Base.iterate(iterator::LinearEquationsIterator, ...)

    nf: Number of degrees of freedom
    q: Generalized joint coordinates of the tree joints
    res: Residuals
    e_i: i-th unit vector of length(q)

    qd  = der(q)
    qdd = der(qd)
    res = M(q)*qdd + h(q,qd,gravity,contact forces)

    computeGeneralizedForces!(..., qdd, ...) is called inside the LinearEquationsIterator to solve a linear equation system:

        res := A*x - b

    that is called in the following way:

        leq_mode = 0        : Return "res := -b"        or "res :=            h(q,qd,gravity,contact forces)"
        leq_mode = i (1..nf): Return "res := A*e_i - b" or "res := M(q)*e_i + h(q,qd,gravity,contact forces)"
        leq_mode = -1       : Return "res := A*x - b"   or "res := 0.0"   # since residual is not used, it need not be computed.

    For Modia3D:
        cache_h = h(q,qd,gravity,contact forces)

        leq_mode =  0: Compute only terms that depend on q, qd, gravity and contact forces.
                    All position and velocity terms, such as Rrel and Rabs are computed
                    and stored in Object3D.
                    res := h(q,qd,gravity)   # qdd = 0
                    cache_h := res
                    return res

        leq_mode =  i: res := M(q)*qdd         # qdd = e_i
                    return res := res + cache_h
    =#



"""
    genForces = computeGeneralizedForces!(mbs::MultibodyData, qdd_hidden, _leq)

Given the states of the joints (provided via functions setStatesXXX(..)), compute
the generalized forces of the joints. The function returns genForces, that is the
vector of generalized forces of all joints with exception of free motion joints
(currently: generalized forces of revolute and prismatic joints).

A typical computation sequence is:

```
# Called outside of the linear equation system
mbs1 = Modia3D.openModel3D!(instantiatedModel, modelPathAsString, time)
mbs2 = Modia3D.setStatesRevolute!(mbs1, jointStatesRevolute...)
mbs3 = Modia3D.setStatesPrismatic!(mbs2, jointStatesPrismatic...)
mbs4 = Modia3D.setStatesFreeMotion!(mbs3, jointStatesFreeMotion...)
mbs5 = Modia3D.setStatesFreeMotion_isrot123!(mbs4, jointStatesFreeMotion_isrot123...)

tau1 = <...>
tau2 = <...>
f1   = <...>
f2   = <...>
<...>

# Called inside the linear equation system, after transforming getGenForces into residual form
while LinearEquationsIteration(_leq, <...>)
    <...>
    mbs6 = Modia3D.setAccelerationsRevolute!  (mbs5, jointAccelerationsRevolute...)
    mbs7 = Modia3D.setAccelerationsPrismatic!( mbs6, jointAccelerationsPrismatic...)
    mbs8 = Modia3D.setAccelerationsFreeMotion!(mbs7, jointAccelerationsFreeMotion2...)
    (mbs9,0.0) = Modia3D.computeGeneralizedForces!(mbs8, qdd_hidden, _leq)
    (jointForcesRevolute...)  = implicitDependency(Modia3D.getGenForcesRevolute(  mbs9, Val(NRevolute))   , jointAccelerationsRevolute...))
    (jointForcesPrismatic...) = implicitDependency(Modia3D.getGenForcesPrismatic( mbs9, Val(NPrismatic))  , jointAccelerationsPrismatic...))
    (0.0, 0.0, ...)           = implicitDependency(Modia3D.getGenForcesFreeMotion(mbs9, Val(NFreeMotion2)), jointAccelerationsFreeMotion2...))

    # The following equations are transformed into residual form
    # (tau1, tau2, <...>) = Modia3D.getGenForcesRevolute(mbs9, Val(<number_of_revolute_joints>))
    # (f1, f2, <...>)     = Modia3D.getGenForcesPrismatic(mbs9, Val(<number_of_prismatic_joints>))
    # (0,0, <..>)         = Modia3D.getGenForcesFreeMotion(mbs9, Val(<number_of_freeMotion_joints>))
    appendVariable(_leq.residuals, Modia3D.getGenForcesRevolute(  mbs9, Val(<number_of_revolute_joints>)) .- (tau1, tau2, <...>))
    appendVariable(_leq.residuals, Modia3D.getGenForcesPrismatic (mbs9, Val(<number_of_revolute_joints>)) .- (f1, f2, <...>))
    appendVariable(_leq.residuals, Modia3D.getGenForcesFreeMotion(mbs9, Val(<number_of_freeMotion_joints>)))
end
```
"""
function computeGeneralizedForces!(mbs::MultibodyData{F,TimeType}, qdd_hidden::Vector{F}, _leq)::Vector{F} where {F <: Modia3D.VarFloatType, TimeType <: AbstractFloat}
    instantiatedModel = mbs.instantiatedModel

    TimerOutputs.@timeit instantiatedModel.timer "Modia3D computeGeneralizedForces!" begin
        storeResult   = instantiatedModel.storeResult
        leq_mode::Int = isnothing(_leq) ? -2 : _leq.mode
        isTerminalOfAllSegments::Bool = Modia.isTerminalOfAllSegments(instantiatedModel) && leq_mode == -2

        scene = mbs.scene
        world = mbs.world
        time  = mbs.time
        setAccelerationsHiddenJoints!(mbs, qdd_hidden)

        tree            = scene.treeForComputation
        forceElements   = scene.forceElements
        resultElements  = scene.resultElements
        visualize       = scene.visualize   # && sim.model.visualiz
        exportAnimation = scene.exportAnimation
        provideAnimationData = scene.provideAnimationData

        if isTerminalOfAllSegments
            TimerOutputs.@timeit instantiatedModel.timer "Modia3D_4 isTerminalOfAllSegments" begin
                for force in forceElements
                    terminateForceElement(force)
                end
                for result in resultElements
                    terminateResultElement(result)
                end
                if exportAnimation
                    TimerOutputs.@timeit instantiatedModel.timer "Modia3D_4b exportAnimation" Modia3D.exportAnimation(scene)
                end
                if visualize
                    #println("... time = $time: visualization is closed")
                    closeVisualization(Modia3D.renderer[1])
                end
            end
            #if scene.collide
            #    closeContactDetection!(m.assembly)
            #end
        end

        # Initialize force/torque of world-frame
        world.f = Modia3D.ZeroVector3D(F)
        world.t = Modia3D.ZeroVector3D(F)

        # Computation depending on leq_mode (the mode of the LinearEquationsIterator)
        if leq_mode == 0 || leq_mode == -1
            # Compute all variables as functions of q,qd.
            TimerOutputs.@timeit instantiatedModel.timer "Modia3D_1" begin
                # Compute kinematics
                TimerOutputs.@timeit instantiatedModel.timer "Modia3D_1 computePositionsVelocitiesAccelerations!" computePositionsVelocitiesAccelerations!(mbs, tree, time)

                # Compute mass/inertia forces in a forward recursion and initializes forces/torques
                for obj in tree
                    if obj.hasMass
                        # Compute inertia forces / torques
                        w     = obj.w
                        grav  = gravityAcceleration(scene.options.gravityField, obj.r_abs)
                        obj.f = -obj.m*( obj.R_abs*(obj.a0 - grav) + cross(obj.z, obj.r_CM) + cross(w, cross(w, obj.r_CM)))
                        obj.t = -(obj.I_CM*obj.z + cross(w, obj.I_CM*w)) + cross(obj.r_CM, obj.f)
                    else
                        obj.f = Modia3D.ZeroVector3D(F)
                        obj.t = Modia3D.ZeroVector3D(F)
                    end
                end # end forward recursion

                # Evaluate force elements
                TimerOutputs.@timeit instantiatedModel.timer "Modia3D_1 evaluateForceElements" begin
                    for force in forceElements
                        evaluateForceElement(instantiatedModel, force, time)
                    end
                end

                # Compute contact forces/torques
                if scene.collide
                    computeContactForcesAndTorques(instantiatedModel, scene, world, time, nothing)
                end

                # Compute forces/torques and residues in a backward recursion
                TimerOutputs.@timeit instantiatedModel.timer "Modia3D_1 computeObject3DForcesTorquesAndGenForces!" computeObject3DForcesTorquesAndGenForces!(mbs, tree, time)

                if leq_mode == 0
                    # Store GenForces in cache (qdd = 0)
                    mbs.genForcesCache_h  .= mbs.genForces
                    mbs.freeMotionCache_h .= mbs.freeMotionGenForces
                    mbs.hiddenCache_h     .= mbs.hiddenGenForces
                elseif leq_mode == -1
                    # Store state derivatives
                    setHiddenStatesDerivatives!(instantiatedModel, mbs, mbs.genForces)
                end
            end

        elseif leq_mode > 0
            # Compute only acceleration terms (all variables as functions of q,qd have been already computed)
            TimerOutputs.@timeit instantiatedModel.timer "Modia3D_2" begin
                # Compute   res := M(q)*e_i + h(q,qd)
                #           res := res + cache_h

                # Compute kinematics
                TimerOutputs.@timeit instantiatedModel.timer "Modia3D_2 computeAccelerations!" computeAccelerations!(mbs, tree, time)

                for obj in tree
                    if obj.hasMass
                        # Compute inertia forces / torques
                        obj.f = -obj.m*( obj.R_abs*obj.a0 + cross(obj.z, obj.r_CM) )
                        obj.t = -obj.I_CM*obj.z + cross(obj.r_CM, obj.f)
                    else
                        obj.f = Modia3D.ZeroVector3D(F)
                        obj.t = Modia3D.ZeroVector3D(F)
                    end
                end # end forward recursion

                # Compute forces/torques and residues in a backward recursion (h(q,qd,...) = 0)
                TimerOutputs.@timeit instantiatedModel.timer "Modia3D_2 computeObject3DForcesTorquesAndGenForces!" computeObject3DForcesTorquesAndGenForces!(mbs, tree,time)

                # Add GenForces from cache (computed with qdd=0)
                mbs.genForces           .+= mbs.genForcesCache_h
                mbs.freeMotionGenForces .+= mbs.freeMotionCache_h
                mbs.hiddenGenForces     .+= mbs.hiddenCache_h
            end

        elseif leq_mode == -2
            # Compute only terms needed at a communication point (currently: Only visualization + export animation)
            TimerOutputs.@timeit instantiatedModel.timer "Modia3D_3" begin
                # objects can have interactionManner (need to rename updateVisuElements)
                if scene.options.useOptimizedStructure
                    objIndices = mbs.objIndices
                    for (i,obj) in enumerate(scene.updateVisuElements)
                        parent = obj.parent
                        obj.r_abs = obj.r_rel ≡ Modia3D.ZeroVector3D(F) ? parent.r_abs : parent.r_abs + parent.R_abs'*obj.r_rel
                        obj.R_abs = obj.R_rel ≡ Modia3D.NullRotation(F) ? parent.R_abs : obj.R_rel*parent.R_abs

                        if objIndices[i,1] > 0
                            Modia.copy_w_segmented_value_to_result(instantiatedModel, objIndices[i,1], obj.r_abs)
                            Modia.copy_w_segmented_value_to_result(instantiatedModel, objIndices[i,2], obj.R_abs)
                        end

                        # is executed only if an internal Object3D called
                        if length( obj.visualizationFrame ) == 1
                            obj.visualizationFrame[1].r_abs = obj.r_abs
                            obj.visualizationFrame[1].R_abs = obj.R_abs
                        end
                        for mesh in obj.fileMeshConvexShapes
                            mesh.r_abs = obj.r_abs
                            mesh.R_abs = obj.R_abs
                        end
                    end
                end

                if storeResult && !isTerminalOfAllSegments
                    # evaluate result elements
                    for result in resultElements
                        evaluateResultElement(instantiatedModel, scene, result, time)
                    end
                end

                if storeResult && !isTerminalOfAllSegments && (visualize || provideAnimationData)
                    if abs(instantiatedModel.options.startTimeFirstSegment + scene.outputCounter*instantiatedModel.options.interval - time) < 1.0e-6*(abs(time) + 1.0)
                        # Visualize at a communication point
                        scene.outputCounter += 1
                        #if sim.options.log
                        #    println("time = $time")
                        #end
                        # Compute positions of frames that are only used for visualization
                        TimerOutputs.@timeit instantiatedModel.timer "Modia3D_3 visualize!" begin

                            if visualize
                                Modia3D.visualize!(Modia3D.renderer[1], time)
                            end
                        end
                        if provideAnimationData
                            TimerOutputs.@timeit instantiatedModel.timer "Modia3D_3 provideAnimationData" begin
                                objectData = []
                                for obj in scene.allVisuElements
                                    pos = Modia3D.convertToFloat64(obj.r_abs)
                                    ori = Modia3D.from_R(Modia3D.convertToFloat64(obj.R_abs))
                                    dat = animationData(pos, ori)
                                    push!(objectData, dat)
                                end
                                aniStep = animationStep(time, objectData)
                                push!(scene.animation, aniStep)
                            end
                        end
                    end
                end
            end

        else
            error("Bug in dynamics.jl (computeGeneralizedForces!): leq_mode = $leq_mode (must be >= -1)")
        end
    end

    return mbs.genForces
end
