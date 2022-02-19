function getJointsAndForceElementsAndObject3DsWithoutParents!(evaluatedParameters,
                                                              object3DWithoutParents::Vector{Object3D{F}},
                                                              revoluteObjects::Vector{Object3D{F}},
                                                              prismaticObjects::Vector{Object3D{F}},                                                              
                                                              freeMotionObjects::Vector{Object3D{F}},
                                                              forceElements::Vector{Modia3D.AbstractForceElement},
                                                              path::String)::Nothing where F <: Modia3D.VarFloatType
    for (key,value) in evaluatedParameters
        #println("$path.$key = $value")
        if typeof(value) <: Object3D
            if value.parent === value
                if typeof(value.feature) <: Scene
                    push!(object3DWithoutParents, value)
                else
                    error("\n", value.path, " is an Object3D that has no parent, but no feature=Scene(..)!\n")
                end
            elseif typeof(value.feature) <: Scene
                error("\n", value.path, " is an Object3D that has feature=Scene(..) and has a parent (= ", value.parent.path, ")!\n")
            end

        elseif typeof(value) <: Modia3D.Composition.Revolute 
            push!(revoluteObjects, value.obj2)
            
        elseif typeof(value) <: Modia3D.Composition.Prismatic
            push!(prismaticObjects, value.obj2)

        elseif typeof(value) <: Modia3D.Composition.FreeMotion
            push!(freeMotionObjects, value.obj2)

        elseif typeof(value) <: Modia3D.AbstractForceElement
            push!(forceElements, value)

        elseif typeof(value) <: OrderedDict
            getJointsAndForceElementsAndObject3DsWithoutParents!(value, object3DWithoutParents, revoluteObjects, prismaticObjects, 
                                                                 freeMotionObjects, forceElements, path*"."*string(key))
        end
    end
    return nothing
end

multiBodyName(instantiatedModel, mbsPath) = mbsPath == "" ? instantiatedModel.modelName :
                                                            instantiatedModel.modelName * "." * mbsPath

"""
    (world, jointObjects, forceElements) = checkMultibodySystemAndGetWorldAndJointsAndForceElements(mbsRoot)

Recursively traverse model mbsRoot and perform the following actions:

- Check that from all Object3Ds exactly one of them has no parent and `feature = SceneOptions(..)`.
  Return this parent as `world`.
- Check that only `world` has a `feature` entry that is a SceneOption.
- Return (world, revoluteObjects, prismaticObjects, freeMotionObjects, forceElements)
"""
function checkMultibodySystemAndGetWorldAndJointsAndForceElements(mbsRoot, mbsPath::String, FloatType::Type)                                                               
    object3DWithoutParents = Object3D{FloatType}[]
    revoluteObjects   = Object3D{FloatType}[]
    prismaticObjects  = Object3D{FloatType}[]
    freeMotionObjects = Object3D{FloatType}[]
    forceElements     = Modia3D.AbstractForceElement[]
    getJointsAndForceElementsAndObject3DsWithoutParents!(mbsRoot, object3DWithoutParents, revoluteObjects, prismaticObjects, 
                                                         freeMotionObjects, forceElements, mbsPath)

    if length(object3DWithoutParents) == 0
        error("\n", multiBodyName(instantiatedModel, mbsPath), ": There is either no Object3D or all of them have a parent!\n",
              "(Note, there must be exactly one Object3D that has no parent and feature=Scene(..).)")
    elseif length(object3DWithoutParents) > 1
        object3DNames = "   " * object3DWithoutParents[1].path
        for i = 2:length(object3DWithoutParents)
            object3DNames *= "\n   " * object3DWithoutParents[i].path
        end
        error("\n", instantiatedModel.modelName, ": The following ", length(object3DWithoutParents), " Object3Ds have no parents and feature=Scene(..)\n",
              "(note, there must be exactly one Object3D that has no parent and feature=Scene(..)):\n", object3DNames, "\n")
    end
    world = object3DWithoutParents[1]
    return (world, revoluteObjects, prismaticObjects, freeMotionObjects, forceElements)
end



function initAnalysis2!(world)
    # use Scene(..) of world object
    if typeof(world.feature) <: Modia3D.Composition.Scene
        scene = world.feature
    else
        scene = Modia3D.Composition.Scene()
    end
    scene.analysis = Modia3D.DynamicAnalysis

    Modia3D.Composition.chooseAndBuildUpTree(world, scene)

    if !scene.options.enableContactDetection
        scene.collide = false
    end

    return scene
end


"""
    mbs = openModel3D!(instantiatedModel, modelPath::String, ndof::Int, time)

Open Model3D.
"""
function openModel3D!(instantiatedModel::ModiaLang.SimulationModel{F,TimeType}, modelPath::String, 
                      nqdd::Int, time)::MultibodyData{F,TimeType} where {F,TimeType}
     TimerOutputs.@timeit instantiatedModel.timer "Modia3D_0 initJoint!" begin
        mbsBuild::MultibodyBuild{F,TimeType} = instantiatedModel.buildDict[modelPath]
        if ModiaLang.isFirstInitialOfAllSegments(instantiatedModel)
            # Instantiate the Modia3D system
            mbsRoot = ModiaLang.getModelFromSplittedPath(instantiatedModel.evaluatedParameters, mbsBuild.Model3DSplittedPath)
            (world, revoluteObjects, prismaticObjects, freeMotionObjects, forceElements) = checkMultibodySystemAndGetWorldAndJointsAndForceElements(mbsRoot, modelPath, F)

            # Initialize force elements
            for force in forceElements
                initializeForceElement(force)
            end

            # Construct MultibodyData
            scene = initAnalysis2!(world)
            scene.forceElements = forceElements
            if scene.options.enableContactDetection
                nz = 2
                zStartIndex = ModiaLang.addZeroCrossings(instantiatedModel, nz)
                scene.zStartIndex = zStartIndex
            else
                nz = 0
                zStartIndex = 0
            end
            mbsBuild.mbs = MultibodyData{F,TimeType}(instantiatedModel, nqdd, world, scene, revoluteObjects, prismaticObjects, freeMotionObjects, zStartIndex, nz, time)

            if scene.visualize
                TimerOutputs.@timeit instantiatedModel.timer "Modia3D_0 initializeVisualization" Modia3D.Composition.initializeVisualization(Modia3D.renderer[1], scene.allVisuElements)
                if instantiatedModel.options.log
                    println(    "        Modia3D: nVisualShapes = ", length(scene.allVisuElements))
                    if scene.options.enableContactDetection
                        println("                 mprTolerance  = ", scene.options.contactDetection.tol_rel)
                        println("                 contact_eps   = ", scene.options.contactDetection.contact_eps)
                    end
                end
            end
        end
        mbs = mbsBuild.mbs
        mbs.time = time
    end
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
    scene   = mbs.scene
    objects = mbs.jointObjects1
    @assert(length(args) == length(objects))

    for (i,obj) in enumerate(objects)
        jointKind = obj.jointKind

        if jointKind == RevoluteKind
            revolute     = scene.revolute[obj.jointIndex]
            revolute.tau = args[i]

        elseif jointKind == PrismaticKind
            prismatic   = scene.prismatic[obj.jointIndex]
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
        leq = ModiaBase.LinearEquations{F}(x_names, x_lengths, length(objects), false)
        push!(mbs.leq, leq)
        push!(mbs.instantiatedModel.linearEquations, leq)  # Stored it also in instantiatedModel, in order to support DAE-mode
    else
        leq = mbs.leq[1]
    end
    leq.mode = -3
    m = mbs.instantiatedModel
    while ModiaBase.LinearEquationsIteration(leq, m.isInitial, m.solve_leq, m.storeResult, m.time, m.timer)
        # Store generalized accelerations in the joints
        a = leq.x
        for (i,obj) in enumerate(objects)
            jointKind = obj.jointKind

            if jointKind == RevoluteKind
                revolute   = scene.revolute[obj.jointIndex]
                revolute.a = a[i]

            elseif jointKind == PrismaticKind
                prismatic   = scene.prismatic[obj.jointIndex]
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
    of ModiaBase/src/EquationAndStateInfo.jl - Base.iterate(iterator::LinearEquationsIterator, ...)
    
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
    mbs = computeGeneralizedForces!(mbs::MultibodyData, _leq)

Given the states of the joints (provided via functions setStatesXXX(..)), compute
the generalized forces of the joints. The function returns mbs. The generalized forces
can be accessed afterwards via functions getGenForcesXXX

A typical computation sequence is:

```
# Called outside of the linear equation system
mbs1 = setStatesRevolute!(mbs , <...>)
mbs2 = setStatesPrismatic!(mbs1, <...>)
mbs3 = setStatesFreeMotion!(mbs2, <...>)
mbs4 = setStatesFreeMotion_isrot123!(mbs3, <...>)

tau1 = <...>
tau2 = <...>
f1   = <...>
f2   = <...>
<...>

# Called inside the linear equation system, after transforming getGenForces into residual form
while LinearEquationsIteration(_leq, <...>)
    <...>
    mbs5 = setAccelerationsRevolute!(mbs4, <...>)
    mbs6 = setAccelerationsPrismatic!(mbs5, <...>)
    mbs7 = setAccelerationsFreeMotion!(mbs6, <...>) 
    mbs8 = computeGeneralizedForces!(mbs7, _leq)
    
    # The following equations are transformed into residual form
    # (tau1, tau2, <...>) = getGenForcesRevolute(mbs8, Val(<number_of_revolute_joints>))
    # (f1, f2, <...>) = getGenForcesPrismatic(mbs8, Val(<number_of_prismatic_joints>))
    # (0,0, <..>) = getGenForcesFreeMotion(mbs8, Val(<number_of_freeMotion_joints>))
    appendVariable(_leq.residuals, getGenForcesRevolute(mbs8, Val(<number_of_revolute_joints>)) .- (tau1, tau2, <...>))
    appendVariable(_leq.residuals, getGenForcesPrismatic(mbs8, Val(<number_of_revolute_joints>)) .- (f1, f2, <...>))
    appendVariable(_leq.residuals, getGenForcesFreeMotion(mbs8, Val(<number_of_freeMotion_joints>)))        
end
```
"""
function computeGeneralizedForces!(mbs::MultibodyData{F}, _leq)::MultibodyData{F} where {F}
    instantiatedModel = mbs.instantiatedModel  
    TimerOutputs.@timeit instantiatedModel.timer "Modia3D computeGeneralizedForces!" begin
        storeResult   = instantiatedModel.storeResult
        leq_mode::Int = isnothing(_leq) ? -2 : _leq.mode         
        isTerminal    = ModiaLang.isTerminal(instantiatedModel) && leq_mode == -2
   
                                
        scene = mbs.scene
        world = mbs.world
        time  = mbs.time
        
        tree            = scene.treeForComputation
        forceElements   = scene.forceElements
        visualize       = scene.visualize   # && sim.model.visualiz
        exportAnimation = scene.exportAnimation
    
        if isTerminal  #if Modia.isTerminalOfAllSegments(sim)
            TimerOutputs.@timeit instantiatedModel.timer "Modia3D_4 isTerminal" begin
                for force in forceElements
                    terminateForceElement(force)
                end
                if exportAnimation
                    Modia3D.exportAnimation(scene)
                end
                if visualize
                    #println("... time = $time: visualization is closed")
                    closeVisualization(Modia3D.renderer[1])
                end
            end
            #if scene.collide
            #    closeContactDetection!(m.assembly)
            #end
            # Do not return, since otherwise the linear system of equations cannot be solved -> error
            #return nothing
        end
    
        # Initialize force/torque of world-frame
        world.f = Modia3D.ZeroVector3D(F)
        world.t = Modia3D.ZeroVector3D(F)
    
        # Computation depending on leq_mode (the mode of the LinearEquationsIterator)
        if leq_mode == 0 || leq_mode == -1 
            # Compute all variables as functions of q,qd.
            TimerOutputs.@timeit instantiatedModel.timer "Modia3D_1" begin
                # Compute kinematics
                TimerOutputs.@timeit instantiatedModel.timer "Modia3D_1 computePositionsVelocitiesAccelerations!" computePositionsVelocitiesAccelerations!(scene, tree, time)
    
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
                for force in forceElements
                    evaluateForceElement(force)
                end
    
                # Compute contact forces/torques
                if scene.collide
                    computeContactForcesAndTorques(instantiatedModel, scene, world, time, nothing)
                end
    
                # Compute forces/torques and residues in a backward recursion
                TimerOutputs.@timeit instantiatedModel.timer "Modia3D_1 computeObject3DForcesTorquesAndGenForces!" computeObject3DForcesTorquesAndGenForces!(mbs, tree, time)
                
                # Store GenForces in cache
                if leq_mode == 0  # qdd = 0 (!)
                    mbs.revoluteCache_h   .= mbs.revoluteGenForces
                    mbs.prismaticCache_h  .= mbs.prismaticGenForces
                    mbs.freeMotionCache_h .= mbs.freeMotionGenForces
                end
            end

        elseif leq_mode > 0
            # Compute only acceleration terms (all variables as functions of q,qd have been already computed)
            TimerOutputs.@timeit instantiatedModel.timer "Modia3D_2" begin
                # Compute   res := M(q)*e_i + h(q,qd)
                #           res := res + cache_h
    
                # Compute kinematics
                TimerOutputs.@timeit instantiatedModel.timer "Modia3D_2 computeAccelerations!" computeAccelerations!(scene, tree, time)
    
                TimerOutputs.@timeit instantiatedModel.timer "Modia3D_2 obj in tree" begin
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
    end
                # Compute forces/torques and residues in a backward recursion (h(q,qd,...) = 0)
                TimerOutputs.@timeit instantiatedModel.timer "Modia3D_2 computeObject3DForcesTorquesAndGenForces!" computeObject3DForcesTorquesAndGenForces!(mbs, tree,time)
                
                # Add GenForces from cache (computed with qdd=0)
                TimerOutputs.@timeit instantiatedModel.timer "Modia3D_2 cache" begin
                mbs.revoluteGenForces   .+= mbs.revoluteCache_h
                mbs.prismaticGenForces  .+= mbs.prismaticCache_h
                mbs.freeMotionGenForces .+= mbs.freeMotionCache_h
end                
            end
    
        elseif leq_mode == -2
            # Compute only terms needed at a communication point (currently: Only visualization + export animation)      
            TimerOutputs.@timeit instantiatedModel.timer "Modia3D_3" begin
                if storeResult && !isTerminal && (visualize || exportAnimation)
                    if abs(instantiatedModel.options.startTime + scene.outputCounter*instantiatedModel.options.interval - time) < 1.0e-6*(abs(time) + 1.0)
                        # Visualize at a communication point
                        scene.outputCounter += 1
                        #if sim.options.log
                        #    println("time = $time")
                        #end
                        # Compute positions of frames that are only used for visualization
                        TimerOutputs.@timeit instantiatedModel.timer "Modia3D_3 visualize!" begin
                            if scene.options.useOptimizedStructure
                                for obj in scene.updateVisuElements
                                    parent = obj.parent
                                    obj.r_abs = obj.r_rel ≡ Modia3D.ZeroVector3D(F) ? parent.r_abs : parent.r_abs + parent.R_abs'*obj.r_rel
                                    obj.R_abs = obj.R_rel ≡ Modia3D.NullRotation(F) ? parent.R_abs : obj.R_rel*parent.R_abs
    
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
                            if visualize
                                Modia3D.visualize!(Modia3D.renderer[1], time)
                            end
                        end
                        if exportAnimation
                            TimerOutputs.@timeit instantiatedModel.timer "Modia3D_3 exportAnimation" begin
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
    return mbs
end
