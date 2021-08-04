function getJointsAndObject3DsWithoutParents!(evaluatedParameters,
                                              object3DWithoutParents::Vector{Object3D},
                                              jointObjects::Vector{Object3D},
                                              path::String)::Nothing
    for (key,value) in evaluatedParameters   # zip(keys(evaluatedParameters), evaluatedParameters)
        if typeof(value) == Object3D
            if value.parent === value
                push!(object3DWithoutParents, value)
            elseif typeof(value.feature) == Modia3D.Composition.SceneOptions
                error(value.path, ": Object3D has a parent and defines SceneOptions!")
            end

        elseif typeof(value) <: Modia3D.AbstractJoint
            push!(jointObjects, value.obj2)

        elseif typeof(value) <: OrderedDict
            getJointsAndObject3DsWithoutParents!(value, object3DWithoutParents, jointObjects, path)
        end
    end
    return nothing
end

multiBodyName(instantiatedModel, mbsPath) = mbsPath == "" ? instantiatedModel.modelName :
                                                            instantiatedModel.modelName * "." * mbsPath

"""
    (world, jointObjects) = checkMultibodySystemAndGetWorldAndJoints(instantiatedModel, id)

Recursively traverse model and perform the following actions:

- Search for an OrderedDict that has `key = :_id, value = id`
  (identifies the root of the multibody system).

- Search from the root of the multibody system and perform the following actions:
  - Check that from all Object3Ds exactly one of them has no parent.
    Return this parent as `world`.
  - Check that only `world` has potentially a `feature` entry that
    is a SceneOption.
  - Return a vector of joint objects as `joints`.
"""
function checkMultibodySystemAndGetWorldAndJoints(instantiatedModel::ModiaLang.SimulationModel{FloatType,ParType,EvaluatedParType,TimeType}, id::Int) where {FloatType,ParType,EvaluatedParType,TimeType}
    # Find root mbs of multibody system
    (mbsRoot, mbsPath) = ModiaLang.getIdParameter(instantiatedModel.evaluatedParameters, ParType, id)
    if isnothing(mbsRoot)
        error(instantiatedModel.modelName, ": did not find _id = ", id, " in the evaluated parameters!")
    end

    object3DWithoutParents = Object3D[]
    jointObjects = Object3D[]
    getJointsAndObject3DsWithoutParents!(mbsRoot, object3DWithoutParents, jointObjects, mbsPath)

    if length(object3DWithoutParents) == 0
        error("\n", multiBodyName(instantiatedModel, mbsPath), ": There is either no Object3D or all of them have a parent!\n",
              "(Note, there must be exactly one Object3D that has no parent.)")
    elseif length(object3DWithoutParents) > 1
        object3DNames = "   " * object3DWithoutParents[1].path
        for i = 2:length(object3DWithoutParents)
            object3DNames *= "\n   " * object3DWithoutParents[i].path
        end
        error("\n", instantiatedModel.modelName, ": The following ", length(object3DWithoutParents), " Object3Ds have no parent\n",
              "(note, there must be exactly one Object3D that has no parent):\n", object3DNames, "\n")
    end
    return (object3DWithoutParents[1], jointObjects)
end



function initAnalysis2!(world)
    # Construct Scene(..) object
    Modia3D.Composition.EmptyObject3DFeature
    if typeof(world.feature) <: Modia3D.SceneOptions
        scene = Modia3D.Scene(world.feature)
    else
        scene = Modia3D.Scene()
    end
    scene.analysis = Modia3D.DynamicAnalysis

    Modia3D.Composition.chooseAndBuildUpTree(world, scene)

    if !scene.options.enableContactDetection
        scene.collide = false
    end

    return scene
end



struct MultibodyData{FloatType}
    nqdd::Int                       # Length of qdd vector
    world::Object3D                 # Pointer to world object
    scene::Scene                    # Pointer to scene
    jointObjects::Vector{Object3D}  # References to Object3Ds that have a joint
    jointStartIndex::Vector{Int}    # Start index of joint in qdd
    jointNdof::Vector{Int}          # Number-of-degrees-of-freedom of joint
    zStartIndex::Int                # eventHandler.z[zStartIndex] is first index of crossing function
                                    # (or zero, if nableContactDetection=false)
    nz::Int                         # Number of used zero crossing functions
    residuals::Vector{FloatType}    # Residuals - length(residuals) = nqdd
    cache_h::Vector{FloatType}      # Cached vector: = h(q,qd,gravity,contact-forces)
end



"""
    jointVariablesHaveValues = setModiaJointVariables!(_id, _leq_mode, instantiatedModel, time, args...)

Set generalized variables (q, qd, f) defined in the Modia model for all joints.
"""
function setModiaJointVariables!(id::Int, _leq_mode, instantiatedModel::ModiaLang.SimulationModel{FloatType}, time, args...)::Bool where {FloatType}
     TimerOutputs.@timeit instantiatedModel.timer "Modia3D_0" begin 
        separateObjects = instantiatedModel.separateObjects  # is emptied for every new simulate! call
        if haskey(separateObjects, id)
            mbs::MultibodyData{FloatType} = separateObjects[id]
            scene           = mbs.scene
            jointObjects    = mbs.jointObjects
            jointStartIndex = mbs.jointStartIndex
            jointNdof       = mbs.jointNdof
        else
            # Search in parameters and retrieve the name of the object with the required id
            # Instantiate the Modia3D system
            #mbsPar = getIdParameter(parameters, id)
            #mbsObj = instantiateDependentObjects(instantiatedModel.modelModule, mbsPar)
            (world, jointObjects) = checkMultibodySystemAndGetWorldAndJoints(instantiatedModel, id)
        
            # Construct startIndex vector and number of degrees of freedom per joint
            nJoints         = length(jointObjects)
            jointStartIndex = fill(0,nJoints)
            jointNdof       = fill(0,nJoints)
            j = 1
            for (i, jointObject) in enumerate(jointObjects)
                jointStartIndex[i] = j
                jointNdof[i]       = jointObject.joint.ndof
                j += jointNdof[i]
            end
            nqdd = j-1
            
            # Construct MultibodyData
            scene = initAnalysis2!(world)
            residuals = zeros(FloatType,nqdd)
            cache_h   = zeros(FloatType,nqdd)
            if scene.options.enableContactDetection
                nz = 2
                zStartIndex = ModiaLang.addZeroCrossings(instantiatedModel, nz)
            else
                nz = 0
                zStartIndex = 0
            end
            mbs = MultibodyData{FloatType}(nqdd, world, scene, jointObjects, jointStartIndex,
                                           jointNdof, zStartIndex, nz, residuals, cache_h)
            separateObjects[id] = mbs

            # Print
            if false
                printScene(scene)
            end
            
            if scene.visualize
                TimerOutputs.@timeit instantiatedModel.timer "Modia3D_0 initializeVisualization" Modia3D.Composition.initializeVisualization(Modia3D.renderer[1], scene.allVisuElements)
                if instantiatedModel.options.log
                    println("        Modia3D: Number of visual shapes = ", length(scene.allVisuElements))
                end
            end
        end
                    
        # Copy generalized position, velocity, and force values in to the joints
        @assert(length(args) == length(jointObjects))
        setJointVariables_q_qd_f!(scene, jointObjects, jointStartIndex, jointNdof, args)
    end
    return true
end


function multibodyResiduals!(id::Int, _leq_mode, instantiatedModel::ModiaLang.SimulationModel{FloatType}, time, jointVariablesHaveValues::Bool, qdd)::Vector{FloatType} where {FloatType}
     TimerOutputs.@timeit instantiatedModel.timer "Modia3D" begin 
        separateObjects = instantiatedModel.separateObjects  # is emptied for every new simulate! call
        if !haskey(separateObjects, id)
            error("Bug in Modia3D/src/composition/dynamics.jl: separateObjects[$id] is not defined.")
        end
        mbs::MultibodyData{FloatType} = separateObjects[id]
        @assert(length(qdd) == mbs.nqdd)
        world           = mbs.world
        scene           = mbs.scene
        jointObjects    = mbs.jointObjects
        jointStartIndex = mbs.jointStartIndex
        jointNdof       = mbs.jointNdof
        residuals       = mbs.residuals
        cache_h         = mbs.cache_h
   
        # Set generalized joint accelerations
        setJointVariables_qdd!(scene, jointObjects, jointStartIndex, jointNdof, qdd)
           
        # Compute residuals
        leq_mode = isnothing(_leq_mode) ? -1 : _leq_mode.mode
    
        # Method with improved efficiency
        multibodyResiduals3!(instantiatedModel, scene, world, time, instantiatedModel.storeResult, 
                             ModiaLang.isTerminal(instantiatedModel) && leq_mode == -1, leq_mode)
    
        # Copy the joint residuals in to the residuals vector
        if leq_mode == 0
            getJointResiduals_for_leq_mode_0!(scene, jointObjects, residuals, jointStartIndex, jointNdof, cache_h)
        elseif leq_mode > 0
            getJointResiduals_for_leq_mode_pos!(scene, jointObjects, residuals, jointStartIndex, jointNdof, cache_h)
        else
            residuals .= convert(FloatType, 0)
        end
    end
    return residuals
end



#= Efficiency improvement, by computing only terms that are needed in the respective phase
   of ModiaBase/src/EquationAndStateInfo.jl - Base.iterate(iterator::LinearEquationsIterator, ...)

 nf: Number of degrees of freedom
  q: Generalized joint coordinates of the tree joints
  u: Generalized joint forces of the tree joints
res: Residuals
e_i: i-th unit vector of length(q)

qd  = der(q)
qdd = der(qd)
res = M(q)*qdd + h(q,qd,gravity,contact forces) - u(q,qd,qdd)

multibodyResidual(..., qdd, ...) is called inside the LinearEquationsIterator to solve a linear equation system:

    res := A*x - b

that is called in the following way:

    leq_mode = 0        : Return "res := -b"        or "res :=            h(q,qd,gravity,contact forces) - u(q,qd,qdd=0)"
    leq_mode = i (1..nf): Return "res := A*e_i - b" or "res := M(q)*e_i + h(q,qd,gravity,contact forces) - u(q,qd,qdd=e_i)"
    leq_mode = -1       : Return "res := A*x - b"   or "res := 0.0"   # since residual is not used, it need not be computed.

For Modia3D:
    cache_h = h(q,qd,gravity,contact forces)

    leq_mode =  0: Compute only terms that depend on q, qd, gravity and contact forces.
                   All position and velocity terms, such as Rrel and Rabs are computed
                   and stored in Object3D.
                   res := h(q,qd,gravity) - u(q,qd,qdd)   # qdd = 0
                   cache_h := res + u
                   return res

    leq_mode =  i: res := M(q)*qdd - u(q,qd,qdd)         # qdd = e_i
                   return res := res + cache_h

    leq_mode = -1: res := M(q)*qdd - u(q,qd,qdd)         # qdd = A\b
                   return res := res + cache_h
=#

function multibodyResiduals3!(sim, scene, world, time, storeResults, isTerminal, leq_mode)
    tree            = scene.treeForComputation
    visualize       = scene.visualize   # && sim.model.visualiz
    exportAnimation = scene.exportAnimation
    
    if isTerminal  #if Modia.isTerminalOfAllSegments(sim)
        TimerOutputs.@timeit sim.timer "Modia3D_4 isTerminal" begin
            if exportAnimation
                Modia3D.exportAnimation(scene)
            end
            if visualize
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
    world.f = Modia3D.ZeroVector3D
    world.t = Modia3D.ZeroVector3D

    compute_vis = storeResults && !isTerminal && (visualize || exportAnimation)

    # Computation depending on leq_mode (the mode of the LinearEquationsIterator)
    if leq_mode == 0
        TimerOutputs.@timeit sim.timer "Modia3D_1" begin
            # Compute only terms that depend on q, qd, gravity and contact forces.
            # All position and velocity terms, such as Rrel and Rabs are computed
            # and stored in Object3D.
            #   res := h(q,qd,gravity) - u(q,qd,qdd)   # qdd = 0
            #   cache_h := res + u
            #   return res
    
            # Compute kinematics
            TimerOutputs.@timeit sim.timer "Modia3D_1 computeKinematics!" computeKinematics!(scene, tree, time)
    
            # Compute mass/inertia forces in a forward recursion and initializes forces/torques
            for obj in tree
                if compute_vis && length( obj.visualizationFrame ) == 1
                    obj.visualizationFrame[1].r_abs = obj.r_abs
                    obj.visualizationFrame[1].R_abs = obj.R_abs
                end
    
                if obj.hasMass
                    # Compute inertia forces / torques
                    w     = obj.w
                    grav  = gravityAcceleration(scene.options.gravityField, obj.r_abs)
                    obj.f = -obj.m*( obj.R_abs*(obj.a0 - grav) + cross(obj.z, obj.r_CM) + cross(w, cross(w, obj.r_CM)))
                    obj.t = -(obj.I_CM*obj.z + cross(w, obj.I_CM*w)) + cross(obj.r_CM, obj.f)
                else
                    obj.f = Modia3D.ZeroVector3D
                    obj.t = Modia3D.ZeroVector3D
                end
            end # end forward recursion
    
            # Compute contact forces/torques
            if scene.collide
                computeContactForcesAndTorques(sim, scene, world, time, nothing)
            end
    
            # Backward recursion (Compute forces/torques and residues)
            TimerOutputs.@timeit sim.timer "Modia3D_1 computeForcesAndResiduals" computeForcesTorquesAndResiduals!(scene, tree, time)
        end
        return nothing

    elseif leq_mode > 0
        TimerOutputs.@timeit sim.timer "Modia3D_2" begin    
            # Compute   res := M(q)*e_i - u(q,qd,qdd=e_i)
            #           res := res + cache_h
    
            # Compute kinematics
            TimerOutputs.@timeit sim.timer "Modia3D_2 computeKinematics!" computeKinematics_for_leq_mode_pos!(scene, tree, time)
    
            for obj in tree
                if obj.hasMass
                    # Compute inertia forces / torques
                    obj.f = -obj.m*( obj.R_abs*obj.a0 + cross(obj.z, obj.r_CM) )
                    obj.t = -obj.I_CM*obj.z + cross(obj.r_CM, obj.f)
                else
                    obj.f = Modia3D.ZeroVector3D
                    obj.t = Modia3D.ZeroVector3D
                end
            end # end forward recursion
    
            # Compute forces/torques and residues in a backward recursion
            TimerOutputs.@timeit sim.timer "Modia3D_2 computeForcesAndResiduals" computeForcesTorquesAndResiduals!(scene, tree,time)
        end
        return nothing

    elseif leq_mode == -1
        TimerOutputs.@timeit sim.timer "Modia3D_3" begin 
            if compute_vis  # Visualize at a communication point
                # Compute positions of frames that are only used for visualization
                TimerOutputs.@timeit sim.timer "Modia3D_3 visualize!" begin
                    if scene.options.useOptimizedStructure
                        for obj in scene.updateVisuElements
                            parent = obj.parent
                            obj.r_abs = obj.r_rel ≡ Modia3D.ZeroVector3D ? parent.r_abs : parent.r_abs + parent.R_abs'*obj.r_rel
                            obj.R_abs = obj.R_rel ≡ Modia3D.NullRotation ? parent.R_abs : obj.R_rel*parent.R_abs
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
                    TimerOutputs.@timeit sim.timer "Modia3D_3 exportAnimation" begin
                        objectData = []
                        for obj in scene.allVisuElements
                            pos = obj.r_abs
                            ori = Modia3D.from_R(obj.R_abs)
                            dat = animationData(pos, ori)
                            push!(objectData, dat)
                        end
                        aniStep = animationStep(time, objectData)
                        push!(scene.animation, aniStep)
                    end
                end
            end
        end

    else
        error("Bug in dynamics.jl: leq_mode = $leq_mode (must be >= -1)")
    end

    # Copy variables to residues
    #Modia3D.copy_variables_to_residue!(var, _x, _derx, _r)
    #end # from open
    return nothing
end
