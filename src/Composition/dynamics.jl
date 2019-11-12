# License for this file: MIT (expat)
# Copyright 2017-2018, DLR Institute of System Dynamics and Control
#
# This file is part of module
#   Modia3D.Composition (Modia3D/Composition/_module.jl)
#



"""
    get_Object3DsWithoutParent!(assembly,frames) - Returns all frames within assembly, that do not have a parent frame

The function traverses assembly and stores all frames without a parent frame in vector frames.
"""
function get_Object3DsWithoutParent!(assembly::Modia3D.AbstractAssembly, frames::AbstractVector)::NOTHING
    assemblyType = typeof(assembly)

    for i = 1:fieldcount(assemblyType)
          field     = getfield(assembly, fieldname(assemblyType,i))
          fieldType = typeof(field)
          if fieldType == Object3D
             if hasNoParent(field)
                push!(frames,field)
             end
          elseif fieldType <: Modia3D.AbstractAssembly
             get_Object3DsWithoutParent!(field, frames)
          end
    end
    return nothing
end


"""
    world = get_WorldObject3D(assembly) - Returns the assembly world frame

The function traverses assembly and collects all frames that do not have a parent frame.
If exactly one such frame is present, it is returned.
Otherwise, an error is triggered
"""
function get_WorldObject3D(assembly::Modia3D.AbstractAssembly)::Object3D
   frames = Object3D[]
   get_Object3DsWithoutParent!(assembly, frames)
   if length(frames) == 1
      return frames[1]
   elseif length(frames) < 1
      error("\nError message from Modia3D.SimulationModel!(..):\n",
            typeof(assembly), " has no reference frame.")
   else
      frameNames = String[]
      for frame in frames
         push!(frameNames, ModiaMath.fullName(frame))
      end
      error("\nError message from Modia3D.SimulationModel!(..):\n",
            typeof(assembly), " has the following frames that do not have a parent frame:\n",
            frameNames, "\n",
            "This is not allowed (exactly one frame must not have a parent frame).")
   end
end


"""
    realScalar = getFlangeVariable(assembly::Modia3D.AbstractAssembly, name::String)

Return the reference of a Revolute or Prismatic flange variable, given the top-level `assembly`
and the full path `name` of the flange variable. If one of the following flange variable names
is provided, a reference to the respective RealScalar is returned:
```
flange_b.phi
flange_b.s
flange_b.tau
flange_b.f
der(<full-name>.flange_b.phi)
der(<full-name>.flange_b.s)
```

If one of the following flange names is provided, `nothing` is returned:
```
flange_a.phi
flange_a.s
flange_a.tau
flange_a.f
der(<full-name>.flange_a.phi)
der(<full-name>.flange_a.s)
```
"""
function getFlangeVariable(assembly::Modia3D.AbstractAssembly, name::String)::Union{ModiaMath.RealScalar,Nothing}
    # Handle der(..)
    lenName = length(name)
    if lenName >= 4 && SubString(name,1,4) == "der("
        if SubString(name,lenName,lenName) != ")" || lenName < 6
            error("Wrong name = \"", name, "\"")
        end
        name2 = SubString(name,5,lenName-1)
        der   = true
    else
        name2 = name
        der   = false
    end

    field = assembly
    while true
        #println("... before. name2 = ", name2)
        i = findnext(".", name2, 1)
        #println("... after.")
        if i==nothing || length(name2) <= i[1] || i[1] <= 1
            error("Name not found: \"", name, "\"")
        end
        fieldName = Symbol(SubString(name2,1,i[1]-1))
        name2     = SubString(name2,i[1]+1)
        if !isdefined(field, fieldName)
            error("Name not found: \"", name, "\"")
        end
        field = getfield(field, fieldName)

        # Handle the names "flange_a" and "flange_b"
        #println("... field name = ", fieldName, ", typeof(field) = ", typeof(field))
        if name2=="flange_a.phi"
            if typeof(field) == Modia3D.Composition.TreeJointRevolute
                return nothing
            else
                error("Name = \"", name, "\" is no flange variable of a Revolute joint")
            end
        elseif name2=="flange_a.tau"
            if typeof(field) == Modia3D.Composition.TreeJointRevolute
                return nothing
            else
                error("Name = \"", name, "\" is no flange variable of a Revolute joint")
            end
        elseif name2=="flange_a.s"
            if typeof(field) == Modia3D.Composition.TreeJointPrismatic
                return nothing
            else
                error("Name = \"", name, "\" is no flange variable of a Prismatic joint")
            end
        elseif name2=="flange_a.f"
            if typeof(field) == Modia3D.Composition.TreeJointPrismatic
                return nothing
            else
                error("Name = \"", name, "\" is no flange variable of a Prismatic joint")
            end
        elseif name2=="flange_b.phi"
            if typeof(field) == Modia3D.Composition.TreeJointRevolute
                return der ? getfield(field,:w) : getfield(field,:phi)
            else
                error("Name = \"", name, "\" is no flange variable of a Revolute joint")
            end
        elseif name2=="flange_b.tau"
            if typeof(field) == Modia3D.Composition.TreeJointRevolute
                if der
                    error("Name = \"", name, "\" is no flange variable of a Revolute joint")
                end
                return getfield(field,:tau)
            else
                error("Name = \"", name, "\" is no flange variable of a Revolute joint")
            end
        elseif name2=="flange_b.s"
            if typeof(field) == Modia3D.Composition.TreeJointPrismatic
                return der ? getfield(field,:v) : getfield(field,:s)
            else
                error("Name = \"", name, "\" is no flange variable of a Prismatic joint")
            end
        elseif name2=="flange_b.f"
            if typeof(field) == Modia3D.Composition.TreeJointPrismatic
                if der
                    error("Name = \"", name, "\" is no flange variable of a Prismatic joint")
                end
                return getfield(field,:f)
            else
                error("Name = \"", name, "\" is no flange variable of a Prismatic joint")
            end
        end
    end
end


"""
    model = Modia3D.Model(assembly::Modia3D.AbstractAssembly;
                          analysis::ModiaMath.AnalysisType=ModiaMath.DynamicAnalysis,
                          potentialNames::Vector{String} = fill("",0),
                          flowNames::Vector{String}      = fill("",0))

Generate a `Modia3D.Model` from an `assembly` generated with macro [`Modia3D.@assembly`](@ref)
and the type of `analysis` to be carried out on the `assembly`.
"""
struct Model
    name::String
    nz::Int   # Number of event indicators
    assembly::Modia3D.AbstractAssembly
    var::ModiaMath.ModelVariables
    analysis::ModiaMath.AnalysisType
    x_start::Vector{Float64}
    x_fixed::Vector{Bool}
    x_nominal::Vector{Float64}
    is_constraint::Vector{Bool}

    # For connection with Modia
    potentials::Vector{Union{ModiaMath.RealScalar,Nothing}}
    flows::Vector{Union{ModiaMath.RealScalar,Nothing}}

    function Model(assembly::Modia3D.AbstractAssembly;
                   analysis::ModiaMath.AnalysisType=ModiaMath.DynamicAnalysis,
                   potentialNames::Vector{String} = fill("",0),
                   flowNames::Vector{String}      = fill("",0))

        name = Modia3D.trailingPartOfName( string( typeof(assembly) ) )
        world = get_WorldObject3D(assembly)
        assembly._internal.referenceObject3D = world
        # println("\n... world reference frame: ", ModiaMath.fullName(world))

        # Determine Variables in assembly
        var = ModiaMath.ModelVariables(assembly, analysis=analysis)
        #ModiaMath.print_ModelVariables(var)

        # Set initial values for x
        x_start   = zeros(var.nx)
        x_fixed   = fill(false,var.nx)
        x_nominal = fill(1.0  ,var.nx)
        ModiaMath.copy_start_to_x!(var, x_start, x_fixed, x_nominal)
        # println("... x0 = ", x_start)

        # Last nfc equations are the constraint equations
        is_constraint = fill(false, var.nx)
        for i = (var.nx-var.nfc+1):var.nx
            is_constraint[i] = true
        end

        # Construct Scene(..) object
        so = assembly._internal.sceneOptions
        sceneOptions::SceneOptions = typeof(so) == NOTHING ? SceneOptions() : so
        scene = Scene(sceneOptions)
        scene.analysis = analysis
        assembly._internal.scene = scene
        if !scene.options.enableContactDetection
            scene.collide = false
        end

        # Build tree for optimized structure or standard structure
        # collision handling is only available for optimized structure
        nz = 0
        if scene.options.useOptimizedStructure
            build_superObjs!(scene, world)
            if !scene.options.enableContactDetection
               scene.collide = false
            end

            if scene.collide
                initializeContactDetection!(world, scene)
                if scene.collide
                    nz = scene.options.contactDetection.contactPairs.nz
                    append!(scene.allVisuElements, world.contactVisuObj1)
                    append!(scene.allVisuElements, world.contactVisuObj2)
                    append!(scene.allVisuElements, world.supportVisuObj1A)
                    append!(scene.allVisuElements, world.supportVisuObj1B)
                    append!(scene.allVisuElements, world.supportVisuObj1C)
                    append!(scene.allVisuElements, world.supportVisuObj2A)
                    append!(scene.allVisuElements, world.supportVisuObj2B)
                    append!(scene.allVisuElements, world.supportVisuObj2C)
                end
            end
            initializeMassComputation!(scene)
        else
            build_tree!(scene, world)
            if scene.options.enableContactDetection
                error("Collision handling is only possible with the optimized structure. Please set useOptimizedStructure = true in Modia3D.SceneOptions.")
            end
        end

        # Initialize connections between signals and frames, joints, ...
        build_SignalObject3DConnections!(assembly)
        scene.initAnalysis = true

        # Generate potentials and flows
        potentials = Vector{Union{ModiaMath.RealScalar,Nothing}}(nothing, length(potentialNames))
        flows      = Vector{Union{ModiaMath.RealScalar,Nothing}}(nothing, length(flowNames))
        for i = 1:length(potentialNames)
            potentials[i] = getFlangeVariable(assembly, potentialNames[i])
            #println("... potentials[", i, "] = ", potentials[i]==nothing ? "nothing" : ModiaMath.instanceName(potentials[i]))
        end
        for i = 1:length(flowNames)
            flows[i] = getFlangeVariable(assembly, flowNames[i])
            #println("... flows[", i, "] = ", flows[i]==nothing ? "nothing" : ModiaMath.instanceName(flows[i]))
        end

        # Generate Model object
        new(name, nz, assembly, var, analysis, x_start, x_fixed, x_nominal, is_constraint, potentials, flows)
   end
end



"""
    simModel = SimulationModel(assembly::Modia3D.AbstractAssembly;
                               analysis::ModiaMath.dAnalysisType=ModiaMath.DynamicAnalysis,
                               startTime = 0.0, stopTime  = 1.0, tolerance = 1e-4,
                               interval  = (stopTime-startTime)/500.0,
                               maxStepSize = NaN, maxNumberOfSteps=missing)

Generate a `simulationModel` from an `assembly` generated with macro [`Modia3D.@assembly`](@ref)
and the type of `analysis` to be carried out on the `assembly`.
Additionally, default `startTime`, `stopTime`, `tolerance`, `interval`,
`maxStepSize`, `maxNumberOfSteps`, for the simulation
engine are defined. These values should be adapted so that assembly-specific, meaningful
defaults are provided.
"""
struct SimulationModel <: ModiaMath.AbstractSimulationModel
    modelName::String
    simulationState::ModiaMath.SimulationState
    var::ModiaMath.ModelVariables
    model::Model

    function SimulationModel(assembly::Modia3D.AbstractAssembly;
                             analysis::ModiaMath.AnalysisType=ModiaMath.DynamicAnalysis,
                             startTime = 0.0,
                             stopTime  = 1.0,
                             tolerance = 1e-4,
                             interval  = (stopTime-startTime)/500.0,
                             maxStepSize=NaN,
                             maxNumberOfSteps=missing,
                             hev = 1e-8,
                             scaleConstraintsAtEvents::Bool = true)
        model           = Model(assembly; analysis=analysis)
        simulationState = ModiaMath.SimulationState(model.name, getModelResidues!, model.x_start, ModiaMath.Variables.getVariableName;
                                x_fixed          = model.x_fixed,
                                x_nominal        = model.x_nominal,
                                is_constraint    = model.is_constraint,
                                nz               = model.nz,
                                defaultStartTime = startTime,
                                defaultStopTime  = stopTime,
                                defaultTolerance = tolerance,
                                defaultInterval  = interval,
                                defaultMaxStepSize = maxStepSize,
                                defaultMaxNumberOfSteps = maxNumberOfSteps,
                                getResultNames   = Composition.getResultNames,
                                getResult        = ModiaMath.Variables.getResult,
                                storeResult!     = ModiaMath.Variables.storeVariables!,
                                hev = hev,
                                scaleConstraintsAtEvents = scaleConstraintsAtEvents)

#=
      scene = assembly._internal.scene
      if useOptimizedStructure
          println("\n... SimulationModel: treeAccVelo begin")
          tree = scene.treeAccVelo
      else
          println("\n... SimulationModel: tree begin")
          tree = scene.tree
      end

      for obj in tree
         if hasParent(obj)
            println(ModiaMath.instanceName(obj), " (parent = ", ModiaMath.instanceName(obj.parent), ")")
            println("    r_rel = ", obj.r_rel, ", R_rel = ", obj.R_rel)
         else
            println(ModiaMath.instanceName(obj), " (no parent)")
         end
         if hasJoint(obj)
            println("    joint type = ", typeof(obj.joint))
         end
         if objectHasMass(obj)
            println("    m = ", obj.massProperties.m, " rCM = ", obj.massProperties.rCM, " I = ", obj.massProperties.I)
         end
      end
      println("...  end\n\n")

      println("... allVisuElements:")
      for obj in scene.allVisuElements
          println(ModiaMath.instanceName(obj))
      end
      println("... end allVisuElements")
=#
       new(model.name, simulationState, model.var, model)
   end
end

ModiaMath.print_ModelVariables(model::Model) = ModiaMath.print_ModelVariables(model.var)
print_ModelVariables(          model::Model) = ModiaMath.print_ModelVariables(model.var)

ModiaMath.print_ModelVariables(simModel::SimulationModel) = ModiaMath.print_ModelVariables(simModel.model.var)
print_ModelVariables(          simModel::SimulationModel) = ModiaMath.print_ModelVariables(simModel.model.var)

getResultNames(simModel::SimulationModel) = simModel.model.var.result_names


function initializeFlowVariables(scene::Scene)
  flowVarInput   = scene.flowVarInput
  flowVarOutput  = scene.flowVarOutput
  @assert(length(flowVarInput) == length(flowVarOutput))
  for i=eachindex(flowVarInput)
    flowVarInput[i].value  = 0.0
    flowVarOutput[i].value = 0.0
  end
end


function setFlowVariables(scene::Scene)
  flowVarInput   = scene.flowVarInput
  flowVarOutput  = scene.flowVarOutput
  @assert(length(flowVarInput) == length(flowVarOutput))
  for i=eachindex(flowVarInput)
    flowVarInput[i].value  += flowVarOutput[i].value
  end
end

function setPotentialVariables(scene::Scene)
  potentialVarInput   = scene.potentialVarInput
  potentialVarOutput  = scene.potentialVarOutput
  @assert(length(potentialVarInput) == length(potentialVarOutput))
  for i=eachindex(potentialVarInput)
    potentialVarInput[i].value = potentialVarOutput[i].value
  end
end

function computationForcesAndTorques(scene::Scene, sim::ModiaMath.SimulationState)
    for forceTorque in scene.uniqueForceTorques
        computeTorque(forceTorque, sim)
    end
end

function computationSignals(scene::Scene, sim::ModiaMath.SimulationState)
    for signal in scene.uniqueSignals
        computeSignal(signal, sim)
    end
end

# Return a table of actual variable and residue values from nonlinear solver in case of error
function ModiaMath.getVariableAndResidueValues(simModel::SimulationModel)
   var = simModel.model.var
   v_table = ModiaMath.get_variableValueTable(var)
   r_table = ModiaMath.get_residueValueTable(var, simModel.simulationState.derxev)
   return (v_table, r_table)
end




# Only temporarily here. Shall be moved to ModiaMath
"""
    copy_x_to_variables!(x::Vector{Float64}, vars::ModelVariables)

Copy `x` of the integrator interface to the model variables `vars`.
"""
function copy_x_to_variables!(x::Vector{Float64}, m::ModiaMath.ModelVariables)::Nothing
    @assert(length(x) == m.nx)

    for v in m.x_var
        #println("... typeof(", fullName(v), ") = ", typeof(v), ", isimmutable(v) = ", isimmutable(v))
        if ModiaMath.isScalar(v)
            v.value = x[ v.ivar ]
        elseif isimmutable(v.value)
            # v is an immutable array (e.g. SVector{3,Float64})
            v.value = x[ v.ivar:v.ivar + length(v.value) - 1 ]
        else
            vv = v.value
            for j in 1:length(vv)
                vv[j] = x[ v.ivar + j - 1 ]
            end
        end
    end
    return nothing
end


"""
    copy_derx_to_variables!(time::Float64, derx::Vector{Float64}, vars::ModelVariables)

Copy `time` and `derx` of the integrator interface to the model variables `vars`.
"""
function copy_derx_to_variables!(time::Float64, derx::Vector{Float64}, m::ModiaMath.ModelVariables)::Nothing
    @assert(length(derx) == m.nx)

    m.var[1].value = time
    for v in m.derx_var
        if ModiaMath.isScalar(v)
            v.value = derx[ v.ivar ]
        elseif isimmutable(v.value)
            # v is an immutable array (e.g. SVector{3,Float64})
            v.value = derx[ v.ivar:v.ivar + length(v.value) - 1 ]
        else
            vv = v.value
            for j in 1:length(vv)
                vv[j] = derx[ v.ivar + j - 1 ]
            end
        end
    end

    return nothing
end



"""
    modelf1!(model, x, potentials)

Copy the potential variables defined for `model::Modia3D.Model` from `x` to vector `potentials`,
"""
function model_f1!(model::Model, x::Vector{Float64}, potentials::Vector{Float64})::Nothing
    @assert( length(model.potentials) == length(potentials) )

    # Copy (time,x) to model variables
    copy_x_to_variables!(x, model.var)

    # Copy selected potential variables from model to output argument potentials
    for i = 1:length(potentials)
        potentials[i] = model.potentials[i] == nothing ? 0.0 : model.potentials[i].value
    end
    return nothing
end


const str_DUMMY = "dummyDistance(nothing,nothing)"
const zeroVector = Vector{Float64}()

getModelResidues!(simModel::SimulationModel, time::Float64, _x::Vector{Float64}, _derx::Vector{Float64}, _r::Vector{Float64}, _w::Vector{Float64}) =
    model_f2!(simModel.model, simModel.simulationState, time, _x, _derx, zeroVector, _r, _w)

function model_f2!(m::Model, sim::ModiaMath.SimulationState,
                   time::Float64, _x::Vector{Float64}, _derx::Vector{Float64}, _flows::Vector{Float64},
                   _r::Vector{Float64}, _w::Vector{Float64})
   # println("... time = ", time, ", x = ", _x, ", derx = ", _derx)
   world = m.assembly._internal.referenceObject3D
   scene = m.assembly._internal.scene
   if scene.options.useOptimizedStructure
      tree          = scene.treeAccVelo
   else
      tree          = scene.tree
   end
   cutJoints        = scene.cutJoints
   var              = m.var
   analysis         = m.analysis

   # Handle initialization and termination of model
   if ModiaMath.isInitial(sim)
      # println("... isInitial = true")
      if scene.collide
         initializeContactDetection!(world, scene)
         append!(scene.allVisuElements, world.contactVisuObj1)
         append!(scene.allVisuElements, world.contactVisuObj2)

         append!(scene.allVisuElements, world.supportVisuObj1A)
         append!(scene.allVisuElements, world.supportVisuObj1B)
         append!(scene.allVisuElements, world.supportVisuObj1C)
         append!(scene.allVisuElements, world.supportVisuObj2A)
         append!(scene.allVisuElements, world.supportVisuObj2B)
         append!(scene.allVisuElements, world.supportVisuObj2C)
      end
      if scene.visualize
         initializeVisualization(Modia3D.renderer[1], scene.allVisuElements)
      end
   end

   if ModiaMath.isTerminal(sim)
      # println("... isTerminal = true")
      if scene.visualize
         # println("... visualization closing")
         closeVisualization(Modia3D.renderer[1])
      end
      if scene.collide
         closeContactDetection!(m.assembly)
      end
      return
   end

   # Check input arguments
   @assert(length(_x)    == var.nx)
   @assert(length(_derx) == var.nx)
   @assert(length(_r)    == var.nx)
   @assert(length(_w)    == 0)
   @assert(length(_flows) == length(m.flows))

   storeResults = ModiaMath.isStoreResult(sim) && (scene.visualize || var.nwr > 0 || var.nwc > 0 )

   # Initialize residues _r with zero
   for i in eachindex(_r)
      _r[i] = 0.0
   end

   # Copy x and derx to variables
   if length(m.potentials) == 0
       ModiaMath.copy_x_and_derx_to_variables!(time, _x, _derx, var)
   else
       copy_derx_to_variables!(time, _derx, var)
   end

   # Compute signals
   initializeFlowVariables(scene)
   computationSignals(scene, sim)
   setPotentialVariables(scene)
   setFlowVariables(scene)

   # Copy flows to model variables
   for i = 1:length(_flows)
       if m.flows[i] != nothing
          m.flows[i].value = _flows[i]
       end
   end

   # Initialize force/torque of world-frame
   world.dynamics.f = ModiaMath.ZeroVector3D
   world.dynamics.t = ModiaMath.ZeroVector3D

   # Compute positions, velocities, accelerations in a forward recursion

   open("log.txt", "a") do file

   #println("\n... Forward recursion")
   for obj in tree
      # Compute kinematics
      computeKinematics!(obj.joint, obj, analysis, time)

      if typeof(obj.visualizationFrame) != NOTHING
         obj.visualizationFrame.r_abs = obj.r_abs
         obj.visualizationFrame.R_abs = obj.R_abs
      end


      # Compute forces and torques
      initializeFlowVariables(scene)
      setPotentialVariables(scene)
      computationForcesAndTorques(scene,sim)
      setFlowVariables(scene)

      # Initialize forces and torques
      dynamics::Object3Ddynamics = obj.dynamics

      if analysis != ModiaMath.KinematicAnalysis
         hasMassProperties = false
         if scene.options.useOptimizedStructure
            if objectHasMass(obj)
               massProperties    = obj.massProperties
               hasMassProperties = true
            end
         else
            if dataHasMass(obj)
               massProperties    = obj.data.massProperties
               hasMassProperties = true
            end
         end

         if hasMassProperties
            # Compute inertia forces / torques
            mass = massProperties.m
            rCM  = massProperties.rCM
            I    = massProperties.I
            w    = dynamics.w
            #println("w = ",w)
            grav = gravityAcceleration(scene.options.gravityField, obj.r_abs)
            # println("grav = ", grav)
            if rCM === ModiaMath.ZeroVector3D
               dynamics.f = -mass*( obj.R_abs*(dynamics.a0  - grav) )
               dynamics.t = -(I*dynamics.z + cross(w, I*w))
               # println("(dynamics.a0  - grav) = ", (dynamics.a0  - grav) )
               #println("dynamics.t = ", dynamics.t)
               #println(" ")
            else
               dynamics.f = -mass*( obj.R_abs*(dynamics.a0 - grav) +
                                  cross(dynamics.z, rCM) + cross(w, cross(w, rCM)))
               dynamics.t = -(I*dynamics.z + cross(w, I*w)) + cross(rCM, dynamics.f)
            end

         else
            dynamics.f = ModiaMath.ZeroVector3D
            dynamics.t = ModiaMath.ZeroVector3D
         end
      end
      #println("    ", ModiaMath.instanceName(obj), ": f = ", dynamics.f, ", t = ", dynamics.t)

      #if ModiaMath.isInitial(sim)
      #   println(ModiaMath.instanceName(obj), ": f = ", dynamics.f, ", t = ", dynamics.t)
      #end
   end # for


   # Compute contact forces/torques
   if scene.collide
      ch = scene.options.contactDetection
      chpairs = ch.contactPairs

      # Compute signed distances of all contact shapes during zero-crossing computation
      setComputationFlag(ch)
      if ModiaMath.isEvent(sim)    # with Event
         selectContactPairsWithEvent!(sim, ch, world) #sim#
      elseif ModiaMath.isZeroCrossing(sim) # no Event
         selectContactPairsNoEvent!(sim, ch, world) #sim
      else
         getDistances!(ch, world)
      end

      # Handle zero crossing event
      simh = sim.eventHandler
      for (pairID, pair) in ch.contactDict
        obj1 = pair.obj1
        obj2 = pair.obj2
        rContact      = (pair.contactPoint1 + pair.contactPoint2)/2.0
        contactNormal = pair.contactNormal

        #simh = sim.eventHandler
        #println( "time = ", sim.time, ": ", ModiaMath.instanceName(obj1), " ", ModiaMath.instanceName(obj2),
        #         " i = ", i, ", initial = ", simh.initial, ", event = ", simh.event, " change = ", chpairs.changeDirection[i] )
        if ModiaMath.isEvent(sim)
            # Include contact pair material into collision pair
            if haskey(ch.lastContactDict, pairID)
                # use material (reference) from previous event
                pair.contactPairMaterial = ch.lastContactDict[pairID].contactPairMaterial    # improve later (should avoid to inquire pairID twice)
            else
                # determine contact pair material
                pair.contactPairMaterial = contactStart(obj1, obj2, rContact, contactNormal,
                                                         scene.options.elasticContactReductionFactor)
                simh.restart = max(simh.restart, ModiaMath.Restart)
                simh.newEventIteration = false
                if ModiaMath.isLogEvents(simh.logger)
                    name1 = ModiaMath.instanceName(obj1)
                    name2 = ModiaMath.instanceName(obj2)
                    n     = contactNormal
                    println("        distance(", name1, ",", name2, ") = ", pair.distanceWithHysteresis, " became < 0")
                    Printf.@printf("            contact normal = [%.3g,%.3g,%.3g], contact position = [%.3g,%.3g,%.3g], c_res=%.3g, d_res=%.3g\n",
                                   n[1],n[2],n[3],rContact[1],rContact[2],rContact[3], pair.contactPairMaterial.c_res, pair.contactPairMaterial.d_res)
                end
            end
        end

        # println("length(ch.dictCommunicate) ", length(ch.dictCommunicate) )
        (f1,f2,t1,t2) = responseCalculation(pair.contactPairMaterial, obj1, obj2, rContact, contactNormal,
                                             pair.distanceWithHysteresis, time, file)

        # Transform forces/torques in local part frames
        obj1.dynamics.f += obj1.R_abs*f1
        obj1.dynamics.t += obj1.R_abs*t1
        obj2.dynamics.f += obj2.R_abs*f2
        obj2.dynamics.t += obj2.R_abs*t2
#=
               if time > 0.785 && time < 0.787  && String(ModiaMath.instanceName(obj1)) != "table.box1"
                  println("obj1= \"", ModiaMath.instanceName(obj1), "\" obj2 = ", ModiaMath.instanceName(obj2), " f = ", obj1.dynamics.f, " t = ", obj1.dynamics.t, " rContact = ", rContact, " ctNormal[i] ", ModiaMath.Vector3D(chpairs.contactNormal[i]) ," time = ", time)
               end
=#
      end

      if ModiaMath.isEvent(sim)
            # Should be possible to make this more efficient
            for (pairID, pair) in ch.lastContactDict
                if !haskey(ch.contactDict, pairID)
                    contactEnd(pair.contactPairMaterial, pair.obj1, pair.obj2)
                    simh.restart = max(simh.restart, ModiaMath.Restart)
                    simh.newEventIteration = false
                    if !ModiaMath.isLogEvents(simh.logger)
                        break
                    end
                    name1 = ModiaMath.instanceName(pair.obj1)
                    name2 = ModiaMath.instanceName(pair.obj2)
                    println("        distance(", name1, ",", name2, ")  became > 0")
                end
            end

            # Save contactDict in lastContactDict
            empty!(ch.lastContactDict)
            for (pairID, pair) in ch.contactDict
                push!(ch.lastContactDict, pairID => pair)
            end
      end
   end


   # For all cut-joints, set forces/torques at the frames and compute cut-joint residues
   for joint in cutJoints
      computePositionResidues!(joint, time)
      if analysis == ModiaMath.DynamicAnalysis
         computeVelocityResidues!(joint, time)
      end
      if analysis != ModiaMath.KinematicAnalysis
         computeCutForcesAndToques!(joint, time)
      end
   end

   #println("\n... Backward recursion")

   # Compute forces/torques and residues in a backward recursion
   if analysis != ModiaMath.KinematicAnalysis
      for i = length(tree):-1:1
         obj = tree[i]
         computeForceTorqueAndResidue!(obj.joint, obj, analysis, time)
      end

      #if ModiaMath.isInitial(sim)
      #   for i = length(tree):-1:1
      #      obj = tree[i]
      #      println(ModiaMath.instanceName(obj), ": f = ", obj.dynamics.f, ", t = ", obj.dynamics.t)
      #   end
      #end
   end

   # Visualize at a communication point

   if scene.visualize && storeResults
      # Compute positions of frames that are only used for visualization
      if scene.options.useOptimizedStructure
          for obj in scene.treeVisu
              parent = obj.parent
              obj.r_abs = obj.r_rel ≡ ModiaMath.ZeroVector3D ? parent.r_abs : parent.r_abs + parent.R_abs'*obj.r_rel
              obj.R_abs = obj.R_rel ≡ ModiaMath.NullRotation ? parent.R_abs : obj.R_rel*parent.R_abs
              if typeof(obj.visualizationFrame) != NOTHING
                 obj.visualizationFrame.r_abs = obj.r_abs
                 obj.visualizationFrame.R_abs = obj.R_abs
              end
          end
      end
      visualize!(Modia3D.renderer[1], time)
   end

   # Copy variables to residues
   ModiaMath.copy_variables_to_residue!(var, _x, _derx, _r)

   #println("... x = ", _x, ", derx = ", _derx, ", residue = ", _r)

end   # from open
   return nothing
end
