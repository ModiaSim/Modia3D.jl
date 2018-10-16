# License for this file: MIT (expat)
# Copyright 2017-2018, DLR Institute of System Dynamics and Control
#
# This file is part of module
#   Modia3D.Composition (Modia3D/Composition/_module.jl)
#


function build_tree_and_velements!(scene::Scene, world::Object3D)::NOTHING
   options             = scene.options
   visualizeFrames     = options.visualizeFrames
   renderer            = Modia3D.renderer[1]
   autoCoordsys        = scene.autoCoordsys
   velements           = scene.velements
   tree                = scene.tree
   cutJoints           = scene.cutJoints
   stack               = scene.stack
   enableVisualization = scene.options.enableVisualization
   analysis            = scene.analysis
   empty!(velements)
   empty!(tree)
   empty!(stack)

   # Handle world (do not push world on stack and do not include it on scene.tree)
   #if analysis != ModiaMath.KinematicAnalysis
      world.dynamics = Object3Ddynamics()
      if visualizeFrames && isNotCoordinateSystem(world) && world.visualizeFrame != Modia3D.False
         # Visualize world frame
         world.visualizationFrame = copyObject3D(world, Graphics.CoordinateSystem(2*options.defaultFrameLength))
         push!(velements, world.visualizationFrame)
      end
   #end

   # Traverse all frames (starting from the children of world) and put frames on tree and visible elements on velements
   push!(stack, world)
   while length(stack) > 0
      frame = pop!(stack)

      # Initialize dynamic part of frame and push frame on tree
      #if analysis != ModiaMath.KinematicAnalysis
         frame.dynamics = Object3Ddynamics()
      #end
      push!(tree,frame)

      # Determine whether cut-joints are connected to the frame
      for obj in frame.twoObject3Dobject
         if typeof(obj) <: Modia3D.AbstractJoint
            if !obj.visited
               println("\n... Cut-joint ", ModiaMath.instanceName(obj), " pushed on scene.cutJoints vector")
               push!(cutJoints, obj)
               obj.visited = true
            end
         end
      end

      # If visualization desired, push frame on velements
      if enableVisualization
         if visualizeFrames && frame != world && isNotCoordinateSystem(frame) && frame.visualizeFrame != Modia3D.False
            # Visualize coordinate system of Object3D
            frame.visualizationFrame = copyObject3D(frame, autoCoordsys)
            push!(velements, frame.visualizationFrame)
         end
         if isVisible(frame.data, renderer)
            # Visualize Object3D
            push!(velements, frame)
         end
      end

      # Visit children of frame
      append!(stack, frame.children)
   end

   scene.visualize = length(scene.velements) > 0
   return nothing
end


# all visible elements are build here
function build_velements!(scene::Scene, world::Object3D)::NOTHING
   options         = scene.options
   visualizeFrames = options.visualizeFrames
   renderer        = Modia3D.renderer[1]
   autoCoordsys    = scene.autoCoordsys
   velements       = scene.velements
   stack           = scene.stack
   empty!(velements)
   empty!(stack)


   if options.visualizeWorld
      # Visualize world
      push!(velements, copyObject3D(world, scene.worldCoordinateSystem))
   end

   # Traverse all frames and put visible elements on velements
   push!(stack, world)
   while length(stack) > 0
      frame = pop!(stack)
      if visualizeFrames && isNotCoordinateSystem(frame) && frame.visualizeFrame != Modia3D.False
         # Visualize coordinate system of frame
         push!(velements, copyObject3D(frame, autoCoordsys))
      end
      if isVisible(frame.data, renderer)
         # Visualize frame (including frame.data)
         push!(velements, frame)
      end
      append!(stack, frame.children)
   end
   if length(scene.velements) > 0
      scene.visualize = true
   end
   return nothing
end


# the indices of super objects, which can't collide, are stored in a list
function fillStackOrQueue!(scene::Scene, frame::Object3D, cantCollSuperObj::Array{Int64,1})
  for child in frame.children
    if isNotWorld(child)
      if isNotFixed(child)
        push!(scene.queue, child)
        if !child.joint.canCollide    # !isFree(child) &&  !( typeof(child.joint) <: Modia3D.AbstractPrismatic )
          push!(cantCollSuperObj, length(scene.queue))
        end
      else
        push!(scene.stack, child)
      end
    end
  end
  return cantCollSuperObj
end


# creates super objects, they are rigidly attached
function checkCollision(scene::Scene, frame::Object3D, superObj::Array{Object3D,1})
  if canCollide(frame)
    push!(superObj, frame)
  end
  return superObj
end


# it builds the elements which may collide
# to reduce the amount of collision pairs, some assumptions are made:
#   elements which are rigidly attached, can't collide
#     these elements form together a super object
#   elements which are directly connected with a joint can't collide
#     these elements are excluded from the collision list
function build_celements!(scene::Scene, world::Object3D)::NOTHING
  stack = scene.stack
  queue = scene.queue
  empty!(stack)
  empty!(queue)

  push!(queue, world)
  actPos = 1
  nPos   = 1

  # println("world = ", world.children)

  while actPos <= nPos
    superObj         = Array{Object3D,1}()
    cantCollSuperObj = Array{Int64,1}()
    AABBrow          = Array{Basics.BoundingBox,1}()
    frameRoot = queue[actPos]

    if frameRoot != world
      superObj = checkCollision(scene,frameRoot,superObj)
    end
    cantCollSuperObj = fillStackOrQueue!(scene,frameRoot,cantCollSuperObj)

    while length(stack) > 0
      frameChild       = pop!(stack)
      superObj         = checkCollision(scene,frameChild,superObj)
      cantCollSuperObj = fillStackOrQueue!(scene,frameChild,cantCollSuperObj)
    end

    if length(superObj) > 0
      AABBrow = [Basics.BoundingBox() for i = 1:length(superObj)]
      push!(scene.celements, superObj)
      push!(scene.AABB, AABBrow)
      if isempty(cantCollSuperObj)
        push!(scene.cantCollide, [0])
      else
        push!(scene.cantCollide, cantCollSuperObj)
      end
    end
    nPos = length(queue)
    actPos += 1
  end


  println("scene.cantCollide ", scene.cantCollide)
  for a in scene.celements
    println("[")
    for b in a
      println(b)
    end
    println("]")
  end


  if length(scene.celements) > 1
    scene.collide = true
  else
    scene.collide = false
  end
  return nothing
end


getAssembly(assembly::Modia3D.AbstractAssembly) = assembly
getAssembly(dummy)                              = nothing

function build_SignalObject3DConnections!(assembly::Modia3D.AbstractAssembly)
  scene     = assembly._internal.scene

  uniqueSignals       = scene.uniqueSignals
  uniqueForceTorques  = scene.uniqueForceTorques
  potentialVarInput   = scene.potentialVarInput
  potentialVarOutput  = scene.potentialVarOutput
  flowVarInput        = scene.flowVarInput
  flowVarOutput       = scene.flowVarOutput

  empty!(uniqueSignals)
  empty!(uniqueForceTorques)

  empty!(potentialVarInput)
  empty!(potentialVarOutput)
  empty!(flowVarInput)
  empty!(flowVarOutput)

  assemblyStack = Modia3D.AbstractAssembly[]
  push!(assemblyStack, assembly)
  while length(assemblyStack) > 0
    assembly = pop!(assemblyStack)

    # for checking if there are any sub assemblies
    for v in fieldnames(typeof(assembly))
      subAssembly = getAssembly(getfield(assembly,v))
      if typeof(subAssembly) != NOTHING && typeof(subAssembly) != Modia3D.Composition.Part
        push!(assemblyStack, subAssembly)
      end
    end

    # for signals
    for signal in assembly._internal.uniqueSignals
      if !in(signal, uniqueSignals)
        push!(uniqueSignals, signal)
      end
    end

    # for forces and torques
    for forceTorque in assembly._internal.uniqueForceTorques
      if !in(forceTorque, uniqueForceTorques)
        push!(uniqueForceTorques, forceTorque)
      end
    end

    append!(potentialVarInput, assembly._internal.potentialVarInput)
    append!(potentialVarOutput, assembly._internal.potentialVarOutput)
    append!(flowVarInput, assembly._internal.flowVarInput)
    append!(flowVarOutput, assembly._internal.flowVarOutput)

  end
end
build_SignalObject3DConnections!(dummy) = nothing


"""
    visualizeAssembly!(assembly::Modia3D.AbstractAssembly)

Visualize the `assembly` defined with macro [`Modia3D.@assembly`](@ref)
in its initial configuration (but without simulating it).
"""
function visualizeAssembly!(assembly::Modia3D.AbstractAssembly)::NOTHING
   Modia3D.initAnalysis!(assembly)
   Modia3D.updatePosition!(assembly)
   Modia3D.visualize!(assembly,0.0)
   Modia3D.visualize!(assembly,1.0)
   Modia3D.closeAnalysis!(assembly)
   return nothing
end


function visualizeWorld!(world::Object3D; sceneOptions = SceneOptions())::NOTHING
   scene = Scene(sceneOptions)
   scene.analysis = ModiaMath.KinematicAnalysis
   initAnalysis!(world, scene)
   updatePosition!(world)
   visualize!(Modia3D.renderer[1], 0.0)
   visualize!(Modia3D.renderer[1], 1.0)
   closeAnalysis!(scene)
   return nothing
end

function initAnalysis!(world::Object3D, scene::Scene)
   # Initialize spanning tree and visualization (if visualization desired and visual elements present)
   build_tree_and_velements!(scene, world)
   if scene.visualize
      initializeVisualization(Modia3D.renderer[1], scene.velements)
   end

   # Initialize contact detection if contact detection desired and objects with contactMaterial are present
   if scene.options.enableContactDetection
      build_celements!(scene, world)
      if scene.collide
        initializeContactDetection!(world, scene) # scene.options.contactDetection, scene.celements, scene.cantCollide, scene.AABB)
      end
   end

   # Initialize connections between signals and frames, joints, ...
   # build_SignalObject3DConnections!(assembly)

   scene.initAnalysis = true
end



function initAnalysis!(assembly::Modia3D.AbstractAssembly;
                       analysis::ModiaMath.AnalysisType=ModiaMath.KinematicAnalysis)
   # Add visual elements to scene.velements
   if typeof(assembly._internal.referenceObject3D) == NOTHING
      error("\nError message from Modia3D.initAnalysis!(..):\n",
            typeof(assembly), " has no reference frame.")
   end

   # Construct Scene(..) object
   world = assembly._internal.referenceObject3D
   so    = assembly._internal.sceneOptions
   sceneOptions::SceneOptions = typeof(so) == NOTHING ? SceneOptions() : so
   scene = Scene(sceneOptions)
   scene.analysis = analysis
   assembly._internal.scene = scene

   # Initialize spanning tree and visualization (if visualization desired and visual elements present)
   build_tree_and_velements!(scene, world)
   if scene.visualize
      initializeVisualization(Modia3D.renderer[1], scene.velements)
   end

   # Initialize contact detection if contact detection desired and objects with contactMaterial are present
   if scene.options.enableContactDetection
      build_celements!(scene, world)
      if scene.collide
        initializeContactDetection!(world, scene) # scene.options.contactDetection, scene.celements, scene.cantCollide, scene.AABB)
      end
   end

   # Initialize connections between signals and frames, joints, ...
   build_SignalObject3DConnections!(assembly)

   scene.initAnalysis = true
end


#=
function initAnalysis!(assembly::Modia3D.AbstractAssembly)
   # Add visual elements to scene.velements
   if typeof(assembly._internal.referenceObject3D) == NOTHING
      error("\nError message from Modia3D.initAnalysis!(..):\n",
            typeof(assembly), " has no reference frame.")
   end

   # Construct Scene(..) object
   world = assembly._internal.referenceObject3D
   scene = typeof(world.data) == SceneOptions ? Scene(world.data) : Scene()
   assembly._internal.scene = scene

   # Initialize visualization, if visualization desired and visual elements present
   if scene.options.enableVisualization
      build_velements!(scene,world)
      if scene.visualize
        initializeVisualization(Modia3D.renderer[1], scene.velements)
      end
   end

   # Initialize contact detection if contact detection desired and objects with contactMaterial are present
   if scene.options.enableContactDetection
      build_celements!(scene, world)
      if scene.collide
        initializeContactDetection!(world, scene) # scene.options.contactDetection, scene.celements, scene.cantCollide, scene.AABB)
      end
   end

   build_revElements!(scene,world)

   scene.initAnalysis = true
end
=#





# Error Messages
function assertInitAnalysis(assembly::Modia3D.AbstractAssembly, functionName::String)
  if !assembly._internal.scene.initAnalysis
    error("\nError message from ", functionName, ":\n",
          "Call Modia3D.initAnalysis!(..) first.")
  end
end

function errorMessageCollision(functionName::String)
  error("\nError message from ", functionName, ":\n",
        "There are too less solids which can collide.")
end


function visualize!(assembly::Modia3D.AbstractAssembly, time::Float64)
  assertInitAnalysis(assembly, "Modia3D.visualize!(..)")
  scene = assembly._internal.scene
  if scene.visualize
    visualize!(Modia3D.renderer[1], time)
  end
end

function visualize!(scene::Scene, time::Float64)
  if scene.visualize
    visualize!(Modia3D.renderer[1], time)
  end
end

function getDistances!(assembly::Modia3D.AbstractAssembly)
  assertInitAnalysis(assembly, "Modia3D.getDistances!(..)")
  if assembly._internal.scene.collide
    ch = assembly._internal.scene.options.contactDetection
    getDistances!(ch)
  else
    errorMessageCollision("Modia3D.getDistances!(..)")
  end
end


function setComputationFlag(assembly::Modia3D.AbstractAssembly)
  assertInitAnalysis(assembly, "Modia3D.setComputationFlag!(..)")
  if assembly._internal.scene.collide
    ch = assembly._internal.scene.options.contactDetection
    setComputationFlag(ch)
  else
    errorMessageCollision("Modia3D.setComputationFlag!(..)")
  end
end

function selectContactPairs!(assembly::Modia3D.AbstractAssembly)
  assertInitAnalysis(assembly, "Modia3D.selectContactPairs!(..)")
  if assembly._internal.scene.collide
    ch = assembly._internal.scene.options.contactDetection
    selectContactPairs!(ch)
  else
    errorMessageCollision("Modia3D.selectContactPairs!(..)")
  end
end


function performAnalysis!(assembly::Modia3D.AbstractAssembly, time::Float64)
   assertInitAnalysis(assembly, "performAnalysis!(..)")
   scene = assembly._internal.scene

   for frame in scene.tree
      parent = frame.parent
      if !(frame.r_rel ≡ ModiaMath.ZeroVector3D)
         frame.r_abs = parent.r_abs + parent.R_abs'*frame.r_rel
      end
      if !(frame.R_rel ≡ ModiaMath.NullRotation)
         frame.R_abs = frame.R_rel*parent.R_abs
      end
   end

   if scene.visualize
      visualize!(Modia3D.renderer[1], time)
   end
end


function closeContactDetection!(assembly::Modia3D.AbstractAssembly)
  assertInitAnalysis(assembly, "Modia3D.closeContactDetection!(..)")
  if assembly._internal.scene.collide
    ch = assembly._internal.scene.options.contactDetection
    closeContactDetection!(ch)
  else
    errorMessageCollision("Modia3D.closeContactDetection!(..)")
  end
end


function closeAnalysis!(scene::Scene)
     # Close Visualisation
     closeVisualization(Modia3D.renderer[1])
     Basics.emptyArray!(scene.velements)
     scene.visualize = false

     # Close Collision detection
     if scene.collide
        closeContactDetection!(scene.options.contactDetection)
     end
     Basics.emptyArray!(scene.stack)
     Basics.emptyArray!(scene.queue)
     Basics.emptyArray!(scene.celements)
     Basics.emptyArray!(scene.cantCollide)
     scene.collide = false

     # Close signals and Forces and Torques
     Basics.emptyArray!(scene.uniqueSignals)
     Basics.emptyArray!(scene.uniqueForceTorques)
     Basics.emptyArray!(scene.potentialVarInput)
     Basics.emptyArray!(scene.potentialVarOutput)
     Basics.emptyArray!(scene.flowVarInput)
     Basics.emptyArray!(scene.flowVarOutput)

     # Close Spanning tree
     Basics.emptyArray!(scene.tree)

     # Close Analysis
     scene.initAnalysis = false
end

closeAnalysis!(assembly::Modia3D.AbstractAssembly) = typeof(assembly._internal.scene) != NOTHING ? closeAnalysis!(assembly._internal.scene) : nothing
