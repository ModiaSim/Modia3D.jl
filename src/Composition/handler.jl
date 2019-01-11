# License for this file: MIT (expat)
# Copyright 2017-2018, DLR Institute of System Dynamics and Control
#
# This file is part of module
#   Modia3D.Composition (Modia3D/Composition/_module.jl)
#


function build_tree_and_allVisuElements!(scene::Scene, world::Object3D)::NOTHING
   options             = scene.options
   visualizeFrames     = options.visualizeFrames
   renderer            = Modia3D.renderer[1]
   autoCoordsys        = scene.autoCoordsys
   allVisuElements     = scene.allVisuElements
   tree                = scene.tree
   cutJoints           = scene.cutJoints
   stack               = scene.stack
   enableVisualization = scene.options.enableVisualization
   analysis            = scene.analysis
#   empty!(allVisuElements)
   empty!(tree)
   empty!(stack)
#   println("build_tree_and_allVisuElements")

   # Handle world (do not push world on stack and do not include it on scene.tree)
   #if analysis != ModiaMath.KinematicAnalysis
      world.dynamics = Object3Ddynamics()

#=
      if visualizeFrames && isNotCoordinateSystem(world) && world.visualizeFrame != Modia3D.False
         # Visualize world frame
         world.visualizationFrame = copyObject3D(world, Graphics.CoordinateSystem(2*options.defaultFrameLength))
         push!(allVisuElements, world.visualizationFrame)
      end
=#

   #end

   # Traverse all frames (starting from the children of world) and put frames on tree and visible elements on allVisuElements
   push!(stack, world)
   while length(stack) > 0
      frame = pop!(stack)

      # Initialize dynamic part of frame and push frame on tree
      #if analysis != ModiaMath.KinematicAnalysis
         frame.dynamics = Object3Ddynamics()
      #end
      push!(tree,frame)

#=
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
=#


#=
      # If visualization desired, push frame on allVisuElements
      if enableVisualization
         if visualizeFrames && frame != world && isNotCoordinateSystem(frame) && frame.visualizeFrame != Modia3D.False
            # Visualize coordinate system of Object3D
            frame.visualizationFrame = copyObject3D(frame, autoCoordsys)
            push!(allVisuElements, frame.visualizationFrame)
         end
         if isVisible(frame.data, renderer)
            # Visualize Object3D
            push!(allVisuElements, frame)
         end
      end
=#
      # Visit children of frame
      append!(stack, frame.children)
   end

#   scene.visualize = length(scene.allVisuElements) > 0
#   println("length(scene.allVisuElements) = ", length(scene.allVisuElements))
   return nothing
end

# the indices of super objects, which can't collide, are stored in a list
function fillStackOrBuffer!(scene::Scene, superObj::SuperObjsRow, obj::Object3D)
  #println("begin fillStackOrBuffer")
  for child in obj.children
    #println("child = ", child)
    if isNotWorld(child)
      if isNotFixed(child)
        push!(scene.buffer, child)
        println("in scene.buffer: child = $child")
        if !child.joint.canCollide    # !isFree(child) &&  !( typeof(child.joint) <: Modia3D.AbstractPrismatic )
          push!(superObj.noCPair, length(scene.buffer))
          println("in scene.buffer: length(scene.buffer) = ", length(scene.buffer))
        #  println("suberObj = ", superObj)
          #println("length(scene.buffer) = ", length(scene.buffer))
        end
      else
        push!(scene.stack, child)
      end
    end
  end
  #println("end fillStackOrBuffer")
#  println(" ")
  return nothing
end


insert_and_dedup!(v::Vector, x) = (splice!(v, searchsorted(v,x), x); v)

function addIndicesOfCutJointsToSuperObj(scene::Scene)
  tmp = collect(values(scene.noCPairsHelp))
  for i=1:length(tmp)
    if length(tmp[i]) == 2
      # println("tmp[$i] = ", tmp[i])
      insert_and_dedup!(scene.noCPairs[minimum(tmp[i])], maximum(tmp[i]))
    else
      error("...from addIndicesOfCutJointsToSuperObj: problems with amount of cut joints")
    end
  end
end


# it builds the elements which may collide
# to reduce the amount of collision pairs, some assumptions are made:
#   elements which are rigidly attached, can't collide
#     these elements form together a super object
#   elements which are directly connected with a joint can't collide
#     these elements are excluded from the collision list
function build_celements!(scene::Scene, world::Object3D)::NOTHING
  stack = scene.stack
  buffer = scene.buffer
  empty!(stack)
  empty!(buffer)
  empty!(scene.allVisuElements)

  # push!(buffer, world)
  actPos = 0
  nPos   = 1

  while actPos <= nPos
    superObjsRow = SuperObjsRow()
    AABBrow      = Array{Basics.BoundingBox,1}()
    if actPos == 0
      frameRoot = world
    else
      frameRoot = buffer[actPos]
    end


    fillVisuElements(scene, frameRoot, world)
    createCutJoints(scene, frameRoot)
    if frameRoot != world
      assignAll(scene, superObjsRow, frameRoot, world, actPos)
    elseif frameRoot == world
      # assignAll(scene, superObjsRow, frameRoot, world, actPos)
      for child in frameRoot.children
        println("child von world hat einen Joint = ", hasJoint(child))
      end
    end
    fillStackOrBuffer!(scene, superObjsRow, frameRoot)

    while length(stack) > 0
      frameChild = pop!(stack)

      fillVisuElements(scene, frameChild, world)
      createCutJoints(scene, frameChild)
      assignAll(scene, superObjsRow, frameChild, world, actPos)
      fillStackOrBuffer!(scene, superObjsRow, frameChild)
    end

    if length(superObjsRow.superObjCollision.superObj) > 0
      AABBrow = [Basics.BoundingBox() for i = 1:length(superObjsRow.superObjCollision.superObj)]
      push!(scene.celements, superObjsRow.superObjCollision.superObj)
      push!(scene.AABB, AABBrow)
      if isempty(superObjsRow.noCPair)
        push!(scene.noCPairs, [0])
      else
        push!(scene.noCPairs, superObjsRow.noCPair)
      end
    end
    nPos = length(buffer)
    actPos += 1
  end
  addIndicesOfCutJointsToSuperObj(scene)

  println("scene.noCPairs ", scene.noCPairs)

  for a in scene.celements
    println("[")
    for b in a
      println(b)
    end
    println("]")
    println(" ")
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
   println("initAnalysis!(world::Object3D, scene::Scene)")
  build_tree_and_allVisuElements!(scene, world)
  build_celements!(scene, world)

   # Initialize contact detection if contact detection desired and objects with contactMaterial are present
   #if scene.options.enableContactDetection
   if scene.visualize
      initializeVisualization(Modia3D.renderer[1], scene.allVisuElements)
   end
   if scene.collide
        initializeContactDetection!(world, scene) # scene.options.contactDetection, scene.celements, scene.noCPairs, scene.AABB)
    end
   #end

   # Initialize connections between signals and frames, joints, ...
   # build_SignalObject3DConnections!(assembly)

   scene.initAnalysis = true
end



function initAnalysis!(assembly::Modia3D.AbstractAssembly;
                       analysis::ModiaMath.AnalysisType=ModiaMath.KinematicAnalysis)
   # Add visual elements to scene.allVisuElements
   if typeof(assembly._internal.referenceObject3D) == NOTHING
      error("\nError message from Modia3D.initAnalysis!(..):\n",
            typeof(assembly), " has no reference frame.")
   end
   println("initAnalysis!(assembly::Modia3D.AbstractAssembly;                        analysis::ModiaMath.AnalysisType=ModiaMath.KinematicAnalysis)")
   # Construct Scene(..) object
   world = assembly._internal.referenceObject3D
   so    = assembly._internal.sceneOptions
   sceneOptions::SceneOptions = typeof(so) == NOTHING ? SceneOptions() : so
   scene = Scene(sceneOptions)
   scene.analysis = analysis
   assembly._internal.scene = scene

   # Initialize spanning tree and visualization (if visualization desired and visual elements present)
   build_tree_and_allVisuElements!(scene, world)
   build_celements!(scene, world)
   if scene.visualize
      initializeVisualization(Modia3D.renderer[1], scene.allVisuElements)
   end

   # Initialize contact detection if contact detection desired and objects with contactMaterial are present
   if scene.options.enableContactDetection

      if scene.collide
        initializeContactDetection!(world, scene) # scene.options.contactDetection, scene.celements, scene.noCPairs, scene.AABB)
      end
   end

   # Initialize connections between signals and frames, joints, ...
   build_SignalObject3DConnections!(assembly)

   scene.initAnalysis = true
end


#=
function initAnalysis!(assembly::Modia3D.AbstractAssembly)
   # Add visual elements to scene.allVisuElements
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
      build_allVisuElements!(scene,world)
      if scene.visualize
        initializeVisualization(Modia3D.renderer[1], scene.allVisuElements)
      end
   end

   # Initialize contact detection if contact detection desired and objects with contactMaterial are present
   if scene.options.enableContactDetection
      build_celements!(scene, world)
      if scene.collide
        initializeContactDetection!(world, scene) # scene.options.contactDetection, scene.celements, scene.noCPairs, scene.AABB)
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
     Basics.emptyArray!(scene.allVisuElements)
     scene.visualize = false

     # Close Collision detection
     if scene.collide
        closeContactDetection!(scene.options.contactDetection)
     end
     Basics.emptyArray!(scene.stack)
     Basics.emptyArray!(scene.buffer)
     Basics.emptyArray!(scene.celements)
     Basics.emptyArray!(scene.noCPairs)
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
