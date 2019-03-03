# License for this file: MIT (expat)
# Copyright 2017-2018, DLR Institute of System Dynamics and Control
#
# This file is part of module
#   Modia3D.Composition (Modia3D/Composition/_module.jl)
#


function build_tree!(scene::Scene, world::Object3D)::NOTHING
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
   empty!(tree)
   empty!(stack)

   world.dynamics = Object3Ddynamics()
   push!(stack, world)
   while length(stack) > 0
      frame = pop!(stack)
      frame.dynamics = Object3Ddynamics()
      push!(tree,frame)
      # Visit children of frame
      append!(stack, frame.children)
   end
   return nothing
end

insert_and_dedup!(v::Vector, x) = (splice!(v, searchsorted(v,x), x); v)

function addIndicesOfCutJointsToSuperObj(scene::Scene)
  tmp = collect(values(scene.noCPairsHelp))
  for i=1:length(tmp)
    if length(tmp[i]) == 2
      insert_and_dedup!(scene.noCPairs[minimum(tmp[i])], maximum(tmp[i]))
    else
      error("...from addIndicesOfCutJointsToSuperObj: problems with amount of cut joints")
    end
  end
end


function createAABB_noCPairs(scene::Scene, superObjsRow::SuperObjsRow)
  if length(superObjsRow.superObjCollision.superObj) > 0 && !isempty(superObjsRow.noCPair)
      push!(scene.noCPairs, superObjsRow.noCPair)
  else
    push!(scene.noCPairs, [0])
  end

  if length(superObjsRow.superObjCollision.superObj) > 0
    AABBrow = [Basics.BoundingBox() for i = 1:length(superObjsRow.superObjCollision.superObj)]
    push!(scene.AABB, AABBrow)
  else
    push!(scene.AABB, [])
  end
end


function changeParentToRootObj(newParent::Object3D, obj::Object3D)
  # Save elements of obj
  child_r_rel  = obj.r_rel
  child_R_rel  = obj.R_rel
  parent_r_rel = obj.parent.r_rel
  parent_R_rel = obj.parent.R_rel
  parent = obj.parent

  child_r_abs  = obj.r_abs
  child_R_abs  = obj.R_abs
  parent_r_abs = obj.parent.r_abs
  parent_R_abs = obj.parent.R_abs

  r_new = parent_r_rel + parent_R_rel' * child_r_rel
  R_new = child_R_rel * parent_R_rel
  obj.r_rel = r_new
  obj.R_rel = R_new


  # Reverse obj, so that newParent is the new parent
  obj.parent = newParent
  push!(newParent.children, obj)

  #obj.r_abs = parent_r_abs + parent_R_abs'*r_new
  #obj.R_abs = R_new*parent_R_abs

  return nothing
end


# the indices of super objects, which can't collide, are stored in a list
function fillStackOrBuffer!(scene::Scene, superObj::SuperObjsRow, obj::Object3D, rootSuperObj::Object3D)::NOTHING
  n_children =  length(obj.children)
  help = fill(false, n_children)

  for i = 1:n_children
    child = obj.children[i]
    if isNotWorld(child)
      if isNotFixed(child)
        push!(scene.buffer, child)
        child.computeAcceleration = true
        if !child.joint.canCollide    # !isFree(child) &&  !( typeof(child.joint) <: Modia3D.AbstractPrismatic )
          push!(superObj.noCPair, length(scene.buffer))
        end
      else
        push!(scene.stack, child)
        assignAccVelo(scene.treeAccVelo, child)
        if !(obj == rootSuperObj)
          changeParentToRootObj(rootSuperObj, child)
          help[i] = true
  end; end; end; end

  if !isempty(obj.children)
    deleteat!(obj.children,help)
  end

  return nothing
end


# it builds the elements which may collide
# to reduce the amount of collision pairs, some assumptions are made:
#   elements which are rigidly attached, can't collide
#     these elements form together a super object
#   elements which are directly connected with a joint can't collide
#     these elements are excluded from the collision list
function build_superObjs!(scene::Scene, world::Object3D)::NOTHING
  if !scene.initSuperObj
  stack = scene.stack
  buffer = scene.buffer
  treeAccVelo  = scene.treeAccVelo
  empty!(stack)
  empty!(buffer)
  empty!(scene.allVisuElements)
  empty!(treeAccVelo)

  world.computeAcceleration = true
  push!(buffer, world)
  actPos = 1
  nPos   = 1

  hasOneCollisionSuperObj  = false
  hasMoreCollisionSuperObj = false

  while actPos <= nPos
    superObjsRow = SuperObjsRow()
    AABBrow      = Array{Basics.BoundingBox,1}()
    rootSuperObj = buffer[actPos]
    assign_Visu_CutJoint_Dynamics!(scene, rootSuperObj, world)
    if rootSuperObj != world
      assignAll(scene, superObjsRow, rootSuperObj, world, actPos)
      push!(treeAccVelo, rootSuperObj)
    end
    fillStackOrBuffer!(scene, superObjsRow, rootSuperObj, rootSuperObj)

    while length(stack) > 0
      frameChild = pop!(stack)
      assign_Visu_CutJoint_Dynamics!(scene, frameChild, world)
      assignAll(scene, superObjsRow, frameChild, world, actPos)
      fillStackOrBuffer!(scene, superObjsRow, frameChild, rootSuperObj)
    end

    if length(superObjsRow.superObjCollision.superObj) > 0 && hasOneCollisionSuperObj == true
      hasMoreCollisionSuperObj = true
    elseif length(superObjsRow.superObjCollision.superObj) > 0
      hasOneCollisionSuperObj = true
    end

    createAABB_noCPairs(scene, superObjsRow)
    push!(scene.superObjs, superObjsRow)
    nPos = length(buffer)
    actPos += 1
    #println(" ")
  end
  addIndicesOfCutJointsToSuperObj(scene)
#=
 println(" ")
 println("treeAccVelo Reihenfolge")
 for obj in treeAccVelo
  println(ModiaMath.fullName(obj), " obj.computeAcceleration = $(obj.computeAcceleration)")
  #println("obj.computeAcceleration = $(obj.computeAcceleration)")
 end

 println(" ")
 println("treeVisu")
 for obj in scene.treeVisu
   println(ModiaMath.fullName(obj))
 end

 println(" ")
 println("allVisuElements")
 for obj in scene.allVisuElements
   println(ModiaMath.fullName(obj))
 end
=#

#=
println(" ")
println("buffer Reihenfolge")
for obj in buffer
  println(ModiaMath.fullName(obj) , " obj.computeAcceleration = $(obj.computeAcceleration)")
end


#  println("scene.noCPairs ", scene.noCPairs)
  println("superObjRow.superObjCollision.superObj")
  for superObjRow in scene.superObjs
    println("[")
    for a in superObjRow.superObjCollision.superObj
      println(ModiaMath.fullName(a))
    end
    println("]")
    println(" ")
  end
=#

#=

println("treeVisu")
for obj in scene.treeVisu
  println(ModiaMath.fullName(obj))
end

  println("geht mit AABB weiter ")
  for a in scene.AABB
    println("[")
    for b in a
      println(b)
    end
    println("]")
    println(" ")
  end
=#

  hasMoreCollisionSuperObj ? (scene.collide = true) : (scene.collide = false)
  scene.initSuperObj = true
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
  build_tree!(scene, world)
  build_superObjs!(scene, world)

   # Initialize contact detection if contact detection desired and objects with contactMaterial are present
   if scene.visualize
      initializeVisualization(Modia3D.renderer[1], scene.allVisuElements)
   end
   if scene.collide
      initializeContactDetection!(world, scene)
   end
   initializeMassComputation!(scene)
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
   build_tree!(scene, world)
   build_superObjs!(scene, world)
   if scene.visualize
      initializeVisualization(Modia3D.renderer[1], scene.allVisuElements)
   end
   if scene.collide
      initializeContactDetection!(world, scene)
  end
  initializeMassComputation!(scene)

   # Initialize connections between signals and frames, joints, ...
   #build_SignalObject3DConnections!(assembly)

   scene.initAnalysis = true
end


#=
function initAnalysis!(assembly::Modia3D.AbstractAssembly)
  println("bin in initAnalysis!(assembly::Modia3D.AbstractAssembly)")
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
      build_superObjs!(scene, world)
      if scene.collide
        initializeContactDetection!(world, scene)
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
     Basics.emptyArray!(scene.superObjs)
     Basics.emptyArray!(scene.noCPairs)
     scene.collide = false

     scene.initSuperObj = false

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
