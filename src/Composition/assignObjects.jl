
function assignObj(scene::Scene, superObjType::SuperObjCollision, obj::Object3D)
    if canCollide(obj)
      push!(superObjType.superObj, obj)
    end
end


function assignObj(scene::Scene, superObjType::SuperObjMass, obj::Object3D)
    if hasMass(obj)
      push!(superObjType.superObj, obj)
    end
end


function assignObj(scene::Scene, superObjType::SuperObjForce, obj::Object3D)
    if canCollide(obj) || hasJoint(obj) || hasForceElement(obj)
    #println("ist ein ObjForce, canCollide = ", canCollide(obj), " hasJoint = ", hasJoint(obj), " hasForceElement = ", hasForceElement(obj))
      push!(superObjType.superObj, obj)
    end
end



function assignObj(scene::Scene, superObjType::SuperObjVisu, obj::Object3D)
    renderer = Modia3D.renderer[1]
    if isVisible(obj, renderer) && !hasJoint(obj) && !canCollide(obj) && !hasForceElement(obj) && !hasCutJoint(obj) # && !hasMass(obj)
      push!(scene.treeVisu, obj)
    end
end

assignObj(scene, superObjType, obj) = nothing


function assignAll(scene::Scene, superObj::SuperObjsRow, obj::Object3D, world::Object3D, actPos::Int64)
    names = fieldnames(typeof(superObj))
    for val in names
        tmp = getfield(superObj,val)
        assignObj(scene,tmp,obj)
    end

    if hasCutJoint(obj)
      for cutJoint in obj.twoObject3Dobject
        if typeof(cutJoint) <: Modia3D.AbstractJoint
          if cutJoint.visited
            push!(scene.noCPairsHelp[cutJoint], actPos)
          end
        end
      end
    end

#=
    if hasForceElement(obj)
        if !(obj.forceElement in scene.forceElements)
            push!(scene.forceElements,obj.forceElement)
        end
    end
=#
  return nothing
end


function assignDynamics!(obj::Object3D)
  obj.dynamics = Object3Ddynamics()
end


function createCutJoints!(scene::Scene, obj::Object3D)
  for cutJoint in obj.twoObject3Dobject
    if typeof(cutJoint) <: Modia3D.AbstractJoint
      if !cutJoint.visited
        println("\n... Cut-joint ", ModiaMath.instanceName(cutJoint), " pushed on scene.cutJoints vector")
        push!(scene.cutJoints, cutJoint)
        #println("length scene.cutJoints = ", length(scene.cutJoints))
        push!(scene.noCPairsHelp, cutJoint => [])
        #println("length scene.noCPairsHelp = ", length(scene.noCPairsHelp))
        cutJoint.visited = true
      end
    end
  end
end


function fillVisuElements!(scene::Scene, obj::Object3D, world::Object3D)
  renderer            = Modia3D.renderer[1]
  visualizeFrames     = scene.options.visualizeFrames
  enableVisualization = scene.options.enableVisualization
  if enableVisualization
    if visualizeFrames && isNotCoordinateSystem(obj) && obj.visualizeFrame != Modia3D.False
      if obj != world
        obj.visualizationFrame = copyObject3D(obj, scene.autoCoordsys)
      else
        obj.visualizationFrame = copyObject3D(obj, Graphics.CoordinateSystem(2*scene.options.defaultFrameLength))
      end
      push!(scene.allVisuElements, obj.visualizationFrame)
    end
    if isVisible(obj, renderer) # isVisible(obj.data, renderer)
       push!(scene.allVisuElements, obj)
    end
  end

  scene.visualize = length(scene.allVisuElements) > 0
  # println("length(scene.allVisuElements) = ", length(scene.allVisuElements))
  return nothing
end


function assign_Visu_CutJoint_Dynamics!(scene::Scene, obj::Object3D, world::Object3D)
  assignDynamics!(obj)
  createCutJoints!(scene, obj)
  fillVisuElements!(scene, obj, world)
end


function assignAccVelo(tree::Vector{Object3D}, obj::Object3D)
  # compute velocity of this object
  if canCollide(obj) && !hasChildJoint(obj)
    push!(tree, obj)
  end
  # compute acceleration of this object
  if hasChildJoint(obj) || hasCutJoint(obj)
    obj.computeAcceleration = true
    push!(tree, obj)
  end
end
assignAccVelo(tree, obj) = nothing
