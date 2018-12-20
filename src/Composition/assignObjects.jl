
function assignObj(superObjType::SuperObjCollision, obj::Object3D)
    if canCollide(obj)
      println("can collidieren")
      push!(superObjType.superObj, obj)
    end
end

function assignObj(superObjType::SuperObjMass, obj::Object3D)
  #=
    if hasMass(obj)
      println("hat masse")
      push!(superObjType.superObj, obj)
    end
    =#
end


function assignObj(superObjType::SuperObjForce, obj::Object3D)
  #=
    if canCollide(obj) || hasJoint(obj) || hasForceElement(obj)
      println("ist ein ObjForce")
      push!(superObjType.superObj, obj)
    end
    =#
end



function assignObj(superObjType::SuperObjVisu, obj::Object3D)
  #=
    renderer = Modia3D.renderer[1]
    if isVisible(obj, renderer) && !hasJoint(obj) && !hasMass(obj) && !canCollide(obj) && !hasForceElement(obj) && !hasCutJoint(obj)
      println("ist ein VisuObj")
      push!(superObjType.superObj, obj)
    end
    =#
end

assignObj(superObjType, obj) = nothing # println("das ist ein dummy wert")


function assignAll(scene::Scene, superObj::SuperObjsRow, obj::Object3D, world::Object3D, actPos::Int64)
    names = fieldnames(typeof(superObj))
    for val in names
        # println("val $val")
        #tmp = getfield(scene,superObj,val)
        #assignObj(tmp,obj)
    end

#=
    if hasCutJoints(obj)
        if !(obj.cutJoint in scene.cutJoints)
            push!(scene.cutJoints, obj.cutJoint)
            push!(scene.noCPairsHelp, [actPos])
        else
            i = indexin(obj.cutJoint, scene.cutJoints)
            push!(scene.noCPairsHelp[i][], actPos)
        end
    end

    if hasForceElement(obj)
        if !(obj.forceElement in scene.forceElements)
            push!(scene.forceElements,obj.forceElement)
        end
    end
=#
  return nothing
end


function fillVisuElements(scene::Scene, obj::Object3D, world::Object3D)
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


function createCutJoints(scene::Scene, obj::Object3D)
  for cutJoint in obj.twoObject3Dobject
    if typeof(cutJoint) <: Modia3D.AbstractJoint
      if !cutJoint.visited
        println("\n... Cut-joint ", ModiaMath.instanceName(cutJoint), " pushed on scene.cutJoints vector")
        push!(scene.cutJoints, cutJoint)
        cutJoint.visited = true
      end
    end
  end
end
