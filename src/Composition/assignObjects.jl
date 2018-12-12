
function assignObj(superObjType::SuperObjCollision, obj::Object3D)
    if canCollide(obj)
      push!(superObjType.superObj, obj)
    end
end

function assignObj(superObjType::SuperObjMass, obj::Object3D)
    if hasMass(obj)
      push!(superObjType.superObj, obj)
    end
end


function assignObj(superObjType::SuperObjForce, obj::Object3D)
    #=
    if canCollide(obj) || hasJoint(obj) || hasForceElement(obj)
      push!(superObjType.superObj, obj)
    end
    =#
end



function assignObj(superObjType::SuperObjVisu, obj::Object3D)
    #=
    if isVisible(obj) && !hasJoint(obj) && !hasMass(obj) && !canCollide(obj) && !hasForceElement(obj) && !hasCutJoint(obj)
      push!(superObjType.superObj, obj)
    end
    =#
end

assignObj(superObjType, obj) = nothing # println("das ist ein dummy wert")


function assignAll(scene::Scene, superObj::SuperObjsRow, obj::Object3D, actPos::Int64)
    names = fieldnames(typeof(superObj))
    for val in names
        # println("val $val")
        tmp = getfield(superObj,val)
        assignObj(tmp,obj)
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

    fillVisuElements(scene, obj)
end


function fillVisuElements(scene::Scene, obj::Object3D)
  renderer            = Modia3D.renderer[1]
  allVisuElements     = scene.allVisuElements
  options             = scene.options
  visualizeFrames     = options.visualizeFrames
  enableVisualization = options.enableVisualization

  if enableVisualization
    if visualizeFrames && isNotCoordinateSystem(obj) && obj.visualizeFrame != Modia3D.False
      if isWorld(obj)
        obj.visualizationFrame = copyObject3D(obj, Graphics.CoordinateSystem(2*options.defaultFrameLength))
      else
        obj.visualizationFrame = copyObject3D(obj, scene.autoCoordsys)
      end
      println("es kommt ein KoordinatenSystem dazu!!")
      push!(allVisuElements, obj.visualizationFrame)
    end
    if isVisible(obj.data, renderer)
       # Visualize Object3D
       println("bin im Visible!!")
       push!(allVisuElements, obj)
    end
  end
#=
  if isWorld(obj)
    if visualizeFrames && isNotCoordinateSystem(obj) && obj.visualizeFrame != Modia3D.False
      obj.visualizationFrame = copyObject3D(obj, Graphics.CoordinateSystem(2*options.defaultFrameLength))
      push!(allVisuElements, obj.visualizationFrame)
    end
  elseif enableVisualization && isNotWorld(obj)
    if visualizeFrames && isNotCoordinateSystem(obj) && obj.visualizeFrame != Modia3D.False
      obj.visualizationFrame = copyObject3D(obj, scene.autoCoordsys)
      push!(allVisuElements, obj.visualizationFrame)
    end
    if isVisible(obj.data, renderer)
       # Visualize Object3D
       push!(allVisuElements, obj)
    end
  end
=#
  scene.visualize = length(scene.allVisuElements) > 0
  println("length(scene.allVisuElements) = ", length(scene.allVisuElements))
  return nothing
end
