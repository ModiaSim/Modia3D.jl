assignObj(scene, superObjType, obj, actPos) = nothing

# assign obj to superObjType.superObj in case it can collide
function assignObj(scene::Scene{F}, superObjType::SuperObjCollision{F}, obj::Object3D{F}, actPos::Int64)::Nothing where F <: Modia3D.VarFloatType
    if canCollide(obj)
        push!(superObjType.superObj, obj)
    end
    return nothing
end

# assign obj to superObjType.superObj in case it has mass
function assignObj(scene::Scene{F}, superObjType::SuperObjMass{F}, obj::Object3D{F}, actPos::Int64)::Nothing where F <: Modia3D.VarFloatType
    if featureHasMass(obj)
        push!(superObjType.superObj, obj)
    end
    return nothing
end

# assign obj to superObjType.superObj in case it can be moved
function assignObj(scene::Scene{F}, superObjType::SuperObjMovable{F}, obj::Object3D{F}, actPos::Int64)::Nothing where F <: Modia3D.VarFloatType
    if isMovable(obj)
        push!(superObjType.superObj, obj)
        append!(superObjType.superObj, reverse(obj.children) )
        obj.interactionManner.movablePos = actPos
        for child in obj.children
            child.interactionManner.movablePos = actPos
        end
    end
    return nothing
end

# assign obj to superObjType.superObj in case it can apply force/torque
function assignObj(scene::Scene{F}, superObjType::SuperObjForce{F}, obj::Object3D{F}, actPos::Int64)::Nothing where F <: Modia3D.VarFloatType
    if canCollide(obj) || hasJoint(obj) || hasForceElement(obj)
        push!(superObjType.superObj, obj)
    end
    return nothing
end

# assign obj to scene.pureResultObject3Ds in case it is pure result or visualization
function assignObj(scene::Scene{F}, superObjType::SuperObjResult{F}, obj::Object3D{F}, actPos::Int64)::Nothing where F <: Modia3D.VarFloatType
    if ( isUserDefined(obj) || isVisible(obj) || length(obj.visualizationFrame) > 0 ) && !hasJoint(obj) && !canCollide(obj) && !hasForceElement(obj) && !hasChildJoint(obj) #&& !hasCutJoint(obj) && !featureHasMass(obj)
        if !(obj in scene.pureResultObject3Ds)
            push!(scene.pureResultObject3Ds, obj)
        end
    end
    return nothing
end

# assign obj to field vectors of superObj
function assignAll(scene::Scene{F}, superObj::SuperObjsRow{F}, obj::Object3D{F}, world::Object3D{F}, actPos::Int64)::Nothing where F <: Modia3D.VarFloatType
    names = fieldnames(typeof(superObj))
    for val in names
        tmp = getfield(superObj, val)
        obj.interactionManner.originPos = actPos
        assignObj(scene, tmp, obj, actPos)
    end
    return nothing
end


function fillVisuElements!(scene::Scene{F}, obj::Object3D{F}, world::Object3D{F})::Nothing where F <: Modia3D.VarFloatType
    if isNotCoordinateSystem(obj) && obj.visualizeFrame != Modia3D.False &&
    ((scene.options.visualizeFrames && obj.visualizeFrame == Modia3D.Inherited) || obj.visualizeFrame == Modia3D.True)
        name = Symbol(obj.path, ".", "visualizationFrame")
        if obj != world
            addObject3DVisualizationFrame!(obj, scene.autoCoordsys, name)
        else
            addObject3DVisualizationFrame!(obj, Shapes.CoordinateSystem(length=2*scene.options.defaultFrameLength), name)
        end
        append!(scene.visualObject3Ds, obj.visualizationFrame)
        if !(obj in scene.pureResultObject3Ds)
            push!(scene.pureResultObject3Ds, obj)
        end
    end
    if isVisible(obj)
        push!(scene.visualObject3Ds, obj)
    end
    if isUserDefined(obj)
        push!(scene.userDefinedObject3Ds, obj)
    end
    return nothing
end


function assignAccVelo(tree::Vector{Object3D{F}}, obj::Object3D{F})::Nothing where F <: Modia3D.VarFloatType
    if hasChildJoint(obj) #|| hasCutJoint(obj)
        # compute acceleration of this object
        obj.computeAcceleration = true
        push!(tree, obj)
    elseif hasForceElement(obj) || canCollide(obj)
        # compute velocity of this object
        push!(tree, obj)
    end
    return nothing
end

assignAccVelo(tree, obj) = nothing
