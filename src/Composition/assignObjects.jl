assignObj(scene, superObjType, obj, actPos) = nothing

function assignObj(scene::Scene{F}, superObjType::SuperObjCollision{F}, obj::Object3D{F}, actPos::Int64)::Nothing where F <: Modia3D.VarFloatType
    if canCollide(obj)
        push!(superObjType.superObj, obj)
    end
    return nothing
end

function assignObj(scene::Scene{F}, superObjType::SuperObjMass{F}, obj::Object3D{F}, actPos::Int64)::Nothing where F <: Modia3D.VarFloatType
    if featureHasMass(obj)
        push!(superObjType.superObj, obj)
    end
    return nothing
end

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

function assignObj(scene::Scene{F}, superObjType::SuperObjForce{F}, obj::Object3D{F}, actPos::Int64)::Nothing where F <: Modia3D.VarFloatType
    if canCollide(obj) || hasJoint(obj) || hasForceElement(obj)
        push!(superObjType.superObj, obj)
    end
    return nothing
end

function assignObj(scene::Scene{F}, superObjType::SuperObjVisu{F}, obj::Object3D{F}, actPos::Int64)::Nothing where F <: Modia3D.VarFloatType
    renderer = Modia3D.renderer[1]
    if ( isVisible(obj, renderer) || isVisible(obj, scene.exportAnimation) || !isnothing(obj.visualizationFrame)  ) && !hasJoint(obj) && !canCollide(obj) && !hasForceElement(obj) && !hasChildJoint(obj) #&& !hasCutJoint(obj) && !featureHasMass(obj)
        # if an Object3D is for visualization/animation export only it is stored in updateVisuElements
        if !(obj in scene.updateVisuElements)
            push!(scene.updateVisuElements, obj)
        end
    end
    return nothing
end


function assignAll(scene::Scene{F}, superObj::SuperObjsRow{F}, obj::Object3D{F}, world::Object3D{F}, actPos::Int64)::Nothing where F <: Modia3D.VarFloatType
    names = fieldnames(typeof(superObj))
    for val in names
        tmp = getfield(superObj,val)
        obj.interactionManner.originPos = actPos
        assignObj(scene,tmp,obj,actPos)
    end
    return nothing
end


function fillVisuElements!(scene::Scene{F}, obj::Object3D{F}, world::Object3D{F})::Nothing where F <: Modia3D.VarFloatType
    if scene.options.enableVisualization || scene.provideAnimationData
        if isNotCoordinateSystem(obj) && obj.visualizeFrame != Modia3D.False &&
        ((scene.options.visualizeFrames && obj.visualizeFrame == Modia3D.Inherited) || obj.visualizeFrame == Modia3D.True)
            name = Symbol(obj.path, ".", "visualizationFrame")
            if obj != world
                addObject3DVisualizationFrame!(obj, scene.autoCoordsys, name)
            else
                addObject3DVisualizationFrame!(obj, Shapes.CoordinateSystem(length=2*scene.options.defaultFrameLength), name)
            end
            append!(scene.allVisuElements, obj.visualizationFrame)
            # if an Object3D is for visualization only it is stored in updateVisuElements
            if !(obj in scene.updateVisuElements)
                push!(scene.updateVisuElements, obj)
            end
        end
        if isVisible(obj, Modia3D.renderer[1]) || isVisible(obj, scene.exportAnimation)  # visible in visualization or animation export
            push!(scene.allVisuElements, obj)
        end
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
