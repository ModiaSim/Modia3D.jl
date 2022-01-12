assignObj(scene, superObjType, obj, actPos) = nothing

function assignObj(scene::Scene, superObjType::SuperObjCollision, obj::Object3D, actPos::Int64)
    if canCollide(obj)
        push!(superObjType.superObj, obj)
    end
end

function assignObj(scene::Scene, superObjType::SuperObjMass, obj::Object3D, actPos::Int64)
    if featureHasMass(obj)
        push!(superObjType.superObj, obj)
    end
end

function assignObj(scene::Scene, superObjType::SuperObjMovable, obj::Object3D, actPos::Int64)
    if isMovable(obj)
        push!(superObjType.superObj, obj)
        append!(superObjType.superObj, reverse(obj.children) )
        obj.interactionManner.movablePos = actPos
        obj.interactionManner.actualPos  = actPos
        for child in obj.children
            child.interactionManner.movablePos = actPos
            child.interactionManner.actualPos  = actPos
        end
    end
end

function assignObj(scene::Scene, superObjType::SuperObjForce, obj::Object3D, actPos::Int64)
    if canCollide(obj) || hasJoint(obj) || hasForceElement(obj)
        push!(superObjType.superObj, obj)
    end
end

function assignObj(scene::Scene, superObjType::SuperObjVisu, obj::Object3D, actPos::Int64)
    renderer = Modia3D.renderer[1]
    if ( isVisible(obj, renderer) || !isnothing(obj.visualizationFrame)  ) && !hasJoint(obj) && !canCollide(obj) && !hasForceElement(obj) && !hasChildJoint(obj) #&& !hasCutJoint(obj) && !featureHasMass(obj)
        # if an Object3D is for visualization only it is stored in updateVisuElements
        if !(obj in scene.updateVisuElements)
            push!(scene.updateVisuElements, obj)
        end
    end
end


function assignAll(scene::Scene, superObj::SuperObjsRow, obj::Object3D, world::Object3D, actPos::Int64)
    names = fieldnames(typeof(superObj))
    for val in names
        tmp = getfield(superObj,val)
        obj.interactionManner.originPos = actPos
        obj.interactionManner.actualPos = actPos
        assignObj(scene,tmp,obj,actPos)
    end
    return nothing
end


function fillVisuElements!(scene::Scene, obj::Object3D, world::Object3D)
    if scene.options.enableVisualization || scene.exportAnimation
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
        if isVisible(obj, Modia3D.renderer[1])  # todo: should not affect animation export; move to initializeVisualization?
            push!(scene.allVisuElements, obj)
        end
    end
    return nothing
end


function assignAccVelo(tree::Vector{Object3D{F}}, obj::Object3D) where F <: AbstractFloat
    if hasChildJoint(obj) #|| hasCutJoint(obj)
        # compute acceleration of this object
        obj.computeAcceleration = true
        push!(tree, obj)
    elseif hasForceElement(obj) || canCollide(obj)
        # compute velocity of this object
        push!(tree, obj)
    end
end

assignAccVelo(tree, obj) = nothing
