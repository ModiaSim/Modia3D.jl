
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
    if canCollide(obj) || hasJoint(obj) || hasForceElement(obj)
      push!(superObjType.superObj, obj)
    end
end

function assignObj(superObjType::SuperObjVisu, obj::Object3D)
    if isVisible(obj) && !hasJoint(obj) && !hasMass(obj) && !canCollide(obj) && !hasForceElement(obj) && !hasCutJoint(obj)
      push!(superObjType.superObj, obj)
    end
end
#=
function assignAll(coll::Scene, superObj::SuperObjsRow, obj::Object3D, actPos::Int64)
    names = fieldnames(typeof(superObj))
    for val in names
        tmp = getfield(superObj,val)
        assignObj(tmp,obj)
    end

    if hasCutJoints(obj)
        if !(obj.cutJoint in coll.cutJoints)
            push!(coll.cutJoints,obj.cutJoint)
            push!(coll.noCPairsHelp, [actPos])
        else
            i = indexin(obj.cutJoint,coll.cutJoints)
            push!(coll.noCPairsHelp[i][], actPos)
        end
    end

    if hasForceElement(obj)
        if !(obj.forceElement in coll.forceElements)
            push!(coll.forceElements,obj.forceElement)
        end
    end

    if isVisible(obj)
        push!(coll.allVisuElements,obj)
    end
end
=#
