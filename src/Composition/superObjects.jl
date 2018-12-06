mutable struct SuperObjCollision
    superObj::Array{Object3D,1}
    function SuperObjCollision()
        new(Array{Object3D,1}())
    end
end
function assignObj(superObjType::SuperObjCollision, obj::Object3D)
    if canCollide(obj)
      push!(superObjType.superObj, obj)
    end
end


mutable struct SuperObjMass
    superObj::Array{Object3D,1}
    function SuperObjMass()
        new(Array{Object3D,1}())
    end
end
function assignObj(superObjType::SuperObjMass, obj::Object3D)
    if hasMass(obj)
      push!(superObjType.superObj, obj)
    end
end


mutable struct SuperObjForce
    superObj::Array{Object3D,1}
    function SuperObjForce()
        new(Array{Object3D,1}())
    end
end
function assignObj(superObjType::SuperObjForce, obj::Object3D)
    if canCollide(obj) || hasJoint(obj) || hasForceElement(obj)
      push!(superObjType.superObj, obj)
    end
end


mutable struct SuperObjVisu
    superObj::Array{Object3D,1}
    function SuperObjVisu()
        new(Array{Object3D,1}())
    end
end
function assignObj(superObjType::SuperObjVisu, obj::Object3D)
    if isVisible(obj) && !hasJoint(obj) && !hasMass(obj) && !canCollide(obj) && !hasForceElement(obj) && !hasCutJoint(obj)
      push!(superObjType.superObj, obj)
    end
end


mutable struct SuperObjsRow
    superObjCollision::SuperObjCollision
    superObjMass::SuperObjMass
    superObjForce::SuperObjForce
    superObjVisu::SuperObjVisu
    noCPair::Array{Int64,1}
    function SuperObjsRow()
        new(SuperObjCollision(), SuperObjMass(), SuperObjForce(), SuperObjVisu(), Array{Int64,1}())
    end
end


mutable struct SuperObjCollection
    superObjs::Array{SuperObjsRow,1}
    allVisuElements::Array{Object3D,1}
    # forceElements::Array{Int64,1}
    cutJoints::Array{Modia3D.AbstractJoint,1}
    noCPairs::Array{Array{Int64,1},1}
    noCPairsHelp::Array{Array{Int64,1},1}
    function SuperObjCollection()
        new(Array{SuperObjsRow,1}(),
            Array{Object3D,1}(),
            Array{Modia3D.AbstractJoint,1}(),
            Array{Array{Int64,1},1}(),
            Array{Array{Int64,1},1}())
    end
end
