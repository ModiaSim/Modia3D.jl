mutable struct SuperObjCollision{FloatType}
    superObj::Vector{Object3D{FloatType}}
    function SuperObjCollision{FloatType}() where {FloatType}
        new(Vector{Object3D{FloatType}}[])
    end
end

mutable struct SuperObjMass{FloatType}
    superObj::Vector{Object3D{FloatType}}
    function SuperObjMass{FloatType}() where {FloatType}
        new(Vector{Object3D{FloatType}}[])
    end
end

mutable struct SuperObjMovable{FloatType}
    superObj::Vector{Object3D{FloatType}}
    function SuperObjMovable{FloatType}() where {FloatType}
        new(Vector{Object3D{FloatType}}[])
    end
end

mutable struct SuperObjForce{FloatType}
    superObj::Vector{Object3D{FloatType}}
    function SuperObjForce{FloatType}() where {FloatType}
        new(Vector{Object3D{FloatType}}[])
    end
end

mutable struct SuperObjVisu{FloatType}
    superObj::Vector{Object3D{FloatType}}
    function SuperObjVisu{FloatType}() where {FloatType}
        new(Vector{Object3D{FloatType}}[])
    end
end



mutable struct SuperObjsRow{FloatType}
    superObjCollision::SuperObjCollision{FloatType}
    superObjMass::SuperObjMass{FloatType}
    superObjMovable::SuperObjMovable{FloatType}
    superObjForce::SuperObjForce{FloatType}
    superObjVisu::SuperObjVisu{FloatType}
    noCPair::Vector{Int64}
    function SuperObjsRow{FloatType}() where {FloatType}
        new(SuperObjCollision{FloatType}(), SuperObjMass{FloatType}(), SuperObjMovable{FloatType}(), SuperObjForce{FloatType}(), SuperObjVisu{FloatType}(),  Vector{Int64}[])
    end
end
