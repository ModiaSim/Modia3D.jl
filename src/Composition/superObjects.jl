mutable struct SuperObjCollision{F}
    superObj::Vector{Object3D{F}}
    function SuperObjCollision{F}() where {F}
        new(Vector{Object3D{F}}[])
    end
end

mutable struct SuperObjMass{F}
    superObj::Vector{Object3D{F}}
    function SuperObjMass{F}() where {F}
        new(Vector{Object3D{F}}[])
    end
end

mutable struct SuperObjMovable{F}
    superObj::Vector{Object3D{F}}
    function SuperObjMovable{F}() where {F}
        new(Vector{Object3D{F}}[])
    end
end

mutable struct SuperObjForce{F}
    superObj::Vector{Object3D{F}}
    function SuperObjForce{F}() where {F}
        new(Vector{Object3D{F}}[])
    end
end

mutable struct SuperObjVisu{F}
    superObj::Vector{Object3D{F}}
    function SuperObjVisu{F}() where {F}
        new(Vector{Object3D{F}}[])
    end
end



mutable struct SuperObjsRow{F}
    superObjCollision::SuperObjCollision{F}
    superObjMass::SuperObjMass{F}
    superObjMovable::SuperObjMovable{F}
    superObjForce::SuperObjForce{F}
    superObjVisu::SuperObjVisu{F}
    noCPair::Vector{Int64}
    function SuperObjsRow{F}() where {F}
        new(SuperObjCollision{F}(), SuperObjMass{F}(), SuperObjMovable{F}(), SuperObjForce{F}(), SuperObjVisu{F}(),  Vector{Int64}[])
    end
end
