mutable struct SuperObjCollision
    superObj::Vector{Object3D}
    function SuperObjCollision()
        new(Vector{Object3D}[])
    end
end

mutable struct SuperObjMass
    superObj::Vector{Object3D}
    function SuperObjMass()
        new(Vector{Object3D}[])
    end
end

mutable struct SuperObjMovable
    superObj::Vector{Object3D}
    function SuperObjMovable()
        new(Vector{Object3D}[])
    end
end

mutable struct SuperObjForce
    superObj::Vector{Object3D}
    function SuperObjForce()
        new(Vector{Object3D}[])
    end
end

mutable struct SuperObjVisu
    superObj::Vector{Object3D}
    function SuperObjVisu()
        new(Vector{Object3D}[])
    end
end



mutable struct SuperObjsRow
    superObjCollision::SuperObjCollision
    superObjMass::SuperObjMass
    superObjMovable::SuperObjMovable
    superObjForce::SuperObjForce
    superObjVisu::SuperObjVisu
    noCPair::Vector{Int64}
    function SuperObjsRow()
        new(SuperObjCollision(), SuperObjMass(), SuperObjMovable(), SuperObjForce(), SuperObjVisu(),  Vector{Int64}[])
    end
end
