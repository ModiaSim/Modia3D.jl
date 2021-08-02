mutable struct SuperObjCollision
    superObj::Array{Object3D,1}
    function SuperObjCollision()
        new(Array{Object3D,1}())
    end
end

mutable struct SuperObjMass
    superObj::Array{Object3D,1}
    function SuperObjMass()
        new(Array{Object3D,1}())
    end
end

mutable struct SuperObjMovable
    superObj::Array{Object3D,1}
    function SuperObjMovable()
        new(Array{Object3D,1}())
    end
end

mutable struct SuperObjForce
    superObj::Array{Object3D,1}
    function SuperObjForce()
        new(Array{Object3D,1}())
    end
end

mutable struct SuperObjVisu
    superObj::Array{Object3D,1}
    function SuperObjVisu()
        new(Array{Object3D,1}())
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
