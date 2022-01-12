mutable struct SuperObjCollision{F <: AbstractFloat}
    superObj::Vector{Object3D{F}}
    function SuperObjCollision{F}() where F <: AbstractFloat
        new(Vector{Object3D{F}}[])
    end
end

mutable struct SuperObjMass{F <: AbstractFloat}
    superObj::Vector{Object3D{F}}
    function SuperObjMass{F}() where F <: AbstractFloat
        new(Vector{Object3D{F}}[])
    end
end

mutable struct SuperObjMovable{F <: AbstractFloat}
    superObj::Vector{Object3D{F}}
    function SuperObjMovable{F}() where F <: AbstractFloat
        new(Vector{Object3D{F}}[])
    end
end

mutable struct SuperObjForce{F <: AbstractFloat}
    superObj::Vector{Object3D{F}}
    function SuperObjForce{F}() where F <: AbstractFloat
        new(Vector{Object3D{F}}[])
    end
end

mutable struct SuperObjVisu{F <: AbstractFloat}
    superObj::Vector{Object3D{F}}
    function SuperObjVisu{F}() where F <: AbstractFloat
        new(Vector{Object3D{F}}[])
    end
end



mutable struct SuperObjsRow{F <: AbstractFloat}
    superObjCollision::SuperObjCollision{F}
    superObjMass::SuperObjMass{F}
    superObjMovable::SuperObjMovable{F}
    superObjForce::SuperObjForce{F}
    superObjVisu::SuperObjVisu{F}
    noCPair::Vector{Int64}
    function SuperObjsRow{F}() where F <: AbstractFloat
        new(SuperObjCollision{F}(), SuperObjMass{F}(), SuperObjMovable{F}(), SuperObjForce{F}(), SuperObjVisu{F}(),  Vector{Int64}[])
    end
end
