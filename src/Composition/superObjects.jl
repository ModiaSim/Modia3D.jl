mutable struct SuperObjCollision{F <: Modia3D.VarFloatType}
    superObj::Vector{Object3D{F}}
    function SuperObjCollision{F}() where F <: Modia3D.VarFloatType
        new(Vector{Object3D{F}}[])
    end
end

mutable struct SuperObjMass{F <: Modia3D.VarFloatType}
    superObj::Vector{Object3D{F}}
    function SuperObjMass{F}() where F <: Modia3D.VarFloatType
        new(Vector{Object3D{F}}[])
    end
end

mutable struct SuperObjMovable{F <: Modia3D.VarFloatType}
    superObj::Vector{Object3D{F}}
    function SuperObjMovable{F}() where F <: Modia3D.VarFloatType
        new(Vector{Object3D{F}}[])
    end
end

mutable struct SuperObjForce{F <: Modia3D.VarFloatType}
    superObj::Vector{Object3D{F}}
    function SuperObjForce{F}() where F <: Modia3D.VarFloatType
        new(Vector{Object3D{F}}[])
    end
end

mutable struct SuperObjVisu{F <: Modia3D.VarFloatType}
    superObj::Vector{Object3D{F}}
    function SuperObjVisu{F}() where F <: Modia3D.VarFloatType
        new(Vector{Object3D{F}}[])
    end
end



mutable struct SuperObjsRow{F <: Modia3D.VarFloatType}
    superObjCollision::SuperObjCollision{F}
    superObjMass::SuperObjMass{F}
    superObjMovable::SuperObjMovable{F}
    superObjForce::SuperObjForce{F}
    superObjVisu::SuperObjVisu{F}
    noCPair::Vector{Int64}
    function SuperObjsRow{F}() where F <: Modia3D.VarFloatType
        new(SuperObjCollision{F}(), SuperObjMass{F}(), SuperObjMovable{F}(), SuperObjForce{F}(), SuperObjVisu{F}(),  Vector{Int64}[])
    end
end
