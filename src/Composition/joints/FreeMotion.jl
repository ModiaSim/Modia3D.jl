# License for this file: MIT (expat)
# Copyright 2017-2018, DLR Institute of System Dynamics and Control
#
# This file is part of module
#   Modia3D.Composition (Modia3D/Composition/_module.jl)
#

# mutable struct of FreeMotion is defined in Modia3D/src/Composition/joints/object3DMotion.jl,
# since Object3D references FreeMotion and FreeMotion references Object3D


"""
    [angle1 angle2 angle3] = rot123fromR(R::SMatrix{3,3,F,9})

Return Cardan angles (rotation sequence x-y-z) from SMatrix{3,3,F,9} `R`.
In case of singular state (angle2 +-90 deg, x and z axes parallel) `rot3_guess` defines angle3.
"""
function rot123fromR(R::SMatrix{3,3,F,9}; rot3_guess=F(0.0) ) where {F}

    sbe = clamp(R[3,1], F(-1.0), F(1.0) )
    cbe2 = F(1.0) - sbe*sbe
    if (cbe2 > 1e-12)
        al = atan(-R[3,2], R[3,3])
        be = atan(sbe/sqrt(cbe2))
        ga = atan(-R[2,1], R[1,1])
    else
        # be is 90 deg -> singular, only al+ga is defined -> set ga to rot3_guess
        al = atan(R[2,3], R[2,2]) - sign(sbe)*F(rot3_guess)  # ???
        be = sign(sbe)*F(0.5*pi)
        ga = F(rot3_guess)
    end
    return SVector{3,F}([al, be, ga])

end

"""
    [angle1 angle2 angle3] = rot132fromR(R::SMatrix{3,3,F,9})

Return Cardan angles (rotation sequence x-z-y) from SMatrix{3,3,F,9} `R`.
In case of singular state (angle2 +-90 deg, x and y axes parallel) `rot3_guess` defines angle3.
"""
function rot132fromR(R::SMatrix{3,3,F,9}; rot3_guess=F(0.0) ) where {F}

    sga = clamp(-R[2,1], -1.0, 1.0)
    cga2 = 1.0 - sga*sga
    if (cga2 > 1e-12)
        al = atan(R[2,3], R[2,2])
        ga = atan(sga/sqrt(cga2))
        be = atan(R[3,1], R[1,1])
    else
        # ga is 90 deg -> singular, only al+be is defined -> set be to rot3_guess
        al = atan(-R[3,2], R[3,3]) - sign(sga)*F(rot3_guess)  # ???
        ga = sign(sga)*F(0.5*pi)
        be = F(rot3_guess)
    end
    return SVector{3,F}([al, ga, be])

end



"""
    w = wfromrot132(rot132::SVector{3,F}, derrot132::SVector{3,F})

Return relative rotational velocity SVector{3,F} `w` from frame `1` to frame `2` resolved in frame `2`.

`rot132` are the Cardan angles (rotation sequence x-z-y) of rotation from frame `1` to frame `2`.
`derrot132` are the time derivatives of `rot132`.
"""
function wfromrot132(rot132::SVector{3,F}, derrot132::SVector{3,F}) where {F}
    (sga, cga) = sincos(rot132[2])
    (sbe, cbe) = sincos(rot132[3])
    return SVector{3,F}([cbe*cga*derrot132[1] - sbe*derrot132[2],
                        -sga*derrot132[1]     + derrot132[3],
                         sbe*cga*derrot132[1] - cbe*derrot132[2]])

end
