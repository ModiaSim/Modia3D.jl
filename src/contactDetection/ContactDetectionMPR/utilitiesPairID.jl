# License for this file: MIT (expat)
# Copyright 2017-2018, DLR Institute of System Dynamics and Control
#
# This file is part of module
#   Modia3D.ContactDetectionMPR (Modia3D/contactDetection/ContactDetectionMPR/_module.jl)
#

const Point = SVector{3,Float64}


const i16max = Int64(typemax(Int16))
const i32max = Int64(typemax(Int32))

pack16(i1::Integer, i2::Integer) = Int64(i1) + i16max*Int64(i2)

function pack(i1::Integer, i2::Integer,i3::Integer,i4::Integer)
   @assert(i1 >= 0 && i1 <= typemax(Int16))
   @assert(i2 >= 0 && i2 <= typemax(Int16))
   @assert(i3 >= 0 && i3 <= typemax(Int16))
   @assert(i4 >= 0 && i4 <= typemax(Int16))
   return pack16(i1,i2) + i32max*pack16(i3,i4)
end


function unpack16(i::Int64)
    i1 = rem(i,i16max)
    i2 = div(i-i1, i16max)
    return (i1,i2)
end


function unpack32(i::Int64)
    i1 = rem(i,i32max)
    i2 = div(i-i1, i32max)
    return (i1,i2)
end


function unpack(i::Int64)
    @assert(i >= 0 && i <= typemax(Int64))
    (tmp1,tmp2) = unpack32(i)
    (i1,i2) = unpack16(tmp1)
    (i3,i4) = unpack16(tmp2)
    return (i1,i2,i3,i4)
end


### -------------------computation of pairID -----------------------------------
### it returns a unique ID
function orderPositions(is,i,js,j)
    if is < js
        return pack(is,i,js,j)
    elseif is > js
        return pack(js,j,is,i)
    else
        error("from orderPositions: is == js.")
end; end

#getPositionsOfObj(scene::Composition.Scene, obj::Composition.Object3D,
#                  movablePos::Nothing) = (false, 0, 0)
function getPositionsOfObj(scene::Composition.Scene,
            obj::Composition.Object3D, movablePos::Int64)
    if movablePos == 0
        return (false, 0, 0)
    else
        return (true, movablePos, findall(x->x==obj, scene.superObjs[movablePos].superObjMovable.superObj)[1] )
    end
end

function computePairID(scene::Composition.Scene,
        actObj::Composition.Object3D, nextObj::Composition.Object3D,
        is, i, js, j)
    # is: actual super - object
    # js: subsequent super - object
    # i: Object3D of is_th super - object
    # j: Object3D of js_th super - object
    (isSetAct, isPosActSuper, iPosActObj)    = getPositionsOfObj(scene, actObj,  actObj.interactionManner.movablePos)
    (isSetNext, jsPosNextSuper, jPosNextObj) = getPositionsOfObj(scene, nextObj, nextObj.interactionManner.movablePos)
    if isSetAct && isSetNext
        return orderPositions(isPosActSuper,iPosActObj,jsPosNextSuper,jPosNextObj)
    elseif isSetAct && !isSetNext
        return orderPositions(isPosActSuper,iPosActObj,js,j)
    elseif isSetNext && !isSetAct
        return orderPositions(is,i,jsPosNextSuper,jPosNextObj)
    else
        return orderPositions(is,i,js,j)
end; end
