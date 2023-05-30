# License for this file: MIT (expat)
# Copyright 2017-2018, DLR Institute of System Dynamics and Control


i16max()::Int64 = Int64(typemax(Int16))
i32max()::Int64 = Int64(typemax(Int32))

pack16(i1::Int64, i2::Int64)::Int64 = Int64(i1) + i16max()*Int64(i2)

function pack(i1::Int64, i2::Int64,i3::Int64,i4::Int64)::Int64
    @assert(i1 >= 0 && i1 <= typemax(Int16))
    @assert(i2 >= 0 && i2 <= typemax(Int16))
    @assert(i3 >= 0 && i3 <= typemax(Int16))
    @assert(i4 >= 0 && i4 <= typemax(Int16))
    return pack16(i1,i2) + i32max()*pack16(i3,i4)
end


function unpack16(i::Int64)::Tuple{Int64, Int64}
    i1 = rem(i,i16max())
    i2 = div(i-i1, i16max())
    return (i1,i2)
end


function unpack32(i::Int64)::Tuple{Int64, Int64}
    i1 = rem(i,i32max())
    i2 = div(i-i1, i32max())
    return (i1,i2)
end


function unpack(i::Int64)::Tuple{Int64, Int64, Int64, Int64}
    @assert(i >= 0 && i <= typemax(Int64))
    (tmp1,tmp2) = unpack32(i)
    (i1,i2) = unpack16(tmp1)
    (i3,i4) = unpack16(tmp2)
    return (i1,i2,i3,i4)
end


### -------------------computation of pairID -----------------------------------
### it returns a unique ID
function orderPositions(is::Int64, i::Int64, js::Int64, j::Int64)::Int64
    p = 0
    if is < js
        p = pack(is,i,js,j)
    elseif is > js
        p = pack(js,j,is,i)
    else
        error("from orderPositions: is == js.")
    end
    return p
end


function computePairID(scene::Composition.Scene{F},
        actObj::Composition.Object3D{F}, nextObj::Composition.Object3D{F},
        is::Int64, i::Int64, js::Int64, j::Int64)::Int64 where F

    return orderPositions(is,i,js,j)
end
