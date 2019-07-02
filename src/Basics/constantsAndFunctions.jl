# License for this file: MIT (expat)
# Copyright 2017-2018, DLR Institute of System Dynamics and Control
#
# This file is part of module
#   Modia3D.Basics (Modia3D/Basics/_module.jl)
#


# Epsilon and sign
const neps = sqrt( eps() )
sign_eps(value::Float64; seps::Float64 = 100*neps)::Float64 = value > seps ? 1.0 : (value < -seps ? -1.0 : 0.0)

function normalizeVector(n::SVector{3,Float64})
  nabs = norm(n)
  if nabs <= neps
    @assert(nabs > neps) # && norm(vec) > eps()
    return nothing
  else
    return n/nabs
  end
end

# MVector / MMatrix
zeroMVector()   = MVector{3,Float64}(0.0, 0.0, 0.0)
onesMVector()   = MVector{3,Float64}(1.0, 1.0, 1.0)
nullMRotation() = MMatrix{3,3,Float64,9}(1.0,0.0,0.0,0.0,1.0,0.0,0.0,0.0,1.0)

# Standard constants
const degToRad      = pi/180.0
const radToDeg      = 180.0/pi
const ZeroMVector   = MVector{3,Float64}(0.0,0.0,0.0)
const NoNameDefined = "_NoNameDefined"


# Standard types
const PositionMVector = MVector{3,Float64}
const RotationMMatrix = MMatrix{3,3,Float64,9}


"""    mutable struct BoundingBox - Smallest box that contains a visual element"""
mutable struct BoundingBox
   x_min::Float64
   x_max::Float64
   y_min::Float64
   y_max::Float64
   z_min::Float64
   z_max::Float64
   BoundingBox() = new(0.0,0.0,0.0,0.0,0.0,0.0)
   BoundingBox(x_min,x_max,y_min,y_max,z_min,z_max) = new(x_min,x_max,y_min,y_max,z_min,z_max)
end


# Time functions
linearMovement(delta_x, tStart, tEnd, time) = delta_x*(time-tStart)/(tEnd-tStart)


# Trailing part of type name
function trailingPartOfTypeAsString(obj)::String
   name = string( typeof(obj) )

   # Determine trailing solid (after last ".")
   i = first(something(findlast(".", name), 0:-1))
   return i > 0 && i < length(name) ? name[i+1:end] : name
end

function trailingPartOfName(name::AbstractString)::String
   # Determine trailing part of name (after last ".")
   i = first(something(findlast(".", name), 0:-1))
   return i > 0 && i < length(name) ? name[i+1:end] : name
end


"""
    readDictOfStructsFromJSON(fileName, StructType)

Read a JSON file from `fileName` and return a `Dict{String, StructType}` dictionary.
`StructType` must be a mutable struct type with a constructor `StructType()`.
"""
function readDictOfStructsFromJSON(fileName, StructType)
    dict1 = JSON.parsefile(fileName)
    palette = Dict{String, StructType}()
    for (key1,value1) in dict1
        obj = StructType()
        for (key2,value2) in value1
            setfield!(obj, Symbol(key2), value2)
        end
        palette[key1] = obj
    end
    return palette
end


"""
    listKeys(dict)

List the keys of dictionary `dict` in sorted order.
"""
function listKeys(dict)
    for key in sort(collect(keys(dict)))
        if typeof(key) == String
            println("    \"$key\"")
        else
            println("   $key")
        end
    end
end
