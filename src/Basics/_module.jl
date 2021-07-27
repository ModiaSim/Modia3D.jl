# License for this file: MIT (expat)
# Copyright 2017-2020, DLR Institute of System Dynamics and Control


"""
    module Modia3D.Basics

Utility constants and functions for Modia3D

# Main developers

Andrea Neumayr and Martin Otter, [DLR - Institute of System Dynamics and Control](https://www.dlr.de/sr/en)
"""
module Basics

export trailingPartOfName

export neps, sign_eps, radToDeg, degToRad
export getAndCheckFullLibraryPath, getEnvironmentVariable
export zeroMVector, onesMVector, nullMRotation
export ZeroMVector
export normalizeVector, BoundingBox

export readDictOfStructsFromJSON
export listKeys
export deleteItem

export linearMovement
export PositionMVector, RotationMMatrix, assertRotationMatrix

using  StaticArrays
using  LinearAlgebra
using  JSON
using  Unitful

include("environment.jl")
include("constantsAndFunctions.jl")

end
