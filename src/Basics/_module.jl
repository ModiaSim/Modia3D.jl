# License for this file: MIT (expat)
# Copyright 2017-2020, DLR Institute of System Dynamics and Control


"""
    module Modia3D.Basics

Utility constants and functions for Modia3D

# Main developers

Andrea Neumayr and Martin Otter, [DLR - Institute of System Dynamics and Control](https://www.dlr.de/sr/en)
"""
module Basics

import Modia3D
import Modia3D.Modia.OrderedCollections

export trailingPartOfName

export nepsType, neps, sign_eps, radToDeg
export getAndCheckFullLibraryPath, getEnvironmentVariable

export normalizeVector, BoundingBox, convertToFloat64

export readDictOfStructsFromJSON
export listKeys
export deleteItem

export linearMovement

using  StaticArrays
using  LinearAlgebra
using  JSON
using  Unitful
import Measurements
import MonteCarloMeasurements

include("environment.jl")
include("constantsAndFunctions.jl")

end
