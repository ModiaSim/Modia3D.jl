# License for this file: MIT (expat)
# Copyright 2017-2020, DLR Institute of System Dynamics and Control


"""
    module Modia3D.PathPlanning

Module for path planning and designing reference paths for Modia3D

# Main developers

Andrea Neumayr and Martin Otter, [DLR - Institute of System Dynamics and Control](https://www.dlr.de/sr/en)
"""
module PathPlanning

export PTP_path, pathEndTime, getPosition!, getPosition, getIndex, plotPath


export ptpJointSpace!

export getPathComponent

import Base
import Modia3D
import Modia3D.Basics
import Modia3D.Composition
import Modia3D.Shapes

using ModiaLang

using Unitful
using StaticArrays
using OrderedCollections

include("pathPlanning.jl")
include("referencePathInternal.jl")
include("referencePathUser.jl")

end
