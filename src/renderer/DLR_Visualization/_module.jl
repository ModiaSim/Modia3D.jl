# License for this file: MIT (expat)
# Copyright 2017-2018, DLR Institute of System Dynamics and Control


"""
    module Modia3D.DLR_Visualization

Visualize Modia3D visual objects with the DLR Visualization engine.

# Main developers
Andrea Neumayr and Martin Otter, [DLR - Institute of System Dynamics and Control](https://www.dlr.de/sr/en)
"""
module DLR_Visualization

using StaticArrays

import Modia3D
import Modia3D.Basics
import Modia3D.Shapes
import Modia3D.Composition
import Measurements

using Libdl
ISWINDOWS() = Sys.iswindows()
ISLINUX()   = Sys.islinux()

include("renderer.jl")
include(joinpath("wrapper","simvis.jl"))
include("handler.jl")
include("visualize.jl")

end
