# License for this file: MIT (expat)
# Copyright 2017-2018, DLR Institute of System Dynamics and Control


"""
    module ContactDetectionMPR

Detects contact between solid objects3D with the improved MPR algorithm.

For further information see [A. Neumayr and M. Otter, Collision Handling with Variable-Step Integrators, EOOLT'17]

# Main developers
Andrea Neumayr and Martin Otter, [DLR - Institute of System Dynamics and Control](https://www.dlr.de/sr/en)
"""
module ContactDetectionMPR

using StaticArrays

import Modia3D
import Modia3D.Basics
import Modia3D.Composition
import Modia3D.Solids
import ModiaMath

using StaticArrays
using DataStructures

@static if VERSION >= v"0.7.0-DEV.2005"
    const NOTHING = Nothing
else
    const NOTHING = Void
end

include("utilities.jl")
include("mpr.jl")
include("handler.jl")

end
