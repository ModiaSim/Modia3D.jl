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
import Modia3D.Graphics
import Modia3D.Solids
import Modia3D.Composition

@static if VERSION >= v"0.7.0-DEV.2005"
    const NOTHING = Nothing
else
    const NOTHING = Void
end

include(joinpath("wrapper","simvis.jl"))

if !simVisInfo.isNoRenderer
   include("handler.jl")
   include("visualize.jl")

   if simVisInfo.isCommercialEdition
      include("visualize_commercialEdition.jl")
   end
end

end