# License for this file: MIT (expat)
# Copyright 2017-2018, DLR Institute of System Dynamics and Control


"""
    module Modia3D.NoRenderer

No renderer (so no animations are shown).

# Main developers
Andrea Neumayr and Martin Otter, [DLR - Institute of System Dynamics and Control](https://www.dlr.de/sr/en)
"""
module NoRenderer

import Modia3D
import Modia3D.Composition
import Modia3D.Solids

@static if VERSION >= v"0.7.0-DEV.2005"
    const NOTHING = Nothing
else
    const NOTHING = Void
end

include("renderer.jl")
include("handler.jl")

end