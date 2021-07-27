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


include("renderer.jl")
include("handler.jl")

end
