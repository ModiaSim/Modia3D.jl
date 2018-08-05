# License for this file: MIT (expat)
# Copyright 2017-2018, DLR Institute of System Dynamics and Control


"""
    module Modia3D.ForceElements

Elements generating forces or torques that act on objects.

# Main developers
Andrea Neumayr and Martin Otter, [DLR - Institute of System Dynamics and Control](https://www.dlr.de/sr/en)
"""
module ForceElements

export @forceElement

using  StaticArrays
import DataStructures
import ModiaMath

import Base
import Modia3D
import Modia3D.Basics

include("forces.jl")
end
