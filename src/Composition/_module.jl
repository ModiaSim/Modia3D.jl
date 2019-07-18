# License for this file: MIT (expat)
# Copyright 2017-2018, DLR Institute of System Dynamics and Control

"""
    module Modia3D.Composition

Structuring of objects moving in 3D. Most important constructors
(dof are the `degrees-of-freedom`):

| Function                                    | dof  | Description                                  |
|:--------------------------------------------|:----:|:---------------------------------------------|
| [`@assembly`](@ref)` Name(..) begin .. end` |   -  | Return a `Name` constructor for Object3Ds    |
| [`Object3D`](@ref)`([data];..)`             |   0  | Return a reference Object3D                  |
| [`Object3D`](@ref)`(parent [, data];..)`    |  0,6 | Return Object3D fixed/moving w.r.t. `parent` |
| [`Modia3D.Revolute`](@ref)`(obj1,obj2;..)`  |   1  | Return a revolute joint                      |
| [`Modia3D.Prismatic`](@ref)`(obj1,obj2;..)` |   1  | Return a prismatic joint                     |

The optional `data` associated with an `Object3D`
can be one of the following:

| `data`                            | Description                                          |
|:----------------------------------|:-----------------------------------------------------|
| `::`[`Modia3D.Solid`](@ref)       | Solids with geometry, mass, visual/contact material  |
| `<:Modia3D.AbstractVisualElement` | Visual elements ([`Modia3D.Graphics`](@ref))         |



# Main developers
Andrea Neumayr and Martin Otter, [DLR - Institute of System Dynamics and Control](https://www.dlr.de/sr/en)
"""
module Composition

export @assembly

export initialize, initAnalysis!, performAnalysis!, closeAnalysis!, visualize!, visualizeAssembly!, visualizeWorld!
export updatePosition!, update!, driveJoint!
export Object3D, AssemblyInternal, initAssemblyInternal!, initAssemblyComponent!, Scene
export Part
export SimulationModel
export BodyWithTwoFrames, ContactBox, ContactBox2
export printObject3DAndAllChildren, writeObject3DAndAllChildrenOnJsonFile
export set_r!, set_q!, set_r_abs!
export Revolute, setAngle!, connect, addRevoluteTorqueObject, computeTorque
export Prismatic, setDistance!
export FreeMotion
export getModelResidues!
export distanceAndAngles, distance, planarRotationAngle

export Flange, RevoluteFlange

export SignalToFlangeAngle, SignalToFlangeTorque
export computeSignal

export AdaptorForceElementToFlange, AdaptorForceElementToPFlange

export setVariable_phi!, setVariable_r_rel!, setVariable_R_rel!


export SceneOptions, DLR_Visualization, ContactDetectionMPR
export NoGravityField, UniformGravityField, PointGravityField, gravityAcceleration
export G, EarthMass, EarthRadius

export DLR_Visualization_renderer

export ContactDetectionMPR_handler
export initializeContactDetection!, selectContactPairsNoEvent!, selectContactPairsWithEvent!, getDistances!, setComputationFlag, closeContactDetection!

export responseCalculation, print_ModelVariables
export contactStart, contactEnd, regularize, resultantCoefficientOfRestitution, resultantDampingCoefficient
export ElasticContactPairResponseMaterial, elasticContactPairCoefficients, normalRelativeVelocityAtContact

using StaticArrays
using DataStructures


import Base
import ModiaMath
import Modia3D
import Modia3D.Basics
import Modia3D.Solids
import Modia3D.Graphics
import Modia3D.Signals
import JSON
import Printf

const NOTHING = Nothing


const MaterialOrNothing                = Union{Graphics.Material,NOTHING}
const MassPropertiesOrNothing          = Union{Solids.MassProperties,NOTHING}
const AbstractContactMaterialOrNothing = Union{Modia3D.AbstractContactMaterial,NOTHING}

include("flange.jl")
include("object3D.jl")
include(joinpath("responseCalculation", "elasticCollisionResponse.jl"))
include(joinpath("joints", "FixedJoint.jl"))
include(joinpath("joints", "FreeMotion.jl"))
include(joinpath("superObjects.jl"))
include("scene.jl")
include("assembly.jl")
include("sensors.jl")
include("joints.jl")
include(joinpath("joints", "Revolute.jl"))
include(joinpath("joints", "Prismatic.jl"))
include("adaptors.jl")
include("assemblies.jl")
include("assignObjects.jl")
include("massComputation.jl")
include("handler.jl")
include("responseCalculation.jl")
include("dynamics.jl")

end
