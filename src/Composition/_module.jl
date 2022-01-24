# License for this file: MIT (expat)
# Copyright 2017-2018, DLR Institute of System Dynamics and Control

"""
    module Modia3D.Composition

Structuring of objects moving in 3D. Most important constructors
(dof are the `degrees-of-freedom`):

| Function                                    | dof | Description |
|:--------------------------------------------|:----:|:---------------------------------------------|
| [`Object3D`](@ref)`(...)`                  | 0,6 | Return a reference Object3D, if a parent is given fixed/moving w.r.t. `parent`|
| [`Modia3D.Revolute`](@ref)`(obj1,obj2;..)`  | 1 | Return a revolute joint |
| [`Modia3D.Prismatic`](@ref)`(obj1,obj2;..)` | 1 | Return a prismatic joint |



# Main developers
Andrea Neumayr and Martin Otter, [DLR - Institute of System Dynamics and Control](https://www.dlr.de/sr/en)
"""
module Composition

export initialize, initAnalysis!, performAnalysis!, closeAnalysis!, visualize!, visualizeWorld!
export updatePosition!, update!
export Object3D
export RotationVariables, RCardan123
export WStartVariables, WCartesian, WCardan123
export SimulationModel, Model
export printObject3DAndAllChildren, writeObject3DAndAllChildrenOnJsonFile
export FixTranslation
export Revolute, setAngle!, connect
export Prismatic, setDistance!
export FreeMotion

export multibodyResiduals!, setModiaJointVariables!

export distanceAndAngles, distance, planarRotationAngle
export measFrameRotation, measFramePosition, measFrameDistance
export measFrameRotVelocity, measFrameTransVelocity, measFrameDistVelocity
export measFrameRotAcceleration, measFrameTransAcceleration, measFrameDistAcceleration
export applyFrameTorque!, applyFrameForce!, applyFrameForceTorque!
export applyFrameTorquePair!, applyFrameForcePair!, applyFrameForceTorquePair!

export deleteMaterialLastContactDictContactEnd

export NoGravityField, UniformGravityField, PointGravityField, gravityAcceleration
export G, EarthMass, EarthRadius
export Scene
export upwardsDirection, cameraPosition
export animationData, animationStep

export DLR_Visualization, DLR_Visualization_renderer

export ContactDetectionMPR_handler
export initializeContactDetection!, selectContactPairsNoEvent!, selectContactPairsWithEvent!, getDistances!, setComputationFlag, closeContactDetection!

export responseCalculation, print_ModelVariables
export contactStart, contactEnd, regularize, resultantCoefficientOfRestitution, resultantDampingCoefficient
export ElasticContactPairResponseMaterial, elasticContactPairCoefficients, normalRelativeVelocityAtContact

export isFree, isNotFree, changeJointFromFreeMotionToFix!, changeJointFromFixToFreeMotion!

export fullName, instanceName

# export updateSimulationModel!

export InteractionBehavior, Gripper, Movable, Lockable, NoInteraction

export rot123fromR, rot132fromR, Rfromrot123, Rfromrot132

export supportPoint, boundingBox!, contactPointIsLocallyBijectiveToNormal

@enum InteractionBehavior Gripper Movable Lockable NoInteraction


using StaticArrays
using OrderedCollections
using LinearAlgebra
using Unitful

import Base
import Modia3D
import Modia3D.Basics
import Modia3D.Frames
import Modia3D.Shapes
import JSON
import Printf
import ModiaLang
import TimerOutputs

include(joinpath("joints", "object3DMotion.jl"))
include("object3D.jl")
include("supportPoints.jl")
include("superObjects.jl")

# Joints and Force Elements must be included before scene, because memory is stored in scene
include(joinpath("joints", "FreeMotion.jl"))
include(joinpath("joints", "Fix.jl"))
include(joinpath("joints", "Revolute.jl"))
include(joinpath("joints", "Prismatic.jl"))

include(joinpath("ForceElements", "Bushing.jl"))
include(joinpath("ForceElements", "SpringDamperPtP.jl"))

include("contactPairs.jl")
include(joinpath(Modia3D.path, "src", "contactDetection", "ContactDetectionMPR", "ContactDetectionMPR_handler.jl"))

include("scene.jl") # must be included after superObjects.jl
include(joinpath("joints", "joints.jl"))
include(joinpath("joints", "changeJointState.jl"))

include("sensors.jl")
include("frameMeasurements.jl")
include("frameForceTorque.jl")
include("assignObjects.jl")
include(joinpath("responseCalculation", "othersCollisionResponse.jl"))
include(joinpath("responseCalculation", "elasticCollisionResponse.jl"))
include("massPropertiesComputation.jl")
include("printObject3D.jl")
include("handler.jl")
include("responseCalculation.jl")
include("dynamicCollision.jl")
include("dynamics.jl")

end
