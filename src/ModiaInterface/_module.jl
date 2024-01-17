module ModiaInterface

export Model3D
export Object3D, Scene
export Visual, Solid
export Box, Beam, Cylinder, Sphere, Ellipsoid
export Cone, Capsule, GearWheel, Grid, SpringShape
export CoordinateSystem, FileMesh, ModelicaShape
export Font, TextShape
export VisualMaterial
export MassProperties, MassPropertiesFromShape
export MassPropertiesFromShapeAndMass
export UniformGravityField
export ptpJointSpace
export executeActions
export getRefPathPosition, getRefPathInitPosition, getVariables
export Fix, Free
export Revolute, RevoluteWithFlange
export Prismatic, PrismaticWithFlange
export singularRem, FreeMotion, change_rotSequenceInNextIteration!
export WorldForce, WorldTorque, Bushing, SpringDamperPtP, PolygonalContactModel
export SensorResult, ContactResult
export FreeMotion2

export ModelActions, ActionAttach, ActionRelease, ActionReleaseAndAttach, ActionDelete, EventAfterPeriod, ActionWait, ActionFlipChain
export addReferencePath

export build_Model3D!, get_animationHistory

import OrderedCollections
import Modia3D
import Modia3D.Composition
using  Modia
using  StaticArrays


# Modia models
include("$(Modia.path)/models/Blocks.jl")
include("$(Modia.path)/models/Electric.jl")
include("$(Modia.path)/models/Rotational.jl")
include("$(Modia.path)/models/Translational.jl")

include("buildModel3D.jl")
include("model3D.jl")

end
