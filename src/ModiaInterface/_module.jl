module ModiaInterface

export Object3D, Scene, Visual, Solid
export Box, Beam, Cylinder, Sphere, Ellipsoid
export Cone, Capsule, GearWheel, Grid, SpringShape
export CoordinateSystem, FileMesh, ModelicaShape
export Font, TextShape
export VisualMaterial
export MassProperties, MassPropertiesFromShape
export MassPropertiesFromShapeAndMass
export UniformGravityField
export RefPath, ptpJointSpace, scheduleReferenceMotion
export calculateRobotMovement
export getRefPathPosition, getRefPathInitPosition, getVariables
export multibodyResiduals!, setModiaJointVariables!
export Fix
export Revolute, RevoluteWithFlange
export Prismatic, PrismaticWithFlange
export J123, J132, J123or132, singularRem, FreeMotion, change_rotSequenceInNextIteration!

export buildModia3D

import Modia3D
using ModiaLang

# ModiaLang models
include("$(ModiaLang.path)/models/Blocks.jl")
include("$(ModiaLang.path)/models/Electric.jl")
include("$(ModiaLang.path)/models/Rotational.jl")
include("$(ModiaLang.path)/models/Translational.jl")

include("buildModia3D.jl")
include("model3D.jl")

end
