var documenterSearchIndex = {"docs": [

{
    "location": "index.html#",
    "page": "Home",
    "title": "Home",
    "category": "page",
    "text": ""
},

{
    "location": "index.html#Modia3D-1",
    "page": "Home",
    "title": "Modia3D",
    "category": "section",
    "text": "Modia3D is a Julia package to model fixed and  moving objects in 3D (e.g. visual shapes, rigid bodies). These objects are driven kinematically by pre-defined time functions or are moving dynamically by  solving Differential Algebraic Equations (DAEs) with a variable-step DAE solver.Collision handling with elastic response calculation is performed for objects that are defined with a contact material and (a) have a convex geometry, or (b) can be approximated by a set of convex geometries, or (c) have a concave geometry that is (automatically) approximated by its convex hull.  Papers about Modia3D:Collision Handling with Variable-Step Integrators (EOOLT 2017, December)\nComponent-Based 3D Modeling Combined with Equation-Based Modeling, accepted for publication at the American Modelica Conference 2018, October 9-10Before releasing version 1.0, Modia3D shall be easily combinable with Modia, for example to define a controlled  electrical motor with Modia, and add 3D behavior/visualization with Modia3D. By this approach the best of both worlds can be combined: Special 3D algorithms (Modia3D) + power/flexibility of equation based modeling (Modia)."
},

{
    "location": "index.html#Package-Features-1",
    "page": "Home",
    "title": "Package Features",
    "category": "section",
    "text": "A 3D object is an instance of struct Modia3D.Object3D and defines a coordinate system moving in 3D together with associated data and properties. The following Object3Ds are currently supported:"
},

{
    "location": "index.html#Object3Ds-with-a-solid-part-1",
    "page": "Home",
    "title": "Object3Ds with a solid part",
    "category": "section",
    "text": "Solid parts can be associated with a Modia3D.Object3D.  They are defined with struct Modia3D.Solid consisting of an optional solid geometry:(Image: Solids)and other optional properties:mass propreties (defined by geometry+material-name, geometry+density, or directly defined mass properties),\ncontact material (for elastic response calculation),\nvisualization material (for visualization, see below).Since the solid geometry itself is optional, it is possible to just define a coordinate system with associated mass and inertia matrix.The following functions are provided for a solid geometry geo that is associated with an Object3D object3D:volume(geo),\ncentroid(geo),\ninertiaMatrix(geo, mass),\nboundingBox(geo, <other arguments>),\nsupportPoint(geo, <other arguments>),\nisVisible(object3D, renderer),\nhasMass(object3D),\ncanCollide(object3D),\nand other functions."
},

{
    "location": "index.html#Object3Ds-for-visualization-1",
    "page": "Home",
    "title": "Object3Ds for visualization",
    "category": "section",
    "text": "Visualization elements that have a visualization material:(Image: VisuElements)A visualiziation material has the following attributes:color (name or rgb-value),\nwireframe (false/true),\ntransparency (0.0 is opaque, 1.0 is fully transparent),\nreflectslight (false/true),\nshininess (0.0 is matte surface, 1.0 is very shiny),\nshadowMask (defines whether or not an object casts or receives shadows)Visualization elements that have no visualization material:(Image: Fonts)It is planned to support all other visualization elements that are available in the DLR Visualization library (see videos of this library)."
},

{
    "location": "index.html#Constraints-on-Object3Ds-1",
    "page": "Home",
    "title": "Constraints on Object3Ds",
    "category": "section",
    "text": "An Object3D can be either fixed or freely moving with respect to another Object3D. In the latter case, the movement is described by relative quaternions.Furthermore, two Object3Ds can be connected together via various joint types. Currently, revolute and prismatic joints are supported. In the near future, more joint types will be added."
},

{
    "location": "index.html#Assemblies-of-Object3Ds-1",
    "page": "Home",
    "title": "Assemblies of Object3Ds",
    "category": "section",
    "text": "Object3D definitions can be collected together with the Modia3D.@assembly macro in hierarchical structures. For example, in the following four bar mechanism (consisting of 3 bars and the ground as 4th bar), a bar is defined as an assembly consisting of a light-blue SolidBeam Object3D and two red Cylinder Object3Ds. Such a bar is then in turn assembled in the assembly FourBar shown below:(Image: fourbar)There are the following operations on an instance of an assembly:Modia3D.visualizeAssembly!(assembly) to visualize the initial configuration of the assembly without simulating anything.\nModia3D.SimulationModel(assembly; analysis=xxx, <other arguments>) to generate a simulationModel of the assembly that can be simulated with ModiaMath.simulate!.  The analysis keyword defines which analysis shall be carried out on the model. Currently supported are KinematicAnalysis to kinematically move the assembly, or DynamicAnalysis to solve the equations of motion of the assembly. In the future it is planned to support QuasiStaticAnalysis as well."
},

{
    "location": "index.html#Main-developers-1",
    "page": "Home",
    "title": "Main developers",
    "category": "section",
    "text": "Andrea Neumayr and  Martin OtterDLR - Institute of System Dynamics and ControlLicense: MIT (expat)"
},

{
    "location": "man/GettingStarted.html#",
    "page": "Getting Started",
    "title": "Getting Started",
    "category": "page",
    "text": ""
},

{
    "location": "man/GettingStarted.html#Getting-Started-1",
    "page": "Getting Started",
    "title": "Getting Started",
    "category": "section",
    "text": ""
},

{
    "location": "man/GettingStarted.html#Installation-1",
    "page": "Getting Started",
    "title": "Installation",
    "category": "section",
    "text": "Modia3D is not yet registered in METADATA.jland need to be installed with Pkg.clone:julia> Pkg.clone(\"https://github.com/ModiaSim/ModiaMath.jl\")\r\njulia> Pkg.clone(\"https://github.com/ModiaSim/Modia3D.jl\")Modia3D performs simulation and plotting with ModiaMath. ModiaMath in turn uses PyPlot as basis for the plotting. Since installation of PyPlot is not robust with the automatic installation procedure of current Julia, it is recommended to first install PyPlot as described in the installation procedure of ModiaMath.Modia3D visualizes the movement of 3D objects with a renderer. Currently, the (free) community or the (commercial) professional version of the DLR Visualization library are supported. To install the free version for Windows perform the following steps (the free Linux version will become available in a few days):Go to https://visualization.ltx.de/, provide your contact information and click on Request download for Community Edition. Afterwards, you get a link to download the library and you need to unzip the file.\nIn your HOME/.juliarc.jl file, include the environment variable ENV[\"DLR_VISUALIZATION\"] = \"<path-to-library>/Visualization/Extras/SimVis\".\nStart Julia and run one of the examples, for example include(\"$(Modia3D.path)/examples/dynamics/Simulate_DoublePendulumWithDampers.jl\")If Modia3D cannot use one of the renderers above, it will continue with renderer NoRenderer that is animation is switched off."
},

{
    "location": "man/GettingStarted.html#To-run-examples-1",
    "page": "Getting Started",
    "title": "To run examples",
    "category": "section",
    "text": "  include(\"$(Modia3D.path)/examples/dynamics/Simulate_DoublePendulumWithDampers.jl\")\r\n  include(\"$(Modia3D.path)/examples/dynamics/Simulate_FallingBall4.jl\")\r\n  include(\"$(Modia3D.path)/examples/kinematics/Move_FourBar.jl\")\r\n  include(\"$(Modia3D.path)/examples/visual/Move_AllVisualObjects.jl\")"
},

{
    "location": "man/GettingStarted.html#To-run-tests-1",
    "page": "Getting Started",
    "title": "To run tests",
    "category": "section",
    "text": "  include(\"$(Modia3D.path)/test/runtests.jl\")"
},

{
    "location": "man/Examples.html#",
    "page": "Examples",
    "title": "Examples",
    "category": "page",
    "text": ""
},

{
    "location": "man/Examples.html#Examples-1",
    "page": "Examples",
    "title": "Examples",
    "category": "section",
    "text": "Modia3D has examples in directory$(Modia3D.path)/examplesto demonstrate various features of the package. Every example is in a separate module. Most important examples (by pasting the corresponding include(..) command in the REPL, the example is executed):  import Modia3D\r\n\r\n  # Examples to demonstrate the visualization capabilities of Modia3D\r\n  include(\"$(Modia3D.path)/examples/visual/Visualize_Solids.jl\")\r\n  include(\"$(Modia3D.path)/examples/visual/Visualize_GeometriesWithMaterial.jl\")\r\n  include(\"$(Modia3D.path)/examples/visual/Visualize_GeometriesWithoutMaterial.jl\")\r\n  include(\"$(Modia3D.path)/examples/visual/Visualize_Text.jl\")\r\n  include(\"$(Modia3D.path)/examples/visual/Visualize_TextFonts.jl\")\r\n  include(\"$(Modia3D.path)/examples/visual/Move_AllVisualObjects.jl\")\r\n\r\n  # Examples to demonstrate kinematic movement of assemblies\r\n  include(\"$(Modia3D.path)/examples/kinematics/Move_DoublePendulum.jl\")\r\n  include(\"$(Modia3D.path)/examples/kinematics/Move_FourBar.jl\")\r\n  include(\"$(Modia3D.path)/examples/kinematics/Move_FourBar2.jl\")\r\n\r\n  # Examples to demonstrate dynamic simulation without force elements\r\n  include(\"$(Modia3D.path)/examples/dynamics/Simulate_Pendulum.jl\")\r\n  include(\"$(Modia3D.path)/examples/dynamics/Simulate_DoublePendulum.jl\")\r\n\r\n  # Examples to demonstrate dynamic simulation with force elements connected to joints\r\n  include(\"$(Modia3D.path)/examples/dynamics/Simulate_PendulumWithDamper.jl\")\r\n  include(\"$(Modia3D.path)/examples/dynamics/Simulate_PendulumWithController.jl\")\r\n  include(\"$(Modia3D.path)/examples/dynamics/Simulate_DoublePendulumWithDampers.jl\")\r\n\r\n  # Examples to demonstrate dynamic simulation with collision handling\r\n  include(\"$(Modia3D.path)/examples/dynamics/Simulate_FallingBall2.jl\")\r\n  include(\"$(Modia3D.path)/examples/dynamics/Simulate_FallingBall4.jl\")"
},

{
    "location": "man/Plans.html#",
    "page": "Plans for version 1.0",
    "title": "Plans for version 1.0",
    "category": "page",
    "text": ""
},

{
    "location": "man/Plans.html#Plans-for-version-1.0-1",
    "page": "Plans for version 1.0",
    "title": "Plans for version 1.0",
    "category": "section",
    "text": "Modia3D is not yet ready and should not be used for production work. The features below are planned to be implemented before Modia3D version 1.0. We are very interested that other people help and contribute to Modia3D. If you would like to implement one of the features below (or any other features), please contact Martin.Otter@dlr.de.Features planned for version 1.0:General\nImproving efficiency: Debugging Modia3D with respect to unnecessary heap allocations during integration (especially, changing some of the remaining MVector{3,Float64} to SVector{3,Float64}).\nImproved documentation.\nChanging to Julia 0.7.ContactDetectionMPR\nThe MPR algorithm for distance computation needs to be more thoroughly tested.\nAdd an option to getDistances(ch) to only compute the distances of contact pairs that had a negative distance in the last call of selectContactPairs(ch) (to enhance efficiency of the simulation with a variable-step integrator).Solids\nMaking contact handling more robust and more thoroughly testing the contact response calculation.\nDefinition of meshes directly in Julia (not only via *.obj files).\nMap all non-triangles of *.obj files to triangles, in order that further operations (like volume computation) can operate on them.Renderer\nInterfacing the remaining features of the DLR Visualization library to Modia3D/Julia (cameras and paths, flexible surface, elastically deformed mesh, fluid flow visualization, head.up displays, effects, large terrains, ...).Composition\nVisualizing (optionally) the direction of the gravity vector if a parallel gravity field is used.\nImproving efficiency: Structure spanning tree in two parts: One only needed for kinematic/dynamic analysis and the remaining part only needed for visualization (need only to be evaluated at communication points).\nSupport of more joint types (Spherical, Cylindrical, Planar, GearConstraint, ...).\nSupport of 3D force/torque objects.\nAutomatic handling of 2D and 3D kinematic loops for kinematic and dynamic analysis (currently only 2D kinematic loops are supported for kinematic analysis).\nSupport of analytic loop handling (along the line of the Assembly joints of the Modelica Standard Library, but fully automatic).\nUsing Modia components with input/output ports as signal generators or as force elements in Modia3D.\nUsing a Modia3D model as component in Modia.\nOptionally supporting an O(n) algorithm for the spanning tree.\nFor models that can be completely mapped to an ODE (= tree-structure with O(n) algorithm and all kinematic loops can be solved with the analytic loop handling), optionally an ODE integrator of DifferentialEquations.jl can be used for simulation.\nSupport for 1-dimensional heat transfer data attached to an Object3D (to make sure that further extensions of the concept in non-mechanical domains are possible)."
},

{
    "location": "man/Plans.html#Plans-after-version-1.0-1",
    "page": "Plans for version 1.0",
    "title": "Plans after version 1.0",
    "category": "section",
    "text": "The following features are planed after version 1.0 (especially together with interested collaborators):Solids\nSupport of operations on meshes with triangles to construct new meshes (extrusion, CSG) along the paper Generic Modelica Framework for MultiBody Contacts and Discrete Element Method.GUI\nSupport of an open source, web-browser-based GUI to graphically define 3D assemblies and use the GUI also as default renderer (along the paper 3D Schematics of Modelica Models and Gamification;  probably using three.js as underlying web-browser, webgml based rendering engine).Composition\nSupport for hard contacts with impulses.\nAttaching other domains to Object3Ds (heat transfer, fluid flow, ...)."
},

{
    "location": "lib/Composition.html#",
    "page": "Composition",
    "title": "Composition",
    "category": "page",
    "text": ""
},

{
    "location": "lib/Composition.html#Modia3D.Composition",
    "page": "Composition",
    "title": "Modia3D.Composition",
    "category": "module",
    "text": "module Modia3D.Composition\n\nStructuring of objects moving in 3D. Most important constructors (dof are the degrees-of-freedom):\n\nFunction dof Description\n@assemblyName(..) begin .. end - Return a Name constructor for Object3Ds\nObject3D([data];..) 0 Return a reference Object3D\nObject3D(parent [, data];..) 0,6 Return Object3D fixed/moving w.r.t. parent\nModia3D.Revolute(obj1,obj2;..) 1 Return a revolute joint\nModia3D.Prismatic(obj1,obj2;..) 1 Return a prismatic joint\n\nThe optional data associated with an Object3D can be one of the following:\n\ndata Description\n::Modia3D.Solid Solids with geometry, mass, visual/contact material\n<:Modia3D.AbstractVisualElement Visual elements (Modia3D.Graphics)\n\nMain developers\n\nAndrea Neumayr and Martin Otter, DLR - Institute of System Dynamics and Control\n\n\n\n"
},

{
    "location": "lib/Composition.html#Modia3D.Composition.Object3D",
    "page": "Composition",
    "title": "Modia3D.Composition.Object3D",
    "category": "type",
    "text": "obj1 = Object3D([data]; visualizeFrame=Modia3D.Inherited)\nobj2 = Object3D(parent [, data]; fixed=true, r=zeros(3), R=nothing, q=nothing, \n                v_start=zeros(3), w_start=zeros(3), \n                visualizeFrame=Modia3D.Inherited)\n\nGenerate a new Object3D object, that is a coordinate system (= frame) with associated data. If parent is present, the Object3D is defined relatively to the parent Object3D. If parent is not present, the Object3D is either a reference object (such as the world-object), or the object is connected later with a joint to another  Object3D. If fixed=true, the object is rigidly connect to its parent; otherwise it is moving freely relative to its parent (mathematically described by quaternions).\n\nNote, there are many convenience functions in ModiaMath.Frames to generate a ModiaMath.RotationMatrix R or a ModiaMath.Quaternion q.\n\nArguments\n\ndata::Modia3D.AbstractObject3Ddata: Optional data associated with Object3D.\nparent::Object3D: Parent object.\nfixed::Bool:\nfixed = true, if the Object3D is fixed relatively to its parent Object3D at position r,R,q. It is best to provide the rotation information via R in this case. \nfixed = false, if Object3D can move freely relatively to its parent Object3D and is   initially placed at r,R,q. The movement is internally described with Quaternion vector q.  Therefore, it is best to provide the rotation information via q in this case.\nr::AbstractVector: Initial relative position vector from frame of parent object to   origin of frame object, resolved in parent frame.\nR::Union{ModiaMath.RotationMatrix,Void}: Initial rotation matrix defining the rotation   from frame of parent object to frame of Object3D. If both R = nothing and q = nothing,  a null rotation is defined.\nq::Union{ModiaMath.Quaternion,Void}: Initial quaternion defining the rotation  from frame of parent object to frame of Object3D. If both R = nothing and q = nothing,  a null rotation is defined.\nv_start::AbstractVector: If fixed=false, initial velocity of the origin of Object3D with respect to parent, resolved in parent frame.\nw_start::AbstractVector: If fixed=false, initial angular velocity of Object3D with respect to parent, resolved in Object3D.\nvisualizeFrame::Union{Bool,Modia3D.Ternary}: Coordinate system of Object3D is always (= true), or never (= false) visualized, or it is visualized if defined in SceneOptions(...) (= Modia3D.Inherited).\n\nExamples\n\nusing Modia3D\n\n# Define assembly\n@assembly MyAssembly begin\n   world = Object3D()\n\n   # Frame fixed in world\n   frame1 = Object3D(world; r=[0.1, 0.2, 0.3])\n\n   # Frame moving relatively to frame1\n   r2     = [0.2, 0.2, 0.3]\n   frame2 = Object3D(frame1; fixed=false, r=r2)\n\n   # Frame moving relatively to world\n   frame3 = Object3D(world; fixed=false, r=-r2)\nend\nModia3D.visualizeAssembly!(MyAssembly())\n\n\n\n"
},

{
    "location": "lib/Composition.html#Modia3D.Composition.PointGravityField",
    "page": "Composition",
    "title": "Modia3D.Composition.PointGravityField",
    "category": "type",
    "text": "PointGravityField(mass), PointGravityField(;mue=G*EarthMass)\n\nReturn a PointGravityField struct with the gravity field constant mue (mue = G*mass).\n\nExample\n\nimport Modia3D\n\ngrav = Modia3D.PointGravityField()   # Gravity field of earth\n   r = Modia3D.EarthRadius\n   g = gravityAcceleration(grav,r)\n\n\n\n"
},

{
    "location": "lib/Composition.html#Modia3D.Composition.SimulationModel",
    "page": "Composition",
    "title": "Modia3D.Composition.SimulationModel",
    "category": "type",
    "text": "simModel = SimulationModel(assembly::Modia3D.AbstractAssembly;\n                           analysis::ModiaMath.AnalysisType=ModiaMath.DynamicAnalysis,\n                           startTime = 0.0, stopTime  = 1.0, tolerance = 1e-4,\n                           interval  = (stopTime-startTime)/500.0)\n\nGenerate a simulationModel from an assembly generated with macro Modia3D.@assembly and the type of analysis to be carried out on the assembly. Additionally, default startTime, stopTime, tolerance, interval for the simulation engine are defined. These values should be adapted so that assembly-specific, meaningful defaults are provided.\n\n\n\n"
},

{
    "location": "lib/Composition.html#Modia3D.Composition.UniformGravityField",
    "page": "Composition",
    "title": "Modia3D.Composition.UniformGravityField",
    "category": "type",
    "text": "UniformGravityField(;g=9.81, n=[0,1,0])\n\nReturn a UniformGravityField struct.\n\nArguments\n\ng::Float64: Gravity constant\nn::AbstractVector: Direction of gravity\n\nExample\n\nimport Modia3D\n\ngrav = Modia3D.UniformGravityField()\n   r = Modia3D.EarthRadius\n   g = gravityAcceleration(grav,r)\n\n\n\n"
},

{
    "location": "lib/Composition.html#Modia3D.Composition.Prismatic-Tuple{Modia3D.Composition.Object3D,Modia3D.Composition.Object3D}",
    "page": "Composition",
    "title": "Modia3D.Composition.Prismatic",
    "category": "method",
    "text": "joint = Modia3D.Prismatic(obj1::Object3D, obj2::Object3D;\n                          axis=1, s_start=0, v_start=0, canCollide=false)\n\nReturn a joint object that constrains the movement of obj2::Object3D with respect to obj1::Object3D along coordinate axis axis (axis = 1,2,3). The initial position/velocity of obj2 with respect to obj1 along axis is s_start [m] and v_start [m/s], respectively. If canCollide=false, no collision detection will occur between obj1 and obj2 (and Object3Ds that are directly or indirectly rigidly fixed to obj1 or obj2).\n\nIf a Prismatic joint closes a kinematic loop, then the already present objects must be consistent to the Prismatic joint that is the frames of obj1 and obj2 must be parallel to each other and movement of obj1 along its axis axis with s_start results in obj2. If s_start=NaN,  its value is computed in this case.\n\nExamples\n\nusing Modia3D\nimport ModiaMath\n\n@assembly FallingBall(;h=1.0) begin\n   world  = Object3D()\n   sphere = Object3D( Modia3D.Solid(Modia3D.SolidSphere(0.1), \"Aluminium\") )\n\n   # Constrain sphere movement (initial placement at position [0,h,0])\n   prismatic = Modia3D.Prismatic(world, sphere, axis=2, s_start=h)  \nend\n\nsimulationModel = Modia3D.SimulationModel( FallingBall(h=1.5), stopTime=1.0 )\nresult          = ModiaMath.simulate!(simulationModel)\nModiaMath.plot(result, (\"prismatic.s\", \"prismatic.v\"))\n\n\n\n"
},

{
    "location": "lib/Composition.html#Modia3D.Composition.Revolute-Tuple{Modia3D.Composition.Object3D,Modia3D.Composition.Object3D}",
    "page": "Composition",
    "title": "Modia3D.Composition.Revolute",
    "category": "method",
    "text": "joint = Modia3D.Revolute(obj1, obj2; axis=3, phi_start=0, w_start=0, canCollide=false)\n\nReturn a Revolute joint that rotates obj1::Object3D into obj2::Object3D along the z-axis of obj1. The initial start angle is phi_start and the initial angular velocity is w_start. If canCollide=false, no collision detection will occur between obj1 and obj2 (and Object3Ds that are directly or indirectly rigidly fixed to obj1 or obj2).\n\nIf a Revolute joint closes a kinematic loop, then the already present objects must be consistent to the Revolute joint that is the frames of obj1 and obj2 must be parallel to each other and rotation of obj1 along its axis axis with phi_start results in obj2. If phi_start=NaN, its value is computed in this case.\n\nExamples\n\nusing Modia3D\nimport ModiaMath\n\n@assembly Pendulum(;L=1.0) begin\n   world  = Modia3D.Object3D()\n   frame1 = Modia3D.Object3D()\n   rev    = Modia3D.Revolute(world, frame1)\n   sphere = Modia3D.Object3D(frame1, Modia3D.Solid(Modia3D.SolidSphere(0.1), \"Aluminium\"),\n                             r = [L,0,0] )\nend\n\nsimulationModel = Modia3D.SimulationModel( Pendulum(L=0.8), stopTime=5.0 )\nresult          = ModiaMath.simulate!(simulationModel)\nModiaMath.plot(result, (\"rev.phi\", \"rev.w\"))\n\n\n\n"
},

{
    "location": "lib/Composition.html#Modia3D.Composition.distance-Tuple{Modia3D.Composition.Object3D,Modia3D.Composition.Object3D}",
    "page": "Composition",
    "title": "Modia3D.Composition.distance",
    "category": "method",
    "text": "d = distance(frame1, frame2)\n\nReturn the distance between the origin of frame1 and the origin of frame2\n\n\n\n"
},

{
    "location": "lib/Composition.html#Modia3D.Composition.planarRotationAngle-Tuple{AbstractArray{T,1} where T,AbstractArray{T,1} where T,AbstractArray{T,1} where T}",
    "page": "Composition",
    "title": "Modia3D.Composition.planarRotationAngle",
    "category": "method",
    "text": "angle = planarRotationAngle(e, v1, v2)\n      = planarRotationAngle(frame1, frame2)\n\nReturn angle of a planar rotation, given the rotation axis e (a unit vector) and the representations of a vector in frame 1 (v1) and frame 2 (v2).\n\nUnder the assumption that the z-axes of frame1 and frame2 coincide, return the angle between the x-axis of frame1 and the position vector from frame1 to frame2.\n\n\n\n"
},

{
    "location": "lib/Composition.html#Modia3D.Composition.visualizeAssembly!-Tuple{Modia3D.AbstractAssembly}",
    "page": "Composition",
    "title": "Modia3D.Composition.visualizeAssembly!",
    "category": "method",
    "text": "visualizeAssembly!(assembly::Modia3D.AbstractAssembly)\n\nVisualize the assembly defined with macro Modia3D.@assembly in its initial configuration (but without simulating it).\n\n\n\n"
},

{
    "location": "lib/Composition.html#Modia3D.Composition.@assembly-Tuple{Any,Any}",
    "page": "Composition",
    "title": "Modia3D.Composition.@assembly",
    "category": "macro",
    "text": "@assembly AssemblyName(arguments) begin ... end\n\nReturn the constructor for a new struct AssemblyName consisting of Object3Ds that are connected together. The new struct consists of all left-hand-side (scalar or vector) symbols present between begin ... end.\n\nExamples\n\nusing Modia3D\n\n@assembly Bar(;Lx = 0.1, Ly=Lx/5, Lz=Ly) begin\n   frame0 = Modia3D.Object3D(Modia3D.Solid(Modia3D.SolidBeam(Lx,Ly,Lz), nothing, vmat1))\n   frame1 = Modia3D.Object3D(frame0; r=[-Lx/2, 0.0, 0.0])\n   frame2 = Modia3D.Object3D(frame0; r=[ Lx/2, 0.0, 0.0])\n   cyl1   = Modia3D.Object3D(frame1, cyl)\n   cyl2   = Modia3D.Object3D(frame2, cyl)\nend\nbar = Bar(;Lx=1.0)\nModia3D.visualizeAssembly!( bar )\n\n\n\n"
},

{
    "location": "lib/Composition.html#Composition-1",
    "page": "Composition",
    "title": "Composition",
    "category": "section",
    "text": "Modules = [Modia3D.Composition]\r\nPrivate = false"
},

{
    "location": "lib/Graphics.html#",
    "page": "Graphics",
    "title": "Graphics",
    "category": "page",
    "text": ""
},

{
    "location": "lib/Graphics.html#Modia3D.Graphics",
    "page": "Graphics",
    "title": "Modia3D.Graphics",
    "category": "module",
    "text": "module Modia3D.Graphics\n\nVisual elements used for animation. The visual elements are passed to an external renderer. Currently, the (free) community edition and the (commercial) professional editions of the DLR Visualization library are supported (not all visualization elements of this library are yet interfaced to Modia3D). Modia3D is designed so that other renderers can be supported as well.\n\nVisualization elements with geometry and visualization material:\n\n(Image: VisuElements)\n\nVisualization elements that do not have a visualization material:\n\n(Image: VisuElementsWithout)\n\nMain developers\n\nAndrea Neumayr and Martin Otter, DLR - Institute of System Dynamics and Control ```\n\n\n\n"
},

{
    "location": "lib/Graphics.html#Modia3D.Graphics.Font",
    "page": "Graphics",
    "title": "Modia3D.Graphics.Font",
    "category": "type",
    "text": "font = Modia3D.Font(;fontFamily=\"FreeSans\", bold=false, italic=false, charSize=0.1,\n                     color=\"LightBlue\", transparency=0.0)\n\nReturn a Font definition.\n\nArguments\n\nfontFamily::String: Font family (\"FreeSans\", \"FreeSerif\", \"Arial\", \"ArialNarrow\", \"CourierNew\", \"TimesNewRoman\", or \"Verdana\").\nbold::Bool: = true, if bold font.\nitalic::Bool: = true, if italic font.\ncharSize::Number: Character size in [m].\ncolor::Modia3D.RGBColor: Color; Examples: rgb(\"Blue\"), rgb([0,0,255]), rgb(0,0,255).\ntransparency::Number: 0.0 (opaque) ... 1.0 (transparent)\n\nExamples\n\nimport Modia3D\nfont1 = Modia3D.Font()\nfont2 = Modia3D.Font(fontFamily=\"Arial\", bold=true, charSize=0.2, \n                     color=\"LightBlue\", transparency=0.5)\n\n\n\n"
},

{
    "location": "lib/Graphics.html#Modia3D.Graphics.Material",
    "page": "Graphics",
    "title": "Modia3D.Graphics.Material",
    "category": "type",
    "text": "material = Modia3D.Material(;color=defaultColor(), wireframe=false, transparency=0.0,\n                             reflectslight=true, shininess=0.7, \n                             shadowMask=CastsAndReceivesShadows))\n\nReturn a material object that defines attributes for the visualization of an Object3D  that has visual or solid properties.\n\nArguments\n\ncolor: This argument is passed to function Modia3D.rgb(color) to return the RGB color value in          form of a vector. E.g. color=\"Red\" or color=[255,0,0].\nwireframe: = false, if solid, otherwise wireframe representation.\ntransparency: = 0.0 (opaque) ... 1.0 (fully transparent).\nreflectslight: = true if it reflects light and false, if it does not reflect light.\nshininess: = 0.0 (matte surface) ... 1.0 (very shiny surface).\nshadowMask: defines whether or not an object casts or receives shadows. Possible values:  NoShadows, CastsShadows, ReceivesShadows, CastsAndReceivesShadows.\n\n\n\n"
},

{
    "location": "lib/Graphics.html#Modia3D.Graphics.TextShape",
    "page": "Graphics",
    "title": "Modia3D.Graphics.TextShape",
    "category": "type",
    "text": "textShape = Modia3D.TextShape(text; font=Modia3D.Font(), offset=[0.0,0.0,0.0], \n                      axisAlignment=Modia3D.Screen, alignment=Modia3D.Center)\n\nReturn a text shape.\n\nArguments\n\ntext::AbstractString: String of the text.\nfont::Modia3D.Font: Font of the text.\noffset::AbstractVector: Offset from origin to text alignment point.\naxisAlignment::Modia3D.AxisAlignment: Alignment of Text (parallel to screen or in  planes of frame: = Modia3D.Screen, Modia3D.XY_Plane, Modia3D.XZ_Plane, Modia3D.YZ_Plane).\naxisAlignment::Modia3D.Alignment: Alignment of Text relative to its origin (= Modia3D.Left, Modia3D.Right or Modia3D.Center).\n\nExamples\n\nimport Modia3D\nfont  = Modia3D.Font(fontFamily=\"Arial\", charSize=0.4, color=Modia3D.rgb(\"Red\"))\ntext1 = Modia3D.TextShape(\"This is a box\")\ntext2 = Modia3D.TextShape(\"This is the xy plane\";\n                          font=font, axisAlignment=Modia3D.XY_Plane, \n                          alignment=Modia3D.Left)\n\n\n\n"
},

{
    "location": "lib/Graphics.html#Modia3D.Graphics.rgb-Tuple{String}",
    "page": "Graphics",
    "title": "Modia3D.Graphics.rgb",
    "category": "method",
    "text": "color = rgb([name::String | vec::AbstractVector | r::Number,g::Number,b::Number])\n\ndefines the color as a 3-vector of RGB values. Currently, the following names of colors are defined: Black, DarkRed, Red, LightRed, DarkGreen, Green, LightGreen, DarkBlue, Blue, LightBlue, Yello, Pink DarkGrey, Grey, White.\n\nExamples\n\ncolor1 = Modia3D.rgb(\"Red\")       # = [255,0,0]\ncolor2 = Modia3D.rgb([255,0,0])   # = [255,0,0]\ncolor3 = Modia3D.rgb(255,0,0)     # = [255,0,0]\n\n\n\n"
},

{
    "location": "lib/Graphics.html#Graphics-1",
    "page": "Graphics",
    "title": "Graphics",
    "category": "section",
    "text": "Modules = [Modia3D.Graphics]\r\nPrivate = false"
},

{
    "location": "lib/Graphics.html#-1",
    "page": "Graphics",
    "title": "",
    "category": "section",
    "text": ""
},

{
    "location": "lib/Solids.html#",
    "page": "Solids",
    "title": "Solids",
    "category": "page",
    "text": ""
},

{
    "location": "lib/Solids.html#Modia3D.Solids",
    "page": "Solids",
    "title": "Modia3D.Solids",
    "category": "module",
    "text": "module Modia3D.Solids\n\nObjects that have a volume and properties associated with the volume. Solid parts can be associated with a Modia3D.Object3D.  They are defined with struct Modia3D.Solid consisting of an optional solid geometry:\n\n(Image: Solids)\n\nand other optional properties:\n\nmass propreties (defined by geometry+material-name, geometry+density, or directly defined mass properties),\ncontact material (for elastic response calculation),\nvisualization material (for visualization, see below).\n\nSince the solid geometry itself is optional, it is possible to just define a coordinate system with associated mass, center of mass and inertia matrix.\n\nThe following functions are provided for a solid geometry geo that is associated with an Object3D object3D:\n\nvolume(geo),\ncentroid(geo),\ninertiaMatrix(geo, mass),\nboundingBox(geo, <other arguments>),\nsupportPoint(geo, <other arguments>),\nisVisible(object3D, renderer),\nhasMass(object3D),\ncanCollide(object3D),\nand other functions.\n\nMain developers\n\nAndrea Neumayr and Martin Otter, DLR - Institute of System Dynamics and Control\n\n\n\n"
},

{
    "location": "lib/Solids.html#Modia3D.Solids.Solid",
    "page": "Solids",
    "title": "Modia3D.Solids.Solid",
    "category": "type",
    "text": "solid = Modia3D.Solid([geo | nothing],\n                      [solidMaterialName | solidMaterial | mass |\n                       massProperties    | nothing] = nothing,\n                      [material = Modia3D.Material() | nothing];\n                      contactMaterial=nothing)\n\nGenerate a new (rigid) solid with optional solid geometry, mass, visualization and collision properties A solid can be associated to a Modia3D.Object3D.\n\nArguments\n\ngeo::Union{Modia3D.AbstractSolidGeometry,Void}: Optional solid geometry object (such as Modia3D.SolidSphere,.SolidBox,.SolidFileMesh).\nMass properties (mass, center of mass, inertia matrix) of geo are computed by one of:\nsolidMaterialName::AbstractString: Name of a solid material defined in dictionary Modia3D.solidMaterialPalette (computed by geo and density of solid material)\nsolidMaterial::Modia3D.SolidMaterial: Solid material properties object (computed by geo and solidMaterial.density)\nmass::Number: Mass in kg (computed by geo and mass).\nmassProperties::Modia3D.MassProperties: Mass properties (mass, center of mass, inertia matrix) are explicitly given.\nnothing: geo has no mass.\nmaterial::Union{Modia3D.Material,Void}: Visualization material of geo.  If material=nothing, geo is not shown in the visualization.\ncontactMaterial::Union{Modia3D.AbstractContactMaterial,Void}: Contact material of geo.  If contactMaterial=nothing, no collision handling takes place for geo.\n\n(Image: Solids)\n\nExamples\n\nimport Modia3D\n\nsbox  = Modia3D.SolidBox(1.0,2.0,3.0)\nsmat  = Modia3D.SolidMaterial(density = 2700)\nvmat  = Modia3D.Material(color=\"Blue\", transparency=0.5)\ncmat  = Modia3D.ContactMaterialElastic(c=1e5, d=100)\nmassProperties = Modia3D.MassProperties(m=0.1, Ixx=1.0, Iyy=2.0, Izz=3.0)\n\nsolid1 = Modia3D.Solid(sbox, \"Aluminium\", vmat)\nsolid2 = Modia3D.Solid(sbox, smat       , vmat)\nsolid3 = Modia3D.Solid(sbox, 2.1        , vmat )\nsolid4 = Modia3D.Solid(sbox, nothing; contactMaterial=cmat)\nsolid5 = Modia3D.Solid(Modia3D.SolidSphere(0.1), massProperties, vmat; contactMaterial=cmat)\nsolid6 = Modia3D.Solid(nothing, massProperties)\n\n\n\n"
},

{
    "location": "lib/Solids.html#Solids-1",
    "page": "Solids",
    "title": "Solids",
    "category": "section",
    "text": "Modules = [Modia3D.Solids]\r\nPrivate = false"
},

{
    "location": "lib/ForceElements.html#",
    "page": "ForceElements",
    "title": "ForceElements",
    "category": "page",
    "text": ""
},

{
    "location": "lib/ForceElements.html#Modia3D.ForceElements",
    "page": "ForceElements",
    "title": "Modia3D.ForceElements",
    "category": "module",
    "text": "module Modia3D.ForceElements\n\nElements generating forces or torques that act on objects.\n\nMain developers\n\nAndrea Neumayr and Martin Otter, DLR - Institute of System Dynamics and Control\n\n\n\n"
},

{
    "location": "lib/ForceElements.html#Modia3D.ForceElements.@forceElement-Tuple{Any,Any}",
    "page": "ForceElements",
    "title": "Modia3D.ForceElements.@forceElement",
    "category": "macro",
    "text": "@forceElement forceName(arguments) begin ... end - Generate a function that instantiates a new forceElement\n\nforceName(arguments) begin ... end is treated as the constructor function of a new (mutable) struct that consists of all left-hand-side (scalar or vector) symbols present between begin ... end.\n\nExamples\n\njulia> include(\"D:\\otter\\_github\\Modia3D.jl/examples/dynamics/Simulate_PendulumWithDamper.jl\");\n\nusing ModiaMath\nusing Modia3D\n\n@forceElement Damper(; d=1.0) begin\n    w   = ModiaMath.RealScalar(\"w\",   causality=ModiaMath.Input,  numericType=ModiaMath.WR)\n    tau = ModiaMath.RealScalar(\"tau\", causality=ModiaMath.Output, numericType=ModiaMath.WR)\nend;\nfunction Modia3D.computeTorque(damper::Damper, sim::ModiaMath.SimulationState)\n    damper.tau.value = -damper.d*damper.w.value\nend;\n@assembly PendulumWithDamper(;Lx = 1.0, Ly=0.2*Lx, Lz=0.2*Lx, m=1.0, g=9.81) begin\n   world  = Modia3D.Object3D(Modia3D.CoordinateSystem(0.5*Lx))\n   body   = Modia3D.Object3D(Modia3D.Solid(Modia3D.SolidBeam(Lx,Ly,Lz), Modia3D.MassProperties(m=m), vmat1))\n   frame1 = Modia3D.Object3D(body; r=[-Lx/2, 0.0, 0.0])\n   cyl    = Modia3D.Object3D(frame1,Modia3D.Cylinder(Ly/2,1.2*Ly; material=vmat2))\n\n   # Connect pendulum to world with a damper in the joint\n   rev    = Modia3D.Revolute(world, frame1)\n   d      = Damper(d=0.2)\n   damper = Modia3D.AdaptorForceElementToFlange(w=d.w, tau=d.tau)\n   Modia3D.connect(damper, rev)\nend\npendulum = PendulumWithDamper()\nmodel = Modia3D.SimulationModel( pendulum )\nresult = ModiaMath.simulate!(model, stopTime=5.0, interval=0.1, tolerance=1e-4, log=true)\nModiaMath.plot(result, [\"rev.phi\", \"rev.w\", \"rev.a\", \"rev.tau\"])\n\n\n\n"
},

{
    "location": "lib/ForceElements.html#ForceElements-1",
    "page": "ForceElements",
    "title": "ForceElements",
    "category": "section",
    "text": "Modules = [Modia3D.ForceElements]\r\nPrivate = false"
},

{
    "location": "lib/ForceElements.html#-1",
    "page": "ForceElements",
    "title": "",
    "category": "section",
    "text": ""
},

{
    "location": "lib/Basics.html#",
    "page": "Basics",
    "title": "Basics",
    "category": "page",
    "text": ""
},

{
    "location": "lib/Basics.html#Modia3D.Basics",
    "page": "Basics",
    "title": "Modia3D.Basics",
    "category": "module",
    "text": "module Modia3D.Basics\n\nUtility constants and functions for Modia3D\n\nMain developers\n\nAndrea Neumayr and Martin Otter, DLR - Institute of System Dynamics and Control\n\n\n\n"
},

{
    "location": "lib/Basics.html#Modia3D.Basics.BoundingBox",
    "page": "Basics",
    "title": "Modia3D.Basics.BoundingBox",
    "category": "type",
    "text": "mutable struct BoundingBox - Smallest box that contains a visual element\n\n\n\n"
},

{
    "location": "lib/Basics.html#Modia3D.Basics.getAndCheckFullLibraryPath-Tuple{Any,Any}",
    "page": "Basics",
    "title": "Modia3D.Basics.getAndCheckFullLibraryPath",
    "category": "method",
    "text": "getAndCheckFullLibraryPath(dir,libname)\n\nReturn joinpath(dir,libname). The returned full path must be a DLL. It is checked whether this DLL can be opened\n\n\n\n"
},

{
    "location": "lib/Basics.html#Modia3D.Basics.getEnvironmentVariable-Tuple{String,String}",
    "page": "Basics",
    "title": "Modia3D.Basics.getEnvironmentVariable",
    "category": "method",
    "text": "value = getEnvironmentVariable(name, description)\n\nReturns the value of the environment variable name.  If name does not exist, an error message is triggered:\n\nEnvironment variable <name> not defined\n(= <description>)\n\n\n\n"
},

{
    "location": "lib/Basics.html#Basics-1",
    "page": "Basics",
    "title": "Basics",
    "category": "section",
    "text": "Modules = [Modia3D.Basics]\r\nPrivate = false"
},

]}
