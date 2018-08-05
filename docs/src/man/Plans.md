# Plans for version 1.0

Modia3D is not yet ready and should not be used for production work.
The features below are planned to be implemented before Modia3D version 1.0.
We are very interested that other people help and contribute to Modia3D. If you
would like to implement one of the features below (or any other features),
please contact [Martin.Otter@dlr.de](mailto:Martin.Otter@dlr.de).

Features planned for version 1.0:

- **General**
  - Improving efficiency: Debugging Modia3D with respect to unnecessary heap allocations
    during integration (especially, changing some of the remaining MVector{3,Float64} to
    SVector{3,Float64}).
  - Improved documentation.
  - Changing to Julia 0.7.

- **ContactDetectionMPR**
  - The MPR algorithm for distance computation needs to be more thoroughly tested.
  - Add an option to getDistances(ch) to only compute the distances of contact pairs that
    had a negative distance in the last call of selectContactPairs(ch)
    (to enhance efficiency of the simulation with a variable-step integrator).

- **Solids**
  - Making contact handling more robust and more thoroughly testing the contact response calculation.
  - Definition of meshes directly in Julia (not only via *.obj files).
  - Map all non-triangles of *.obj files to triangles, in order that further operations
    (like volume computation) can operate on them.


- **Renderer**
  - Interfacing the remaining features of the DLR Visualization library to Modia3D/Julia
    (cameras and paths, flexible surface, elastically deformed mesh,
    fluid flow visualization, head.up displays, effects, large terrains, ...).

- **Composition**
  - Visualizing (optionally) the direction of the gravity vector if a parallel gravity field is used.
  - Improving efficiency: Structure spanning tree in two parts: One only needed for
    kinematic/dynamic analysis and the remaining part only needed for visualization
    (need only to be evaluated at communication points).
  - Support of more joint types (Spherical, Cylindrical, Planar, GearConstraint, ...).
  - Support of 3D force/torque objects.
  - Automatic handling of 2D and 3D kinematic loops for kinematic and dynamic analysis
    (currently only 2D kinematic loops are supported for kinematic analysis).
  - Support of analytic loop handling (along the line of the
    [Assembly joints](https://doc.modelica.org/help/Modelica_Mechanics_MultiBody_Joints_Assemblies.html#Modelica.Mechanics.MultiBody.Joints.Assemblies)
    of the Modelica Standard Library, but fully automatic).
  - Using Modia components with input/output ports as signal generators or as force elements in Modia3D.
  - Using a Modia3D model as component in Modia.
  - Optionally supporting an O(n) algorithm for the spanning tree.
  - For models that can be completely mapped to an ODE (= tree-structure with O(n) algorithm
    and all kinematic loops can be solved with the analytic loop handling), optionally an ODE
    integrator of [DifferentialEquations.jl](https://github.com/JuliaDiffEq/DifferentialEquations.jl)
    can be used for simulation.
  - Support for 1-dimensional heat transfer data attached to an Object3D
    (to make sure that further extensions of the concept in non-mechanical domains are possible).



## Plans after version 1.0

The following features are planed after version 1.0 (especially together with interested collaborators):

- **Solids**
  - Support of operations on meshes with triangles to construct new meshes (extrusion, CSG)
    along the paper [Generic Modelica Framework for MultiBody Contacts and Discrete Element Method](http://www.ep.liu.se/ecp/118/046/ecp15118427.pdf).

- **GUI**
  - Support of an open source, web-browser-based GUI to graphically define 3D assemblies and use the GUI also as
    default renderer (along the paper [3D Schematics of Modelica Models and Gamification](http://www.ep.liu.se/ecp/118/057/ecp15118527.pdf); 
    probably using [three.js](https://threejs.org) as underlying web-browser, webgml based rendering engine).

- **Composition**
  - Support for hard contacts with impulses.
  - Attaching other domains to Object3Ds (heat transfer, fluid flow, ...).
