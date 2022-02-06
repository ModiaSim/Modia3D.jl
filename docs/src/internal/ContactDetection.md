# Contact Detection

This section is based on [^1] with some minor improvements.

The following goals shall be achieved:

1. Whenever two objects start to penetrate, an event shall be triggered in order that integration methods
   with step-size control are simulating efficiencly (at the start of the contact, contact forces and torques
   with non-zero values are typically introduced leading to strong discontinuous changes of the equations).
2. When two objects start to penetrate, the normal contact velocity ``\dot{\delta}^-`` needs to be stored,
   because used in the [contact force law](ContactForceLaw.md).
3. Whenever two penetrating objects are not penetrating any more, again an event shall be triggered.

Modia3D uses [DifferentialEquations.jl](https://github.com/SciML/DifferentialEquations.jl) integrators
to solve the equations. These integrators trigger an event when a **zero crossing function** crosses zero.
Hereby, the option `DifferentialEquations.SciMLBase.RightRootFind` is used, that is, an event is triggered
when the zero crossing function is either identical to zero or has an opposite sign.
At an event restart it is required that the **zero crossing functions are not identical to zero**, since
otherwise a zero crossing cannot be detected.


## Zero crossing functions

The distance between two objects computed either in the narrow phase or in an approximate
way in the broad phase (= distance between Axis Aligned Bounding Boxes) is called `distanceOrg`.
If `distanceOrg < 0`, then the two objects are penetrating each other.

Modia3D uses the dictionary `contactDict` to keep track of the
contact situation. Every pair of objects is identified by a unique
Integer value called *PairID* that is used as key in `conctactDict`.
A dictionary value is an instance of [`Modia3D.ContactPair`](@ref)

At an **event instant** (including initialization), dictionary `contactDict` is emptied
and all object pairs are stored newly in `contactDict` that have `distanceOrg <= 0`, so are
either touching or penetrating.

During **continuous integration**, two zero crossing functions are used:

* ``z_1(t)``: The maximum of `distanceOrg` ``- 10^{-16}`` of all object pairs that are in `contactDict`.
  ``z_1`` monitors if an object pair that has been in contact, looses contact.
  Since `distanceOrg` values in `contactDict` are in the order of ``10^{-3} .. 10^{-6} ~m``
  and `eps(Float64)` ``\approx 10^{-16}``, a hysteresis epsilon must be larger as ``10^{-19} .. 10^{-24}``.
  Actually, a hysteresis epsilon of ``10^{-16}`` is used. Note, that in `contactDict` all `distanceOrg` values
  are zero or negative at event restart. It is therefore guaranteed that ``z_1 \le - 10^{-16}`` at event restart
  (when no contact pair is present, a dummy value is used).

* ``z_2(t)``: The minimum `distanceOrg` of all object pairs that are **not** in `contactDict`.
  ``z_2`` monitors if two objects that have not been in contact to each other and start to penetrate each other.
  At an event restart (including the start of the integration after the initialization), it is guaranteed
  that all `distanceOrg` not in `contactDict` are positive (otherwise, they would be
  in `contactDict`). Therefore, it is guaranteed that ``z_2 > 0`` at event restart (without a hysteresis epsilon).

The distance between two objects is only computed up to a certain precision. Therefore, `distanceOrg` might be
nearly zero but still positive, if two objects are placed initially in touching position.
For this reason the actually used distance is `distanceOrg - 1e-16`,
in order that the probability is larger that objects initially in touching position
are treated to be in contact initially.

To summarize, the following equations are actually used:

```
distance = distanceOrg - 1e-16
contact  = isEvent ? distance <= 0 : <contact from previous event>
z[1]     = max(<distance that has contact=true>) - 1e-16
z[2]     = min(<distance that has contact=false>)
```

Typically, the `simulate!(..., log=true, ...)` produces the following output (here for a sphere boucing on ground):

```
 State event (zero-crossing) at time = 2.3924142776549155 s (due to z[2])
   distance(ground,sphere) = -3.815173674537923e-18 became <= 0
       contact normal = [0.0, 1.0, 0.0], contact position = [0.0, -0.1, 0.0], c_res = 1.1e11 d_res = 47.5
   restart = Restart

State event (zero-crossing) at time = 2.3924413895015424 s (due to z[1])
   distance(ground,sphere) = 1.0362163563391329e-16 became > 0
   restart = Restart
```

Whenever the integrator requires the values of the zero crossing functions, the values of
``z_1, z_2`` are newly computed. At all other time instants (e.g. communication points),
only the distances of the object pairs are computed that are in dictionary `contactDict`,
because these distances are used to compute the contact forces and torques. Distances
of object pairs that are not in contact to each other are not computed at these time instants.

Whenever contact starts newly for an object pair, the normal contact velocity ``\dot{\delta}^-``
needs to be stored, since needed from the [elastic response calculation](ContactForceLaw.md).
When a contact pair was already in contact at an event, it is necessary to utilize ``\dot{\delta}^-``
from the previous event also at the actual event. In order that these two requirements are fulfilled,
a second dictionary `lastContactDict` is used: Once `contactDict` is constructed
at an event, a copy is stored in `lastContactDict`. At the next event, a new `contactDict`
is constructed. Whenever an object pair is included in `contactDict` that is also present in
`lastContactDict`, then the normal contact velocity ``\dot{\delta}^-`` stored in `lastContactDict` is utilized.

Note, in `scene.options.contactDetection`
(file `Modia3D/src/contactDetection/ContactDectionMPR_handler.jl`)
information about the contact situation is stored in an instance
of [`Modia3D.ContactDetectionMPR_handler`](@ref).

## Collision handling types

```@docs
Modia3D.ContactPair
```

```@docs
Modia3D.ContactDetectionMPR_handler
```

## Literature

[^1]: Andrea Neumayr, Martin Otter (2019):
      [Collision Handling with Elastic Response Calculation and Zero-Crossing Functions](https://doi.org/10.1145/3365984.3365986).
      Proceedings of the 9th International Workshop on Equation-Based Object-Oriented Modeling Languages and Tools. EOOLT’19. ACM, pp. 57–65.
