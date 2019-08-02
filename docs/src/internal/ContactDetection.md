# Contact Detection


## Distances

In `scene.options.contactDetection`
(file `Modia3D/src/contactDetection/ContactDectionMPR_handler.jl`)
information about the contact situation is stored in an instance
of [`Modia3D.Composition.ContactDetectionMPR_handler`](@ref).

The *distance* between two objects is defined by two variables:

**distanceOrg**

The original distance which is the distance between
two objects computed either in the narrow phase or in an approximate
way in the broad phase (= distance between Axis Aligned Bounding Boxes).
If `distanceOrg < 0`, then the two objects are penetrating each other.


**distanceWithHysteresis**

The distance reported to the integrator as
zero crossing function. If a zero crossing occurs (the value is changing
from positive to negative or from negative to positive), an *event* is
triggered and the time instant of the zero crossing is detected up to a certain
precision. `distanceWithHysteresis` is a slightly modified value
of `distanceOrg` in order to cope with numerical inaccuracies and in order to guarantee
that this value is not identical to zero when restarting from the event
(otherwise the integrator would trigger an error, because it would not be
able to detect a zero crossing in the future). At an *event instant*
(including initialization), the following flag is set and keeps its value until
the next event (`zEps = 1e-8`):

```
contact = distanceOrg < -zEps
```

Whenever a zero crossing function computation is required by the integrator,
the following value is provided to the integrator:

```
distanceWithHysteresis = contact ? distanceOrg : distanceOrg + 2*zEps
```

The result is the following:

- At initialization, `contact = false` if `distanceOrg >= -zEps`.
  Therefore, if two objects are touching each other or are slightly penetrating
  each other (e.g. `distanceOrg = -1e-14`), then this is not treated as contacting.
  Otherwise, `contact = true`.

- During simulation, an event is triggered if
  * `distanceOrg` becomes smaller as `-2*zEps`, if `contact = false`.
  * `distanceOrg` becomes larger as zero if `contact = true`.

By this formulation it is guaranteed that `distanceWithHysteresis` is not
identical to zero when starting after initialization or restarting after an
event.


## Crossing functions

Modia3D uses the dictionary `contactDict` to keep track of the
contact situation. Every pair of objects is identified by a unique
Integer value called *PairID* that is used as key in `conctactDict`.
A dictionary value is an instance of [`Modia3D.Composition.ContactPair`](@ref)

At an event instant, dictionary `contactDict` is emptied and all object pairs are stored
newly in `contactDict` that have `distanceWithHysteresis < 0`.
Between events, contact forces/torques are only
applied on object pairs that are in this dictionary.

The following functions are used as zero crossing functions:

* ``z_1(t)``: The maximum `distanceWithHysteresis` of all object pairs that are in `contactDict`.
* ``z_2(t)``: The minimum `distanceWithHysteresis` of all object pairs that are **not** in `contactDict`.

As a result ``z_1`` monitors if an object pair that has been in contact, looses contact, so
`distanceWithHysteresis` becomes greater than zero,
and ``z_2`` monitors if an object pair that has not been in contact, starts to penetrate each other, so
`distanceWithHysteresis` becomes less than zero,

Whenever the integrator requires the values of the zero crossing functions, the values of
``z_1, z_2`` are newly computed. At all other time instants (e.g. communication points),
only the distances of the object pairs are computed that are in dictionary `contactDict`,
because these distances are used to compute the contact forces and torques. Distances
of object pairs that are not in contact to each other are not computed at these time instant.

Whenever contact starts for an object pair the normal contact velocity is stored in the value
(of type [`Modia3D.Composition.ContactPair`](@ref)) of the dictionary.
When a contact pair was already in contact at an event, it is necessary to utilize the
normal contact velocity from the previous event also at the actual event. In order that this
is possible, a second dictionary `lastContactDict` is used: Once `contactDict` is constructed
at an event, a copy is stored in `lastContactDict`. At the next event, a new `contactDict`
is constructed. Whenever an object pair is included in `contactDict` that is also present in
`lastContactDict`, then the normal contact velocity stored in `lastContactDict` is utilized.
