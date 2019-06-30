# Contact Detection


## Distance

In `scene.options.contactDetection`
(file `Modia3D/src/contactDetection/ContactDectionMPR_handler.jl`)
information about the contact situation is stored in an instance
of [`Modia3D.Composition.ContactDetectionMPR_handler`](@ref).
Hereby information about at most `nz_max` objects pairs are stored that
are in contact or are close to each other.

The *distance* between two objects is defined by two variables:

**distanceOrg**

The original distance which is the distance between   
two convex objects computed either by the improved MPR algorithm for
the narrow phase, or the distance between Axis Aligned Bounding Boxes
of two objects in the broad phase.
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
able to detect a zero crossing in the future). When `distanceWithHysteresis`
crosses from positive to negative and at initialization, the following flag
is set (`zEps = 1e-8`):

```
contact = distanceOrg < -zEps
```

This flag is kept unchanged until another event occurs where
`distanceWithHysteresis` crosses from negative to positive.
At this event, `contact = false` is set.

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

- During simulation, a *contact appears* (and therefore `contact` becomes
  `true`) when `distanceOrg` becomes smaller as `-2*zEps`.

- During simulation, a *contact is released* (and therefore `contact`
  becomes `false`) when  `distanceOrg` becomes larger as zero.

By this formulation it is guaranteed that `distanceWithHysteresis` is not
identical to zero when starting after initialization or restarting after an
event. Furthermore, during contact:

```
distanceWithHysteresis = distanceOrg     # if contact = true
```


## ID and key of object pair

An *object pair* is identified by a *unique* `pairID::Int64`. A pairID
is constructed by packing the object indices of the two objects into an `Int64` number.

The object pairs with the *smallest distances* are stored in the sorted
dictionary [noContactDict](@ref).
The entries in this dictionary are ordered according to their `key::PairKey`.
This key consists of two elements that are evaluated
in the following order:

2. `distanceWithHysteresis`
3. `pairID`

This leads to the following definition of the `key` ordering:

```
Base.:isless(key1::PairKey, key2::PairKey) =
   key1.distanceWithHysteresis != key2.distanceWithHysteresis ?
       key1.distanceWithHysteresis < key2.distanceWithHysteresis :
       key1.pairID                 < key2.pairID
```


## Dictionaries

The following dictionaries are utilized in Modia3D.


### lastContactDict

This dictionary is defined as `lastContactDict::Dict{Int64,MaterialContactPair}`

A *key* of this dictionary is a `PairID` of an object pair
that had **`contact=true`** at the *last* event instant.

A *value* is an instance of the mutable struct
[`Modia3D.Composition.MaterialContactPair`](@ref) which contains
the material constants of the two objects in contact
and the derivative of the distance
(= relative velocity projected on contact normal)
when the last event occured.

At every *event instant*, after dictionary `contactDict`
was constructed, dictionary `lastContactDict` is
emptied and then filled with the derivative of the distance
and the material constants of the elements stored in
dictionary `contactDict`.

Before the next event, dictionary `lastContactDict` is not changed.

This dictionary is used to copy the distance derivative and the
material constants from object pairs that had been in contact at the last
event, to dictionary `contactDict` at the current event,
in case these object pairs are still in contact at the current event.


### contactDict

This dictionary is defined as `contactDict::Dict{Int64,ContactPair}`.

A *key* of this dictionary is a `PairID` of an object pair
that has **`contact=true`**.

A *value* is an instance of the mutable struct
[`Modia3D.Composition.ContactPair`](@ref) which contains
all information needed for the collision handling of this contact pair,
especially to compute the response calculation at the actual time instant.

At every *event instant* dictionary `contactDict` is emptied and
then filled with `PairID` contact pairs that have `contact=true`.
An error occurs if `length(contactDict)` becomes larger as `nz_max`.

Before restarting the integration, the derivative of the distance
(= relative velocity projected on contact normal) together with the
contact materials is stored in `ContactPair`. If `PairID` is
also a key in `lastContactPair`, then this data is copied from
dictionary `lastContactPair`. Otherwise, this information is newly
constructed because the object pairs are newly in contact.

At *any other model evaluation*, the keys of `contactDict`
are not changed and only the values are updated:

1. It is inquired whether the object pair with `pairID` is an existing key
   in dictionary `contactDict`.

2. If this is the case, then
   `contactDict[pairID]` is updated with the actual
   contact situation, especially with `distanceWithHysteresis`.

The zero crossing functions reported to the integrator are stored in
vector `z::Vector{Float64}`. Hereby, the first `nzContact` entries
in `z` are the `contactDict[pairID].distanceWithHysteresis` values
of the elements of this dictionary.



### noContactDict

This dictionary is defined as `noContactDict::SortedDict{PairKey,Float64}`.

A *key* of this dictionary is a `PairKey` of an object pair
that has **`contact=false`**. The dictionary is sorted so that
`pairKey1` is before `pairKey2`, if `pairKey1 < pairKey2`.

A *value* is the `distanceWithHysteresis` of the object pair.

At every *event instant* and
whenever the *integrator requires zero crossing functions*,
dictionary `noContactDict` is emptied and
then filled with `pairKey` object pairs that have `contact=false`.
This is performed in the following way:

- Given a `pairID`, it is inquired whether the object pair is
  part of dictionary `contactDict`; if this is not the case,
  the `pairKey` is constructed and `pairKey => distanceWithHysteresis`
  is inserted in `noContactDict`.

- If `length(ContactDict) + length(NoContactDict) = nz_max`,
  it is inquired whether the last element of `noContactDict` has a
  `distanceWithHysteresis` that is larger as the
  `distanceWithHysteresis` of the object pair P that shall be
  inserted. If this is the case, the last element is deleted
  and P is inserted in `noContactDict`.
  If this is not the case, P is ignored.

At *any other model evaluation*, this dictionary is not changed.
(because it is not used).

The zero crossing functions reported to the integrator are stored in
vector `z::Vector{Float64}`. Hereby, the `nzContact+1..end` entries
in `z` are the `noContactDict[pairKey] values.


### visualizeDict

This dictionary is defined as `visualizeDict::Dict{Int64,PairVisualization}`.
It is only constructed if the collision situation
for the object pairs shall be visualized
(`visualizeContactPoints=true` and/or
`visualizeSupportPoints=true` in `sceneOptions`).

A *key* of this dictionary is a `PairID` of an object pair.

A *value* is an instance of the mutable struct
[`Modia3D.Composition.PairVisualization`](@ref) which contains
all information needed to visualize the collision situation of
the object pairs.

At every *event instant* and whenever the
*integrator requires zero crossing functions*,
this dictionary is emptied and then newly filled, in synchronization
to the dictionaries `contactDict`and `noContactDict`.
At any other *model evaluation*, the keys of this dictionary
are not changed and only the values are updated.
