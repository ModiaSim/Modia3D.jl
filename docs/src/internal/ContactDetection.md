# Contact Detection


## Distances

In `scene.options.contactDetection`
(file `Modia3D/src/contactDetection/ContactDectionMPR_handler.jl`)
information about the contact situation is stored in an instance
of [`Modia3D.Composition.ContactDetectionMPR_handler`](@ref).
Hereby information about at most `nz_max` objects pairs are stored that
are in contact or are close to each other.

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


## Keys

An *object pair* is identified by a *unique* `pairID::PairID`, where `const PairID = Int64`.
A pairID is constructed by packing the object indices of the two objects into an `Int64` number.

The information about the *smallest distances* are stored in the sorted
dictionary `distanceSet` (see below).
The entries in this dictionary are ordered according to their `key::DistanceKey`.
This key consists of three elements that are evaluated in the following order:

1. `contact`
2. `distanceWithHysteresis`
3. `pairID`

If `contact=true`, order does not matter and is therefore only based on the unique `pairID`. If `contact=false`, order matters and is basically on `distanceWithHysteresis`. This leads to the following definition of the `DistanceKey` ordering:

```
Base.:isless(key1::DistanceKey, key2::DistanceKey) =
     key1.contact &&  key2.contact ? key1.pairID < key2.pairID :
   ( key1.contact && !key2.contact ? true                        :
   (!key1.contact &&  key2.contact ? false                       :
   (key1.distanceWithHysteresis != key2.distanceWithHysteresis   ?
    key1.distanceWithHysteresis < key2.distanceWithHysteresis    :
    key1.pairID                < key2.pairID)))
```


## Dictionaries

The following dictionaries are utilized for contact detection:

| Dictionary/Set   | key type    | value type    | contact    | step     | else   |
|:---------------- |:------------|:--------------|:----------:|:--------:|:------:|
|`lastContactDict` | PairID      | ContactPair   | true       |   -      |   -    |
|`contactDict`     | PairID      | ContactPair   | true       | update   | update |
|`noContactDict`   | PairID      | NoContactPair | false      | new keys | update |
|`distanceSet`     | DistanceKey | ---           | true/false | new keys |   -    |

- `lastContactDict` and `contactDict` are standard `Dict`s that have
   [`Modia3D.Composition.ContactPair`](@ref) as value type.

- `noContactDict` is a standard `Dict` that has
   [`Modia3D.Composition.NoContactPair`](@ref) as value type.
   The difference to `ContactPair` is that the contact points and the contact normal
   might be `Nothing` (whereas concrete vectors are required by `ContactPair`),
   because in the broad phase this information is not determined.

- `distanceSet` is a `SortedSet` where the keys are basically sorted according to
  `distanceWithHysteresis` (see section [Keys](@ref)).

- Column *contact* defines the value of the corresponding variable
  (`contact = true` if object pairs have been in contact at the last event instant).

- At an *event* instant, all dictionaries have action *new keys* (see below).

- Column *step* contains the actions after a *completed integrator step*.
  (Currently, this situation is not flagged by ModiaMath. Therefore the actions of column
  *step* are executed when the *integrator requires zero crossing functions*.
  This gives a slight degradation of simulation efficiency).

- Column *else* contains the actions at other model evaluations (no event and no completed step).

- *new keys* means that the dictionary is emptied and is filled with *new (key,value)* pairs.

- *update* means that the keys are not changed and the *values are updated*.



**Actions at an event instant**

1. `contactDict, noContactDict, distanceSet` are emptied.

2. For every object pair P:

   - distanceKey(P) is inserted in `distanceSet`.
     If `length(distanceSet) = nz_max`, it is inquired whether the last item of `distanceSet` has a
     `distanceWithHysteresis` that is larger as the
     `distanceWithHysteresis` of P. If this is the case, the last item
     is deleted and `distanceWithHysteresis` is inserted in `distanceSet`
     (it is an error, if there are more than nz\_max object pairs with `contact=true`).
     If this is not the case, P is ignored.

   - If an item was inserted in `distanceSet`, the corresponding collision information
     is either inserted in `ContactDict` (if `contact=true`) or
     `noContactDict` (if `contact=false`).
     If an item was deleted in `distanceSet`, the corresponding item is deleted
     in `noContactDict` as well (deletion cannot occur for items in `contactDict`).

3. After all object pairs have been visited:

   - All (key,value) pairs in `contactDict` are inspected: If a key is also
     in `lastContactDict`, the contact material of `lastContactDict` is used
     in `contactDict`. If this is not the case, the contact material is newly
     determined and stored in `contactDict` (e.g. the derivative
     of the distance needs to be determined and used to compute
     the contact material constants)

   - `lastContactDict` is emptied and all items of
     `contactDict` are included in `lastContactDict`.


**Actions after a completed integrator step**

1. `noContactDict, distanceSet` are emptied.

2. For every object pair P:

   - If pairID(P) is a key of `contactDict`, then the corresponding value
     is updated with the actual contact situation, especially with
     `distanceWithHysteresis`.

   - If pairID(P) is not a key of `contactDict`, then
     distanceKey(P) is inserted in `distanceSet`.
     If `length(contactDict)+length(distanceSet) = nz_max`, it is inquired whether the last item of `distanceSet` has a
     `distanceWithHysteresis` that is larger as the
     `distanceWithHysteresis` of P. If this is the case, the last item
     is deleted and `distanceWithHysteresis` is inserted in `distanceSet`.
     If this is not the case, P is ignored.

   - If an item was inserted in `distanceSet`, the corresponding
     `NoContactPair` is inserted in `noContactDict`.
     If an item was deleted in `distanceSet`, the corresponding item is deleted
     in `noContactDict` as well.


**Actions at other model evaluations (no event and no completed integrator step)**

For every object Pair P:

- If pairID(P) is a key of `contactDict`, then the corresponding value
  is updated with the actual contact situation, especially with
  `distanceWithHysteresis`.

- If pairID(P) is a key of `noContactDict`, then the corresponding value
  is updated with the actual contact situation, especially with
  `distanceWithHysteresis`.


The zero crossing functions are reported to the integrator with vector `z::Vector{Float64}`
of length `nz_max`. Hereby, the first entries
in `z` are the `contactDict[pairID].distanceWithHysteresis` values
and the remaining ones are the `noContactDict[pairID].distanceWithHysteresis` values.
Vector `z` is updated, whenever the integrator requires zero-crossing functions.

Strictly speaking, dictionary `noContactDict` is not absolutely necessary,
because the values are not needed for a standard simulation.
However, if additional functionality will be added later,
such as storing the closest distances in the result data structure,
or when printing a warning when there is the danger that two objects
are too close together, then this data is needed at every model evaluation.
Furthermore, when visualizing the contact situation, this data is also needed
(but then this dictionary could be constructed, when the corresponding
options are set, and not otherwise).
