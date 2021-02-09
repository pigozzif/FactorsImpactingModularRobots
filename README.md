# VSRBiodiversity
This is the official repository for the GECCO (Genetic and Evolutionary Computation Conference-2021) paper "Biodiversity in Evolved Voxel-based Soft Robots". This work is partially based on Marco Rochelli's master's thesis (https://github.com/MarcoRochelli/Co-evolution).


## Dependencies

It relies on:

* [JGEA](https://github.com/ericmedvet/jgea), for the evolutionary optimization;

* [2D-VSR-Sim](https://github.com/ericmedvet/2dhmsr), for the simulation of VSRs.

The relative jars have already been included in the directory `libs`. See `pom.xml` for more details on dependencies.

## Usage

This is a table of possible command-line arguments:
Argument       | Type                                         | Optional (yes/no) | Default
---------------|----------------------------------------------|-------------------|-------------------------
evolver        | {cmaes, ga, se-geno, se-shape, se-behaviour} | no                | -
representation | {homogeneous, heterogeneous}                 | no                | -
seed           | integer                                      | no                | -
threads        | integer                                      | yes               | # available cores on CPU

where {A, B, C} denotes a finite and discrete set of possible choices for the corresponding argument.

## TODO
Bibliography, if accepted
