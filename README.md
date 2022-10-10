# Factors impacting diversity and effectiveness of evolved modular robots
This is the official repository for the ACM TELO (Transactions on Evolutionary Learning and Optimization) paper "Factors impacting diversity and effectiveness of evolved modular robots", hosting all the code necessary to replicate the experiments. This work was carried on at the Evolutionary Robotics and Artificial Life Laboratory (ERALLab) at the Department of Engineering and Architecture, University of Trieste (Italy).

## Scope
By running
```
java -cp libs:JGEA.jar:libs/TwoDimHighlyModularSoftRobots.jar:target/VSRBiodiversity.jar it.units.erallab.factors.Main {args}
```
where `{args}` is a placeholder for the arguments you provide (see below), you will launch an evolutionary optimization for evolving jointly the body (number and arrangement of the voxels) and brain (the controller, an Artificial Neural Network (ANN)) of Voxel-based Soft Robots (VSRs). At the same time, a number of evolution metadata and relevant features will be saved inside the `output` folder. Features include descriptors of VSRs that we devised as a way of characterizing their shape and behaviour. We employed the features (in a supervised learning pipeline) to assign each robot to a species, measure biodiversity in VSRs and study how it mutates as the evolutionary algorithm and the controller representation are changed. The project has been tested with Java `14.0.2`.

## Structure
* `src` contains all the source code for the project;
* `libs` contains the .jar files for some of the dependencies (see below);
* `Data_Analysis_Notebook.ipynb` is Jupyter Notebook (Python) with some routines and starter code to perform analysis on the evolution output files;
* `models` contains the pickled random forest models for the supervised classification of VSRs into "species";
* `target` contains the target .jar file.

## Dependencies
It relies on:
* [JGEA](https://github.com/ericmedvet/jgea), for the evolutionary optimization;
* [2D-VSR-Sim](https://github.com/ericmedvet/2dhmsr), for the simulation of VSRs.

The relative jars have already been included in the directory `libs`. See `pom.xml` for more details on dependencies; other versions of these dependencies are not guaranteed to deliver the same results.

## Usage
This is a table of possible command-line arguments:

Argument       | Type                                         | Optional (yes/no) | Default
---------------|----------------------------------------------|-------------------|-------------------------
evolver        | {cmaes, ga, se-geno, se-shape, se-behaviour} | no                | -
representation | {homogeneous, heterogeneous}                 | no                | -
terrain        | {flat, uphill-20, downhill-30}               | no                | -
seed           | integer                                      | no                | -
threads        | integer                                      | yes               | # available cores on CPU

where {...} denotes a finite and discrete set of possible choices for the corresponding argument. The description for each argument is as follows:
* evolver: the evolutionary algorithm to perform optimization with.
* representation: the representation for the robotic controller. For this work, we use a distributed model (one ANN for each voxel), of which we devised two alternatives, _homogeneous_ and _heterogeneous_. In the former, all the ANNs share the same weights; in the latter, different voxels have different weights.
* terrain: the terrain to evaluate the robots on.
* seed: the random seed for the experiment.
* threads: the number of threads to perform evolution with. Defaults to the number of available cores on the current CPU. Parallelization is taken care by JGEA and implements a distributed fitness assessment.

Evolution output files contain the serialized best individual for every generation, that can be deserialized to compute any necessary shape or behavior feature.

## Bibliography
TODO, if accepted
