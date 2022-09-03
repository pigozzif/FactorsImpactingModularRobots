package it.units.erallab.factors;

import it.units.malelab.jgea.core.IndependentFactory;
import it.units.malelab.jgea.representation.sequence.FixedLengthListFactory;
import it.units.malelab.jgea.representation.sequence.numeric.UniformDoubleFactory;

import java.util.List;
import java.util.Random;


public class GaussianFactory implements IndependentFactory<List<Double>> {

  private final FixedLengthListFactory<Double> controllerFactory;
  private final FixedLengthListFactory<Double> bodyFactory;
  private static final int NUM_GAUSSIANS = 5;

  public GaussianFactory(int genotypeSize) {
    this.controllerFactory = new FixedLengthListFactory<>(genotypeSize - (NUM_GAUSSIANS * 5), new UniformDoubleFactory(-1d, 1d));
    this.bodyFactory = new FixedLengthListFactory<>(NUM_GAUSSIANS * 5, new UniformDoubleFactory(0d, 1d));
  }

  @Override
  public List<Double> build(Random random) {
    // weights of the body
    List<Double> genotype = this.bodyFactory.build(random);
    // weights of the neural network
    genotype.addAll(this.controllerFactory.build(random));
    return genotype;
  }

}