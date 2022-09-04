package it.units.erallab.factors;

import it.units.malelab.jgea.core.Factory;
import it.units.malelab.jgea.core.Individual;
import it.units.malelab.jgea.core.evolver.AbstractIterativeEvolver;
import it.units.malelab.jgea.core.order.PartialComparator;
import it.units.malelab.jgea.core.order.PartiallyOrderedCollection;

import java.util.*;
import java.util.concurrent.ExecutionException;
import java.util.concurrent.ExecutorService;
import java.util.function.Function;
import java.util.stream.Collectors;
import java.util.stream.IntStream;

/**
 * @author federico
 */
public class CanonicalEvolutionaryStrategy<S, F> extends AbstractIterativeEvolver<List<Double>, S, F> {

    protected static class CanonicalESState extends State {
      private final double[] weights;

      public CanonicalESState(int iterations, int births, int fitnessEvaluations, long elapsedMillis, int parentsSize) {
        super(iterations, births, fitnessEvaluations, elapsedMillis);
        this.weights = new double[parentsSize];
        for (int i = 1; i < parentsSize + 1; ++i) {
          this.weights[i - 1] = (Math.log(parentsSize) + Math.log(i)) / IntStream.range(0, parentsSize).mapToDouble(j -> Math.log(parentsSize + 0.5) - Math.log(j)).sum();
        }
      }

      public double[] getWeights() {
        return weights;
      }

    }

    private List<Double> mean;
    private final double sigma;
    private List<List<Double>> noise;
    private final int populationSize;
    private final int parentsSize;

    public CanonicalEvolutionaryStrategy(Function<? super List<Double>, ? extends S> solutionMapper, Factory<? extends List<Double>> genotypeFactory, PartialComparator<? super Individual<List<Double>, S, F>> individualComparator, double sigma, int populationSize, int parentsSize) {
      super(solutionMapper, genotypeFactory, individualComparator);
      this.mean = null;
      this.sigma = sigma;
      this.noise = null;
      this.populationSize = populationSize;
      this.parentsSize = parentsSize;
    }

    @Override
    protected Collection<Individual<List<Double>, S, F>> initPopulation(Function<S, F> fitnessFunction, Random random, ExecutorService executor, State state) throws ExecutionException, InterruptedException {
      mean = genotypeFactory.build(1, random).get(0);
      noise = IntStream.range(0, populationSize).mapToObj(i -> mean.stream().map(j -> random.nextGaussian()).collect(Collectors.toList())).collect(Collectors.toList());
      Collection<? extends List<Double>> genotypes = buildGenotypes(mean, noise, sigma, populationSize);
      return AbstractIterativeEvolver.map(genotypes, List.of(), solutionMapper, fitnessFunction, executor, state);
    }

    @Override
    protected Collection<Individual<List<Double>, S, F>> updatePopulation(PartiallyOrderedCollection<Individual<List<Double>, S, F>> orderedPopulation, Function<S, F> fitnessFunction, Random random, ExecutorService executor, State state) throws ExecutionException, InterruptedException {
      List<Individual<List<Double>, S, F>> all = new ArrayList<>(orderedPopulation.all());
      List<Integer> parents = new ArrayList<>();
      //extract parents
      while (parents.size() < parentsSize) {
        int best = -1;
        for (int j = 0; j < all.size(); ++j) {
          if (best == -1 || individualComparator.compare(all.get(j), all.get(best)).equals(PartialComparator.PartialComparatorOutcome.BEFORE)) {
            best = j;
          }
        }
        all.remove(best);
        parents.add(best);
      }
      //update mean
      CanonicalEvolutionaryStrategy.CanonicalESState s = ((CanonicalEvolutionaryStrategy.CanonicalESState) state);
      for (int i = 0; i < mean.size(); ++i) {
        double update = 0.0;
        for (int j = 0; j < parentsSize; ++j) {
          update += s.getWeights()[j] * noise.get(parents.get(j)).get(i);
        }
        mean.set(i, mean.get(i) + sigma * update);
      }
      //build offspring
      for (int i = 0; i < populationSize; ++i) {
        for (int j = 0; j < mean.size(); ++j) {
          noise.get(i).set(j, random.nextGaussian());
        }
      }
      List<List<Double>> offspringGenotypes = buildGenotypes(mean, noise, sigma, populationSize);
      return new ArrayList<>(map(offspringGenotypes, List.of(), solutionMapper, fitnessFunction, executor, state));
    }

    private static List<List<Double>> buildGenotypes(List<Double> mean, List<List<Double>> noise, double sigma, int n) {
      List<List<Double>> offspringGenotypes = new ArrayList<>();
      for (int i = 0; i < n; ++i) {
        List<Double> newGenotype = new ArrayList<>();
        for (int j = 0; j < mean.size(); ++j) {
          newGenotype.add(mean.get(j) + sigma * noise.get(i).get(j));
        }
        offspringGenotypes.add(newGenotype);
      }
      return offspringGenotypes;
    }

    @Override
    protected State initState() {
      State state = super.initState();
      return new CanonicalEvolutionaryStrategy.CanonicalESState(state.getIterations(), state.getBirths(), state.getFitnessEvaluations(), state.getElapsedMillis(), parentsSize);
    }

}
