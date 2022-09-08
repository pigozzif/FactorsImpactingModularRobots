package it.units.erallab.factors;

import com.google.common.base.Stopwatch;
import com.google.common.collect.Range;
import it.units.erallab.hmsrobots.core.objects.Robot;
import it.units.erallab.hmsrobots.core.sensors.Sensor;
import it.units.erallab.hmsrobots.tasks.locomotion.Locomotion;
import it.units.erallab.hmsrobots.tasks.locomotion.Outcome;
import it.units.erallab.hmsrobots.tasks.locomotion.Outcome.Component;
import it.units.erallab.hmsrobots.util.Grid;
import it.units.malelab.jgea.Worker;
import it.units.malelab.jgea.core.IndependentFactory;
import it.units.malelab.jgea.core.Individual;
import it.units.malelab.jgea.core.evolver.*;
import it.units.malelab.jgea.core.evolver.speciation.KMeansSpeciator;
import it.units.malelab.jgea.core.evolver.speciation.SpeciatedEvolver;
import it.units.malelab.jgea.core.evolver.stopcondition.Births;
import it.units.malelab.jgea.core.listener.CSVPrinter;
import it.units.malelab.jgea.core.listener.Listener;
import it.units.malelab.jgea.core.listener.NamedFunction;
import it.units.malelab.jgea.core.order.PartialComparator;
import it.units.malelab.jgea.core.selector.Last;
import it.units.malelab.jgea.core.selector.Tournament;
import it.units.malelab.jgea.core.util.Args;
import it.units.malelab.jgea.core.util.Misc;
import it.units.malelab.jgea.distance.LNorm;
import it.units.malelab.jgea.representation.sequence.FixedLengthListFactory;
import it.units.malelab.jgea.representation.sequence.numeric.GaussianMutation;
import it.units.malelab.jgea.representation.sequence.numeric.GeometricCrossover;
import it.units.malelab.jgea.representation.sequence.numeric.UniformDoubleFactory;

import java.io.*;
import java.util.*;
import java.util.concurrent.ExecutionException;
import java.util.concurrent.TimeUnit;
import java.util.function.Function;

import org.dyn4j.dynamics.Settings;

import static it.units.malelab.jgea.core.listener.NamedFunctions.*;


public class Main extends Worker {

    private static int seed;
    private static String evolverName;
    private static int nBirths;
    private static String terrain;
    private static double episodeTime;
    private static final double frequencyThreshold = 10.0D;
    private static final int nFrequencySamples = 100;
    private static String  bestFileName = "./output/";
    private static String allFileName = "";
    private static Settings physicsSettings;

    public Main(String[] args) {
        super(args);
    }

    public static void main(String[] args) {
        new Main(args);
    }

    public void run() {
        int[] innerNeurons = new int[0];
        seed = Args.i(this.a("seed", null));
        evolverName = this.a("evolver", null);
        String representation = this.a("representation", null);
        episodeTime = 30.0D;
        nBirths = Args.i(this.a("births", null));
        terrain = this.a("terrain", "flat");
        String size = this.a("size", "5x5");
        String sensorsConfig = "vel-area-touch";
        String signals = "1";
        physicsSettings = new Settings();
        bestFileName += String.join(".", evolverName, String.valueOf(seed), representation, size, sensorsConfig, signals, terrain);
        allFileName += bestFileName + "." + "all.csv";
        bestFileName += ".csv";

        try {
            this.evolve(representation, size, sensorsConfig, signals, innerNeurons);
        } catch (FileNotFoundException e) {
            e.printStackTrace();
        }
    }

    private void evolve(String representation, String size, String sensorConfig, String signal, int[] innerNeurons) throws FileNotFoundException {
        int width = Integer.parseInt(size.split("x")[0]);
        int height = Integer.parseInt(size.split("x")[1]);
        List<Sensor> sensors = RobotMapper.getSensors(sensorConfig);
        RobotMapper mapper = RobotMapper.createMapper(representation, width, height, sensors, innerNeurons, Integer.parseInt(signal));
        IndependentFactory<List<Double>> factory = (representation.contains("direct")) ? new FixedLengthListFactory<>(mapper.getGenotypeSize(), new UniformDoubleFactory(-1.0D, 1.0D)) : new GaussianFactory(mapper.getGenotypeSize());
        Function<Robot<?>, Outcome> trainingTask = new Locomotion(episodeTime, Locomotion.createTerrain(terrain), physicsSettings);

        try {
            Stopwatch stopwatch = Stopwatch.createStarted();
            L.info(String.format("Starting %s", bestFileName));
            Collection<Robot<?>> solutions = switch (evolverName) {
                case "es" -> this.evolveES(factory, mapper, trainingTask);
                case "ga" -> this.evolveGA(factory, mapper, trainingTask);
                case "se-geno" -> this.evolveSEgeno(factory, mapper, trainingTask);
                case "se-shape" -> this.evolveSEshape(factory, mapper, trainingTask);
                case "se-behaviour" -> this.evolveSEbehaviour(factory, mapper, trainingTask);
                case "me" -> this.evolveMAPElites(factory, mapper, trainingTask);
                default -> throw new IllegalArgumentException("Unknown evolver name: " + evolverName);
            };
            L.info(String.format("Done %s: %d solutions in %4ds", bestFileName, solutions.size(), stopwatch.elapsed(TimeUnit.SECONDS)));
        }
        catch (ExecutionException | InterruptedException e) {
            L.severe(String.format("Cannot complete %s due to %s", bestFileName, e));
            e.printStackTrace();
        }
    }

    private Collection<Robot<?>> evolveES(IndependentFactory<List<Double>> factory, RobotMapper mapper, Function<Robot<?>, Outcome> trainingTask) throws ExecutionException, InterruptedException {
        Evolver<List<Double>, Robot<?>, Outcome> evolver = new CanonicalEvolutionaryStrategy<>(mapper, factory, PartialComparator.from(Outcome.class).reversed().comparing(Individual::getFitness), 0.35, 40, 40 / 4);  //BasicEvolutionaryStrategy<>(mapper, factory, PartialComparator.from(Outcome.class).reversed().comparing(Individual::getFitness), 0.35, 40, 40 / 4, 1, true);
        return evolver.solve(trainingTask, new Births(nBirths), new Random(seed), this.executorService, createListenerFactory().build());
    }

    private Collection<Robot<?>> evolveGA(IndependentFactory<List<Double>> factory, RobotMapper mapper, Function<Robot<?>, Outcome> trainingTask) throws ExecutionException, InterruptedException {
        Evolver<List<Double>, Robot<?>, Outcome> evolver = new StandardEvolver<>(mapper, factory, PartialComparator.from(Outcome.class).reversed().comparing(Individual::getFitness), 100, Map.of(new GaussianMutation(0.35D), 0.02D, new GeometricCrossover(Range.closed(-1.0D, 2.0D)).andThen(new GaussianMutation(0.1D)), 0.08D), new Tournament(5), new Last(), 100, true, false);
        return evolver.solve(trainingTask, new Births(nBirths), new Random(seed), this.executorService, createListenerFactory().build());
    }

    private Collection<Robot<?>> evolveSEgeno(IndependentFactory<List<Double>> factory, RobotMapper mapper, Function<Robot<?>, Outcome> trainingTask) throws ExecutionException, InterruptedException {
        Evolver<List<Double>, Robot<?>, Outcome> evolver = new SpeciatedEvolver<>(mapper, factory, PartialComparator.from(Outcome.class).reversed().comparing(Individual::getFitness), 100, Map.of(new GaussianMutation(0.35D), 0.02D, new GeometricCrossover(Range.closed(-1.0D, 2.0D)), 0.08D),
                5, new KMeansSpeciator<>(10, 200, new LNorm(2), individual -> {
                                        List<Double> g = individual.getGenotype();
                                        double[] out = new double[g.size()];
                                        for (int i = 0; i < g.size(); ++i) {
                                          out[i] = g.get(i);
                                        }
                                        return out;
                                    }), 0.75, false);
        return evolver.solve(trainingTask, new Births(nBirths), new Random(seed), this.executorService, createListenerFactory().build());
    }

    private Collection<Robot<?>> evolveSEshape(IndependentFactory<List<Double>> factory, RobotMapper mapper, Function<Robot<?>, Outcome> trainingTask) throws ExecutionException, InterruptedException {
        Evolver<List<Double>, Robot<?>, Outcome> evolver = new SpeciatedEvolver<>(mapper, factory, PartialComparator.from(Outcome.class).reversed().comparing(Individual::getFitness), 100, Map.of(new GaussianMutation(0.35D), 0.02D, new GeometricCrossover(Range.closed(-1.0D, 2.0D)), 0.08D),
                5, new KMeansSpeciator<>(10, 200, new LNorm(2), individual -> individual.getFitness().getAveragePosture().stream().mapToDouble(b -> (b.getValue()) ? 1.0 : 0.0).toArray()), 0.75, false);
        return evolver.solve(trainingTask, new Births(nBirths), new Random(seed), this.executorService, createListenerFactory().build());
    }

    private Collection<Robot<?>> evolveSEbehaviour(IndependentFactory<List<Double>> factory, RobotMapper mapper, Function<Robot<?>, Outcome> trainingTask) throws ExecutionException, InterruptedException {
        Evolver<List<Double>, Robot<?>, Outcome> evolver = new SpeciatedEvolver<>(mapper, factory, PartialComparator.from(Outcome.class).reversed().comparing(Individual::getFitness), 100, Map.of(new GaussianMutation(0.35D), 0.02D, new GeometricCrossover(Range.closed(-1.0D, 2.0D)), 0.08D),
                5, new KMeansSpeciator<>(10, 200, new LNorm(2), individual -> individual.getFitness().getCenterPowerSpectrum(Component.Y, 0, frequencyThreshold, nFrequencySamples).stream().mapToDouble(Outcome.Mode::getStrength).toArray()), 0.75, false);
        return evolver.solve(trainingTask, new Births(nBirths), new Random(seed), this.executorService, createListenerFactory().build());
    }

    private Collection<Robot<?>> evolveMAPElites(IndependentFactory<List<Double>> factory, RobotMapper mapper, Function<Robot<?>, Outcome> trainingTask) throws ExecutionException, InterruptedException {
        Evolver<List<Double>, Robot<?>, Outcome> evolver = new MAPElitesEvolver<>(x -> List.of((double) x.getSolution().getVoxels().count(Objects::nonNull), ExtractFeatures.shapeCompactness(Grid.create(10, 10, (i, j) -> x.getSolution().getVoxels().get(i, j) != null)), ExtractFeatures.shapeElongation(Grid.create(10, 10, (i, j) -> x.getSolution().getVoxels().get(i, j) != null), 8)), List.of(10.0, 10.0), List.of(0.0, 0.0), List.of(10, 10), mapper, factory, PartialComparator.from(Outcome.class).reversed().comparing(Individual::getFitness), new GaussianMutation(0.35), 20, 20, x -> x.getFitness().getVelocity());
        return evolver.solve(trainingTask, new Births(nBirths), new Random(seed), this.executorService, createListenerFactory().build());
    }

    private List<Double> getSpectrumDescriptor(Individual<List<Double>, Robot<?>, Outcome> ind) {
        Outcome o = ind.getFitness();
        double[] xSpectrum = o.getCenterPowerSpectrum(Outcome.Component.X, 0.0, frequencyThreshold, nFrequencySamples).stream()
                .mapToDouble(Outcome.Mode::getStrength)
                .toArray();
        double[] ySpectrum = o.getCenterPowerSpectrum(Outcome.Component.Y, 0.0, frequencyThreshold, nFrequencySamples).stream()
                .mapToDouble(Outcome.Mode::getStrength)
                .toArray();
        double[] spectrum = new double[nFrequencySamples * 2];
        System.arraycopy(xSpectrum, 0, spectrum, 0, nFrequencySamples);
        System.arraycopy(ySpectrum, 0, spectrum, nFrequencySamples, nFrequencySamples);
        List<Double> descriptors = new ArrayList<>();
        for (double d : spectrum){
            descriptors.add(d);
        }
        return descriptors;
    }

    private static Listener.Factory<Event<?, ? extends Robot<?>, ? extends Outcome>> createListenerFactory() {
        // consumers
        Function<Outcome, Double> fitnessFunction = Outcome::getDistance;
        List<NamedFunction<Event<?, ? extends Robot<?>, ? extends Outcome>, ?>> basicFunctions = AuxUtils.basicFunctions();
        List<NamedFunction<Individual<?, ? extends Robot<?>, ? extends Outcome>, ?>> basicIndividualFunctions = AuxUtils.individualFunctions(fitnessFunction);
        List<NamedFunction<Event<?, ? extends Robot<?>, ? extends Outcome>, ?>> populationFunctions = AuxUtils.populationFunctions(fitnessFunction);
        List<NamedFunction<Outcome, ?>> basicOutcomeFunctions = AuxUtils.basicOutcomeFunctions();
        List<NamedFunction<Outcome, ?>> detailedOutcomeFunctions = AuxUtils.detailedOutcomeFunctions(0.0, frequencyThreshold, nFrequencySamples);
        Listener.Factory<Event<?, ? extends Robot<?>, ? extends Outcome>> factory = Listener.Factory.deaf();
        // file listeners
        if (bestFileName != null) {
            factory = factory.and(new CSVPrinter<>(Misc.concat(List.of(
                    basicFunctions,
                    populationFunctions
            )), new File(bestFileName)
            ));
        }
        if (allFileName != null) {
            factory = factory.and(Listener.Factory.forEach(
                    event -> event.getOrderedPopulation().all(),
                    new CSVPrinter<>(Misc.concat(List.of(
                            NamedFunction.then(as(Individual.class),
                                List.of(f("iterations", x -> ((Individual<?, ? extends Robot<?>, ? extends Outcome>) x).getBirthIteration()))),
                            NamedFunction.then(as(Outcome.class).of(fitness()), detailedOutcomeFunctions),
                            NamedFunction.then(as(Outcome.class).of(fitness()), basicOutcomeFunctions),
                            basicIndividualFunctions,
                            AuxUtils.serializationFunction(true)
                            )), new File(allFileName))));
        }
        return factory;
    }

}
