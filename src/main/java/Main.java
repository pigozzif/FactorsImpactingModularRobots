
import com.google.common.base.Stopwatch;
import com.google.common.collect.Range;
import it.units.erallab.hmsrobots.core.objects.Robot;
import it.units.erallab.hmsrobots.core.sensors.Sensor;
import it.units.erallab.hmsrobots.tasks.locomotion.Locomotion;
import it.units.erallab.hmsrobots.tasks.locomotion.Outcome;
import it.units.erallab.hmsrobots.tasks.locomotion.Outcome.Component;
import it.units.malelab.jgea.Worker;
import it.units.malelab.jgea.core.IndependentFactory;
import it.units.malelab.jgea.core.Individual;
import it.units.malelab.jgea.core.evolver.CMAESEvolver;
import it.units.malelab.jgea.core.evolver.Event;
import it.units.malelab.jgea.core.evolver.Evolver;
import it.units.malelab.jgea.core.evolver.StandardEvolver;
import it.units.malelab.jgea.core.evolver.speciation.KMeansSpeciator;
import it.units.malelab.jgea.core.evolver.speciation.SpeciatedEvolver;
import it.units.malelab.jgea.core.evolver.stopcondition.Births;
import it.units.malelab.jgea.core.listener.CSVPrinter;
import it.units.malelab.jgea.core.listener.Listener;
import it.units.malelab.jgea.core.listener.NamedFunction;
import it.units.malelab.jgea.core.order.PartialComparator;
import it.units.malelab.jgea.core.selector.Tournament;
import it.units.malelab.jgea.core.selector.Worst;
import it.units.malelab.jgea.core.util.Args;
import it.units.malelab.jgea.core.util.Misc;
import it.units.malelab.jgea.core.util.Pair;
import it.units.malelab.jgea.distance.LNorm;
import it.units.malelab.jgea.representation.sequence.FixedLengthListFactory;
import it.units.malelab.jgea.representation.sequence.numeric.GaussianMutation;
import it.units.malelab.jgea.representation.sequence.numeric.GeometricCrossover;
import it.units.malelab.jgea.representation.sequence.numeric.UniformDoubleFactory;

import java.io.File;
import java.io.FileNotFoundException;
import java.util.Collection;
import java.util.List;
import java.util.Map;
import java.util.Random;
import java.util.concurrent.ExecutionException;
import java.util.concurrent.TimeUnit;
import java.util.function.Function;
import java.util.stream.Collectors;

import org.dyn4j.dynamics.Settings;

import static it.units.malelab.jgea.core.listener.NamedFunctions.*;


public class Main extends Worker {

    private static int seed;
    private static String evolverName;
    private static int nBirths;
    private static double episodeTime;
    private static final double frequencyThreshold = 10.0D;
    private static final int nFrequencySamples = 100;
    private static String  bestFileName = "./output/";
    private static String lastFileName = "./output/";
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
        if (! (evolverName.equals("cmaes") || evolverName.equals("ga") || evolverName.equals("se-geno") ||
                evolverName.equals("se-shape") || evolverName.equals("se-behaviour"))) {
            throw new IllegalArgumentException("Evolver name must be one of [cmaes, ga, se-geno, se-shape, se-behaviour]");
        }
        String representation = this.a("representation", null);
        if (! (representation.equals("homogeneous") || representation.equals("heterogeneous"))) {
            throw new IllegalArgumentException("Representation name must be one of [homogeneous, heterogeneous]");
        }
        episodeTime = 30.0D;
        nBirths = 200;
        String size = "10x10";
        String sensorsConfig = "vel-area-touch";
        String signals = "1";
        physicsSettings = new Settings();
        bestFileName += evolverName + "." + seed + "." + representation + "." + size + "." + sensorsConfig + "." + signals;
        lastFileName += bestFileName + "." + "last.csv";
        bestFileName += ".csv";

        try {
            this.evolve(representation, size, sensorsConfig, signals, innerNeurons);
        } catch (FileNotFoundException e) {
            e.printStackTrace();
        }
    }

    private void evolve(String controller, String size, String sensorConfig, String signal, int[] innerNeurons) throws FileNotFoundException {
        int width = Integer.parseInt(size.split("x")[0]);
        int height = Integer.parseInt(size.split("x")[1]);
        List<Sensor> sensors = RobotMapper.getSensors(sensorConfig);
        RobotMapper mapper = RobotMapper.createMapper(controller, width, height, sensors, innerNeurons, Integer.parseInt(signal));
        IndependentFactory<List<Double>> factory = new FixedLengthListFactory<>(mapper.getGenotypeSize(), new UniformDoubleFactory(-1.0D, 1.0D));
        Function<Robot<?>, Outcome> trainingTask = new Locomotion(episodeTime, Locomotion.createTerrain("flat"), physicsSettings);

        try {
            Stopwatch stopwatch = Stopwatch.createStarted();
            L.info(String.format("Starting %s", bestFileName));
            Collection<Robot<?>> solutions = switch (evolverName) {
                case "cmaes" -> this.evolveCMAES(factory, mapper, trainingTask);
                case "ga" -> this.evolveGA(factory, mapper, trainingTask);
                case "se-geno" -> this.evolveSEgeno(factory, mapper, trainingTask);
                case "se-shape" -> this.evolveSEshape(factory, mapper, trainingTask);
                default -> this.evolveSEbehaviour(factory, mapper, trainingTask);
            };
            L.info(String.format("Done %s: %d solutions in %4ds", bestFileName, solutions.size(), stopwatch.elapsed(TimeUnit.SECONDS)));
        }
        catch (ExecutionException | InterruptedException e) {
            L.severe(String.format("Cannot complete %s due to %s", bestFileName, e));
            e.printStackTrace();
        }
    }

    private Collection<Robot<?>> evolveCMAES(IndependentFactory<List<Double>> factory, RobotMapper mapper, Function<Robot<?>, Outcome> trainingTask) throws ExecutionException, InterruptedException {
        Evolver<List<Double>, Robot<?>, Outcome> evolver = new CMAESEvolver<>(mapper, factory, PartialComparator.from(Outcome.class).reversed().comparing(Individual::getFitness));
        return evolver.solve(trainingTask, new Births(nBirths), new Random(seed), this.executorService, createListenerFactory().build());
    }

    private Collection<Robot<?>> evolveGA(IndependentFactory<List<Double>> factory, RobotMapper mapper, Function<Robot<?>, Outcome> trainingTask) throws ExecutionException, InterruptedException {
        Evolver<List<Double>, Robot<?>, Outcome> evolver = new StandardEvolver<>(mapper, factory, PartialComparator.from(Outcome.class).reversed().comparing(Individual::getFitness), 100, Map.of(new GaussianMutation(0.35D), 0.02D, new GeometricCrossover(Range.closed(-1.0D, 2.0D)), 0.08D), new Tournament(5), new Worst(), 100, true, false);
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

    private Listener.Factory<Event<?, ? extends Robot<?>, ? extends Outcome>> createListenerFactory() {
        Function<Outcome, Double> fitnessFunction = Outcome::getDistance;
        // consumers
        List<NamedFunction<Event<?, ? extends Robot<?>, ? extends Outcome>, ?>> basicFunctions = AuxUtils.basicFunctions();
        List<NamedFunction<Event<?, ? extends Robot<?>, ? extends Outcome>, ?>> populationFunctions = AuxUtils.populationFunctions(fitnessFunction);
        List<NamedFunction<Individual<?, ? extends Robot<?>, ? extends Outcome>, ?>> individualFunctions = AuxUtils.individualFunctions(fitnessFunction);
        List<NamedFunction<Outcome, ?>> basicOutcomeFunctions = AuxUtils.basicOutcomeFunctions();
        List<NamedFunction<Outcome, ?>> detailedOutcomeFunctions = AuxUtils.detailedOutcomeFunctions(0, frequencyThreshold, nFrequencySamples);
        // file listener (one best per iteration)
        Listener.Factory<Event<?, ? extends Robot<?>, ? extends Outcome>> factory = new CSVPrinter<>(Misc.concat(List.of(
                    basicFunctions,
                    populationFunctions,
                    NamedFunction.then(best(), individualFunctions),
                    NamedFunction.then(as(Outcome.class).of(fitness()).of(best()), basicOutcomeFunctions),
                    NamedFunction.then(as(Outcome.class).of(fitness()).of(best()), detailedOutcomeFunctions),
                    NamedFunction.then(best(), AuxUtils.serializationFunction(true))
            )), new File(bestFileName)
            );
        // file listener (all individuals of last iteration)
        factory = factory.and(Listener.Factory.forEach((Event<?, ? extends Robot<?>, ? extends Outcome>) event -> event.getOrderedPopulation().all().stream()
                .map(i -> Pair.of(event, i))
                .collect(Collectors.toList()), new CSVPrinter<>(Misc.concat(
                NamedFunction.then(f("individual", Pair::second), individualFunctions),
                NamedFunction.then(f("individual", Pair::second), AuxUtils.serializationFunction(true)))
                //NamedFunction.then(as(Outcome.class).of(fitness()).of(all()), basicOutcomeFunctions),
                //NamedFunction.then(as(Outcome.class).of(fitness()).of(all()), detailedOutcomeFunctions),
                //NamedFunction.then(all(), AuxUtils.serializationFunction(true))
        ), new File(lastFileName)
        )).onLast();
        return factory;
    }

}
