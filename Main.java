
import com.google.common.base.Stopwatch;
import com.google.common.collect.Range;
import it.units.erallab.hmsrobots.core.objects.Robot;
import it.units.erallab.hmsrobots.core.sensors.Sensor;
import it.units.erallab.hmsrobots.tasks.locomotion.Footprint;
import it.units.erallab.hmsrobots.tasks.locomotion.Locomotion;
import it.units.erallab.hmsrobots.tasks.locomotion.Outcome;
import it.units.erallab.hmsrobots.tasks.locomotion.Outcome.Component;
import it.units.erallab.hmsrobots.util.Grid;
import it.units.erallab.hmsrobots.util.SerializationUtils;
import it.units.erallab.hmsrobots.util.Utils;
import it.units.erallab.hmsrobots.util.SerializationUtils.Mode;
import it.units.malelab.jgea.Worker;
import it.units.malelab.jgea.core.IndependentFactory;
import it.units.malelab.jgea.core.Individual;
import it.units.malelab.jgea.core.evolver.CMAESEvolver;
import it.units.malelab.jgea.core.evolver.Evolver;
import it.units.malelab.jgea.core.evolver.StandardEvolver;
import it.units.malelab.jgea.core.evolver.StandardWithEnforcedDiversityEvolver;
import it.units.malelab.jgea.core.evolver.speciation.KMeansSpeciator;
import it.units.malelab.jgea.core.evolver.speciation.SpeciatedEvolver;
import it.units.malelab.jgea.core.evolver.stopcondition.Births;
import it.units.malelab.jgea.core.listener.Listener;
import it.units.malelab.jgea.core.listener.PrintStreamListener;
import it.units.malelab.jgea.core.listener.PrintStreamPopulationListener;
import it.units.malelab.jgea.core.listener.collector.Basic;
import it.units.malelab.jgea.core.listener.collector.Diversity;
import it.units.malelab.jgea.core.listener.collector.Item;
import it.units.malelab.jgea.core.listener.collector.Population;
import it.units.malelab.jgea.core.order.PartialComparator;
import it.units.malelab.jgea.core.selector.Tournament;
import it.units.malelab.jgea.core.selector.Worst;
import it.units.malelab.jgea.core.util.Args;
import it.units.malelab.jgea.core.util.Misc;
import it.units.malelab.jgea.distance.LNorm;
import it.units.malelab.jgea.representation.sequence.FixedLengthListFactory;
import it.units.malelab.jgea.representation.sequence.UniformCrossover;
import it.units.malelab.jgea.representation.sequence.numeric.GaussianMutation;
import it.units.malelab.jgea.representation.sequence.numeric.GeometricCrossover;
import it.units.malelab.jgea.representation.sequence.numeric.UniformDoubleFactory;
import java.io.FileNotFoundException;
import java.io.PrintStream;
import java.util.ArrayList;
import java.util.Collection;
import java.util.List;
import java.util.Map;
import java.util.Objects;
import java.util.Random;
import java.util.concurrent.ExecutionException;
import java.util.concurrent.TimeUnit;
import java.util.function.Function;
import java.util.function.Predicate;
import java.util.stream.Collectors;
import org.dyn4j.dynamics.Settings;

public class Main extends Worker {
    private static final int CACHE_SIZE = 0;
    private static int seed;
    private static int nBirths;
    private static double episodeTime;
    private static double frequencyThreshold;
    private static int nFrequencySamples;
    private static final String statsDir = "./output/posedist/";
    //private static final String serializedDir = "./output/enforced/serialized";
    private static Settings physicsSettings;

    public Main(String[] args) {
        super(args);
    }

    public static void main(String[] args) {
        new Main(args);
    }

    public void run() {
        int[] innerNeurons = new int[0];
        episodeTime = Args.d(this.a("episodeT", "30.0"));
        nBirths = Args.i(this.a("nBirths", "30000"));
        seed = Integer.parseInt(this.a("seed", null));
        frequencyThreshold = 10.0D;
        nFrequencySamples = 100;
        String[] sizes = new String[]{"5x5"};
        String[] controllers = new String[]{"heterogeneous"};
        String[] sensorsConfig = new String[]{"vel-area-touch"};
        String[] signals = new String[]{"1"};
        physicsSettings = new Settings();

        //for (int i=0; i < 10; ++i) {
        //    seed = i;
            for (String size : sizes) {
                for (String controller : controllers) {
                    for (String signal : signals) {
                        for (String config : sensorsConfig) {
                            try {
                                this.evolve(controller, size, config, signal, innerNeurons);
                            } catch (FileNotFoundException e) {
                                e.printStackTrace();
                            }
                        }
                    }
                }
            }
        //}

    }

    private void evolve(String controller, String size, String sensorConfig, String signal, int[] innerNeurons) throws FileNotFoundException {
        String commonFileName = statsDir.split("/")[2] + "." + seed + "." + controller + "." + size + "." + sensorConfig + "." + signal;
        String statsFileName = statsDir + commonFileName + ".stats.csv";
        //String serializedFileName = "./output/enforced/serialized/" + commonFileName + ".serialized.csv";
        int width = Integer.parseInt(size.split("x")[0]);
        int height = Integer.parseInt(size.split("x")[1]);
        List<Sensor> sensors = RobotMapper.getSensors(sensorConfig);
        RobotMapper mapper = RobotMapper.createMapper(controller, width, height, sensors, innerNeurons, Integer.parseInt(signal));
        IndependentFactory<List<Double>> factory = new FixedLengthListFactory<>(mapper.getGenotypeSize(), new UniformDoubleFactory(-1.0D, 1.0D));
        Function<Robot<?>, Outcome> trainingTask = new Locomotion(episodeTime, Locomotion.createTerrain("flat"), physicsSettings);

        Listener<? super List<Double>, ? super Robot<?>, ? super Outcome> newListener = new PrintStreamPopulationListener<List<Double>, Robot<?>, Outcome>(new PrintStreamListener<>(new PrintStream(statsFileName), false, 1000000000, ",", ",", List.of(new Basic(), new Population(), new Diversity())), (individual) ->
            outcomeTransformer(trainingTask.apply(SerializationUtils.clone(individual.getSolution())), frequencyThreshold, nFrequencySamples),
        (individual) -> List.of(new Item("shape.static", printBodies(Utils.cropGrid((individual.getSolution()).getVoxels(), Objects::nonNull), Objects::nonNull), "%s")/*, new Item("serialized.genotype", SerializationUtils.serialize(individual.getGenotype(), Mode.GZIPPED_JSON), "%s")*/));
        //Listener newSerializedListener = (new PrintStreamPopulationListener(new PrintStreamListener(new PrintStream(serializedFileName), false, 1000000000, ",", ",", List.of(new Basic(), new Population(), new Diversity())), new Function[]{(i) -> {
        //    return List.of(new Item("robot.id", System.identityHashCode(i), "%d"), new Item("serialized.genotype", SerializationUtils.serialize(i.getGenotype(), Mode.GZIPPED_JSON), "%s"));
        //}})).then(newListener);

        try {
            Stopwatch stopwatch = Stopwatch.createStarted();
            L.info(String.format("Starting %s", commonFileName));
            int popSize = 100;
            Evolver<List<Double>, Robot<?>, Outcome> evolver = new SpeciatedEvolver/*new StandardEvolver*/<List<Double>, Robot<?>, Outcome>/*new CMAESEvolver<>*/(mapper, factory, PartialComparator.from(Outcome.class).reversed().comparing(Individual::getFitness), popSize, Map.of(new GaussianMutation(0.35D), 0.02D, new GeometricCrossover(Range.closed(-1.0D, 2.0D)), 0.08D),
                    5, new KMeansSpeciator<>(10, 200, new LNorm(2), individual -> {
                    //    List<Double> g = individual.getGenotype();
                        //return individual.getFitness().getCenterPowerSpectrum(Component.Y, 0, frequencyThreshold, nFrequencySamples).stream().mapToDouble(Outcome.Mode::getStrength).toArray();
                        return individual.getFitness().getAveragePosture().stream().mapToDouble(b -> (b.getValue()) ? 1.0 : 0.0).toArray();
                    //    double[] out = new double[g.size()];
                    //    for (int i=0; i < g.size(); ++i) {
                    //        out[i] = g.get(i);
                    //    }
                    //    return out;
            }), 0.75);
            //        new Tournament(5), new Worst(), popSize, true);//, 100);
            Collection<Robot<?>> solutions = evolver.solve(trainingTask/*.andThen(Outcome::getDistance)*/, new Births(nBirths), new Random(seed), this.executorService, Listener.onExecutor(newListener, this.executorService));
            L.info(String.format("Done %s: %d solutions in %4ds", commonFileName, solutions.size(), stopwatch.elapsed(TimeUnit.SECONDS)));
        }
        catch (ExecutionException | InterruptedException e) {
            L.severe(String.format("Cannot complete %s due to %s", commonFileName, e));
            e.printStackTrace();
        }
    }

    public static List<Item> outcomeTransformer(Outcome o, double frequencyThreshold, int nSamples) {
        List<Item> result = new ArrayList<>();
        result.add(new Item("time", o.getComputationTime(), "5.3f"));
        result.add(new Item("distance", o.getDistance(), "%5.1f"));
        result.add(new Item("velocity", o.getVelocity(), "%5.1f"));
        result.add(new Item("shape.dynamic", Grid.toString(o.getAveragePosture(), (Predicate<Boolean>)  b -> b,"|"), "%10.10s"));
        result.add(new Item("compressed.frequency.y", o.getCenterPowerSpectrum(Component.Y, 0.0D, frequencyThreshold, nSamples).stream().map((m) -> String.valueOf(m.getStrength())).collect(Collectors.joining("-")), "%s"));
        result.add(new Item("compressed.frequency.x", o.getCenterPowerSpectrum(Component.X, 0.0D, frequencyThreshold, nSamples).stream().map((m) -> String.valueOf(m.getStrength())).collect(Collectors.joining("-")), "%s"));
        Outcome.Gait g = o.getMainGait();
        if (g != null) {
            result.add(new Item("gait.average.touch.area", g.getAvgTouchArea(), "%5.3f"));
            result.add(new Item("gait.coverage", g.getCoverage(), "%4.2f"));
            result.add(new Item("gait.mode.interval", g.getModeInterval(), "%3.1f"));
            result.add(new Item("gait.purity", g.getPurity(), "%4.2f"));
            result.add(new Item("gait.num.unique.footprints", g.getFootprints().stream().distinct().count(), "%2d"));
            result.add(new Item("gait.num.footprints", g.getFootprints().size(), "%2d"));
            result.add(new Item("gait.footprints", g.getFootprints().stream().map(Footprint::toString).collect(Collectors.joining("|")), "%10.10s"));
        }
        else {
            result.add(new Item("gait.average.touch.area", "", "%5.3f"));
            result.add(new Item("gait.coverage", "", "%4.2f"));
            result.add(new Item("gait.mode.interval", "", "%3.1f"));
            result.add(new Item("gait.purity", "", "%4.2f"));
            result.add(new Item("gait.num.unique.footprints", "", "%2d"));
            result.add(new Item("gait.num.footprints", "", "%2d"));
            result.add(new Item("gait.footprints", "", "%10.10s"));
        }
        return result;
    }

    public static <K> String printBodies(Grid<K> grid, Predicate<K> p) {
        StringBuilder sb = new StringBuilder();
        for(int y = 0; y < grid.getH(); ++y) {
            for(int x = 0; x < grid.getW(); ++x) {
                sb.append(p.test(grid.get(x, y)) ? "1" : "0");
            }

            if (y < grid.getH() - 1) {
                sb.append("/");
            }
        }
        return sb.toString();
    }

}
