
import it.units.erallab.hmsrobots.core.objects.Robot;
import it.units.erallab.hmsrobots.tasks.locomotion.Outcome;
import it.units.erallab.hmsrobots.util.Grid;
import it.units.erallab.hmsrobots.util.SerializationUtils;
import it.units.erallab.hmsrobots.util.Utils;
import it.units.malelab.jgea.core.Individual;
import it.units.malelab.jgea.core.evolver.Event;
import it.units.malelab.jgea.core.listener.NamedFunction;
import it.units.malelab.jgea.core.listener.NamedFunctions;
import it.units.malelab.jgea.core.util.*;

import java.util.*;
import java.util.function.Function;
import java.util.function.Predicate;
import java.util.stream.Collectors;
import java.util.stream.IntStream;

import static it.units.malelab.jgea.core.listener.NamedFunctions.*;


public class AuxUtils {

    private AuxUtils() {}

    public static List<NamedFunction<Event<?, ? extends Robot<?>, ? extends Outcome>, ?>> basicFunctions() {
        return List.of(
                iterations(),
                births(),
                fitnessEvaluations(),
                elapsedSeconds()
        );
    }

    public static List<NamedFunction<Individual<?, ? extends Robot<?>, ? extends Outcome>, ?>> serializationFunction(boolean flag) {
        if (!flag) {
            return List.of();
        }
        return List.of(f("serialized", r -> SerializationUtils.serialize(r, SerializationUtils.Mode.GZIPPED_JSON)).of(solution()));
    }

    public static List<NamedFunction<Individual<?, ? extends Robot<?>, ? extends Outcome>, ?>> individualFunctions(Function<Outcome, Double> fitnessFunction) {
        NamedFunction<Individual<?, ? extends Robot<?>, ? extends Outcome>, ?> size = size().of(genotype());
        return List.of(
                f("w", "%2d", (Function<Grid<?>, Number>) Grid::getW)
                        .of(f("shape", (Function<Robot<?>, Grid<?>>) Robot::getVoxels))
                        .of(solution()),
                f("h", "%2d", (Function<Grid<?>, Number>) Grid::getH)
                        .of(f("shape", (Function<Robot<?>, Grid<?>>) Robot::getVoxels))
                        .of(solution()),
                f("num.voxel", "%2d", (Function<Grid<?>, Number>) g -> g.count(Objects::nonNull))
                        .of(f("shape", (Function<Robot<?>, Grid<?>>) Robot::getVoxels))
                        .of(solution()),
                size.reformat("%5d"),
                genotypeBirthIteration(),
                f("fitness", "%5.1f", fitnessFunction).of(fitness()),
                f("shape.static", "%s", (Function<Robot<?>, String>) s -> printBodies(Utils.cropGrid(s.getVoxels(), Objects::nonNull), Objects::nonNull)).of(solution())
                );
    }

    public static List<NamedFunction<Event<?, ? extends Robot<?>, ? extends Outcome>, ?>> populationFunctions(Function<Outcome, Double> fitnessFunction) {
        NamedFunction<Event<?, ? extends Robot<?>, ? extends Outcome>, ?> min = min(Double::compare).of(each(f("fitness", fitnessFunction).of(fitness()))).of(all());
        NamedFunction<Event<?, ? extends Robot<?>, ? extends Outcome>, ?> median = median(Double::compare).of(each(f("fitness", fitnessFunction).of(fitness()))).of(all());
        return List.of(
                size().of(all()),
                size().of(firsts()),
                size().of(lasts()),
                uniqueness().of(each(genotype())).of(all()),
                uniqueness().of(each(solution())).of(all()),
                uniqueness().of(each(fitness())).of(all()),
                min.reformat("%+4.1f"),
                median.reformat("%5.1f")
        );
    }

    public static List<NamedFunction<Outcome, ?>> basicOutcomeFunctions() {
        return List.of(
                f("computation.time", "%4.2f", Outcome::getComputationTime),
                f("distance", "%5.1f", Outcome::getDistance),
                f("velocity", "%5.1f", Outcome::getVelocity),
                f("corrected.efficiency", "%5.2f", Outcome::getCorrectedEfficiency),
                f("area.ratio.power", "%5.1f", Outcome::getAreaRatioPower),
                f("control.power", "%5.1f", Outcome::getControlPower),
                f("shape.dynamic", "%s", o -> Grid.toString(o.getAveragePosture(), (Predicate<Boolean>)  b -> b,"|"))
        );
    }

    public static List<NamedFunction<Outcome, ?>> detailedOutcomeFunctions(double spectrumMinFreq, double spectrumMaxFreq, int spectrumSize) {
        return Misc.concat(List.of(
                NamedFunction.then(cachedF("gait", Outcome::getMainGait), List.of(
                        f("avg.touch.area", "%4.2f", g -> g == null ? null : g.getAvgTouchArea()),
                        f("coverage", "%4.2f", g -> g == null ? null : g.getCoverage()),
                        f("num.footprints", "%2d", g -> g == null ? null : g.getFootprints().size()),
                        f("mode.interval", "%3.1f", g -> g == null ? null : g.getModeInterval()),
                        f("purity", "%4.2f", g -> g == null ? null : g.getPurity()),
                        f("num.unique.footprints", "%2d", g -> g == null ? null : g.getFootprints().stream().distinct().count()),
                        f("footprints", g -> g == null ? null : g.getFootprints().stream().map(Objects::toString).collect(Collectors.joining(",")))
                )),
                NamedFunction.then(cachedF("center.spectrum.x",
                        o -> o.getCenterPowerSpectrum(Outcome.Component.X, spectrumMinFreq, spectrumMaxFreq, spectrumSize).stream()
                                .map(Outcome.Mode::getStrength)
                                .collect(Collectors.toList())),
                        IntStream.range(0, spectrumSize).mapToObj(NamedFunctions::nth).collect(Collectors.toList())
                ),
                NamedFunction.then(cachedF("center.spectrum.y",
                        o -> o.getCenterPowerSpectrum(Outcome.Component.Y, spectrumMinFreq, spectrumMaxFreq, spectrumSize).stream()
                                .map(Outcome.Mode::getStrength)
                                .collect(Collectors.toList())),
                        IntStream.range(0, spectrumSize).mapToObj(NamedFunctions::nth).collect(Collectors.toList())
                )
        ));
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