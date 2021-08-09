
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

import java.io.BufferedWriter;
import java.io.FileWriter;
import java.io.IOException;
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

    public static List<NamedFunction<Event<?, ? extends Robot<?>, ? extends Outcome>, ?>> lastPopulationFunctions() {
        return List.of(
                f("all", "%s", Function.identity()).of(each(AuxUtils::getLinePerLastIndividual)).of(all()));
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

    private static String getLinePerLastIndividual(Individual<?, ? extends Robot<?>, ? extends Outcome> individual) {
        List<String> toWrite = new ArrayList<>();
        toWrite.add(printBodies(individual.getSolution().getVoxels(), Objects::nonNull));
        toWrite.add(Grid.toString(individual.getFitness().getAveragePosture(), (Predicate<Boolean>)  b -> b,"|"));
        toWrite.add(String.valueOf(individual.getFitness().getDistance()));
        toWrite.addAll(individual.getFitness().getCenterPowerSpectrum(Outcome.Component.X, 0.0, 10.0D, 100).stream()
                .map(Outcome.Mode::getStrength).map(String::valueOf)
                .collect(Collectors.toList()));
        toWrite.addAll(individual.getFitness().getCenterPowerSpectrum(Outcome.Component.Y, 0.0, 10.0D, 100).stream()
                .map(Outcome.Mode::getStrength).map(String::valueOf)
                .collect(Collectors.toList()));
        return String.join(";", toWrite) + "\n";
    }

    public static void saveLastPopulation(Collection<Robot<?>> solutions, String lastFileName, Function<Robot<?>, Outcome> task, double spectrumMinFreq, double spectrumMaxFreq, int spectrumSize) throws IOException {
        BufferedWriter writer = new BufferedWriter(new FileWriter(lastFileName));
        writer.write("best→solution→shape.static;best→fitness→shape.dynamic;best→fitness→fitness;best→fitness→as[Outcome]→center.spectrum.x→[0];best→fitness→as[Outcome]→center.spectrum.x→[1];best→fitness→as[Outcome]→center.spectrum.x→[2];best→fitness→as[Outcome]→center.spectrum.x→[3];best→fitness→as[Outcome]→center.spectrum.x→[4];best→fitness→as[Outcome]→center.spectrum.x→[5];best→fitness→as[Outcome]→center.spectrum.x→[6];best→fitness→as[Outcome]→center.spectrum.x→[7];best→fitness→as[Outcome]→center.spectrum.x→[8];best→fitness→as[Outcome]→center.spectrum.x→[9];best→fitness→as[Outcome]→center.spectrum.x→[10];best→fitness→as[Outcome]→center.spectrum.x→[11];best→fitness→as[Outcome]→center.spectrum.x→[12];best→fitness→as[Outcome]→center.spectrum.x→[13];best→fitness→as[Outcome]→center.spectrum.x→[14];best→fitness→as[Outcome]→center.spectrum.x→[15];best→fitness→as[Outcome]→center.spectrum.x→[16];best→fitness→as[Outcome]→center.spectrum.x→[17];best→fitness→as[Outcome]→center.spectrum.x→[18];best→fitness→as[Outcome]→center.spectrum.x→[19];best→fitness→as[Outcome]→center.spectrum.x→[20];best→fitness→as[Outcome]→center.spectrum.x→[21];best→fitness→as[Outcome]→center.spectrum.x→[22];best→fitness→as[Outcome]→center.spectrum.x→[23];best→fitness→as[Outcome]→center.spectrum.x→[24];best→fitness→as[Outcome]→center.spectrum.x→[25];best→fitness→as[Outcome]→center.spectrum.x→[26];best→fitness→as[Outcome]→center.spectrum.x→[27];best→fitness→as[Outcome]→center.spectrum.x→[28];best→fitness→as[Outcome]→center.spectrum.x→[29];best→fitness→as[Outcome]→center.spectrum.x→[30];best→fitness→as[Outcome]→center.spectrum.x→[31];best→fitness→as[Outcome]→center.spectrum.x→[32];best→fitness→as[Outcome]→center.spectrum.x→[33];best→fitness→as[Outcome]→center.spectrum.x→[34];best→fitness→as[Outcome]→center.spectrum.x→[35];best→fitness→as[Outcome]→center.spectrum.x→[36];best→fitness→as[Outcome]→center.spectrum.x→[37];best→fitness→as[Outcome]→center.spectrum.x→[38];best→fitness→as[Outcome]→center.spectrum.x→[39];best→fitness→as[Outcome]→center.spectrum.x→[40];best→fitness→as[Outcome]→center.spectrum.x→[41];best→fitness→as[Outcome]→center.spectrum.x→[42];best→fitness→as[Outcome]→center.spectrum.x→[43];best→fitness→as[Outcome]→center.spectrum.x→[44];best→fitness→as[Outcome]→center.spectrum.x→[45];best→fitness→as[Outcome]→center.spectrum.x→[46];best→fitness→as[Outcome]→center.spectrum.x→[47];best→fitness→as[Outcome]→center.spectrum.x→[48];best→fitness→as[Outcome]→center.spectrum.x→[49];best→fitness→as[Outcome]→center.spectrum.x→[50];best→fitness→as[Outcome]→center.spectrum.x→[51];best→fitness→as[Outcome]→center.spectrum.x→[52];best→fitness→as[Outcome]→center.spectrum.x→[53];best→fitness→as[Outcome]→center.spectrum.x→[54];best→fitness→as[Outcome]→center.spectrum.x→[55];best→fitness→as[Outcome]→center.spectrum.x→[56];best→fitness→as[Outcome]→center.spectrum.x→[57];best→fitness→as[Outcome]→center.spectrum.x→[58];best→fitness→as[Outcome]→center.spectrum.x→[59];best→fitness→as[Outcome]→center.spectrum.x→[60];best→fitness→as[Outcome]→center.spectrum.x→[61];best→fitness→as[Outcome]→center.spectrum.x→[62];best→fitness→as[Outcome]→center.spectrum.x→[63];best→fitness→as[Outcome]→center.spectrum.x→[64];best→fitness→as[Outcome]→center.spectrum.x→[65];best→fitness→as[Outcome]→center.spectrum.x→[66];best→fitness→as[Outcome]→center.spectrum.x→[67];best→fitness→as[Outcome]→center.spectrum.x→[68];best→fitness→as[Outcome]→center.spectrum.x→[69];best→fitness→as[Outcome]→center.spectrum.x→[70];best→fitness→as[Outcome]→center.spectrum.x→[71];best→fitness→as[Outcome]→center.spectrum.x→[72];best→fitness→as[Outcome]→center.spectrum.x→[73];best→fitness→as[Outcome]→center.spectrum.x→[74];best→fitness→as[Outcome]→center.spectrum.x→[75];best→fitness→as[Outcome]→center.spectrum.x→[76];best→fitness→as[Outcome]→center.spectrum.x→[77];best→fitness→as[Outcome]→center.spectrum.x→[78];best→fitness→as[Outcome]→center.spectrum.x→[79];best→fitness→as[Outcome]→center.spectrum.x→[80];best→fitness→as[Outcome]→center.spectrum.x→[81];best→fitness→as[Outcome]→center.spectrum.x→[82];best→fitness→as[Outcome]→center.spectrum.x→[83];best→fitness→as[Outcome]→center.spectrum.x→[84];best→fitness→as[Outcome]→center.spectrum.x→[85];best→fitness→as[Outcome]→center.spectrum.x→[86];best→fitness→as[Outcome]→center.spectrum.x→[87];best→fitness→as[Outcome]→center.spectrum.x→[88];best→fitness→as[Outcome]→center.spectrum.x→[89];best→fitness→as[Outcome]→center.spectrum.x→[90];best→fitness→as[Outcome]→center.spectrum.x→[91];best→fitness→as[Outcome]→center.spectrum.x→[92];best→fitness→as[Outcome]→center.spectrum.x→[93];best→fitness→as[Outcome]→center.spectrum.x→[94];best→fitness→as[Outcome]→center.spectrum.x→[95];best→fitness→as[Outcome]→center.spectrum.x→[96];best→fitness→as[Outcome]→center.spectrum.x→[97];best→fitness→as[Outcome]→center.spectrum.x→[98];best→fitness→as[Outcome]→center.spectrum.x→[99];best→fitness→as[Outcome]→center.spectrum.y→[0];best→fitness→as[Outcome]→center.spectrum.y→[1];best→fitness→as[Outcome]→center.spectrum.y→[2];best→fitness→as[Outcome]→center.spectrum.y→[3];best→fitness→as[Outcome]→center.spectrum.y→[4];best→fitness→as[Outcome]→center.spectrum.y→[5];best→fitness→as[Outcome]→center.spectrum.y→[6];best→fitness→as[Outcome]→center.spectrum.y→[7];best→fitness→as[Outcome]→center.spectrum.y→[8];best→fitness→as[Outcome]→center.spectrum.y→[9];best→fitness→as[Outcome]→center.spectrum.y→[10];best→fitness→as[Outcome]→center.spectrum.y→[11];best→fitness→as[Outcome]→center.spectrum.y→[12];best→fitness→as[Outcome]→center.spectrum.y→[13];best→fitness→as[Outcome]→center.spectrum.y→[14];best→fitness→as[Outcome]→center.spectrum.y→[15];best→fitness→as[Outcome]→center.spectrum.y→[16];best→fitness→as[Outcome]→center.spectrum.y→[17];best→fitness→as[Outcome]→center.spectrum.y→[18];best→fitness→as[Outcome]→center.spectrum.y→[19];best→fitness→as[Outcome]→center.spectrum.y→[20];best→fitness→as[Outcome]→center.spectrum.y→[21];best→fitness→as[Outcome]→center.spectrum.y→[22];best→fitness→as[Outcome]→center.spectrum.y→[23];best→fitness→as[Outcome]→center.spectrum.y→[24];best→fitness→as[Outcome]→center.spectrum.y→[25];best→fitness→as[Outcome]→center.spectrum.y→[26];best→fitness→as[Outcome]→center.spectrum.y→[27];best→fitness→as[Outcome]→center.spectrum.y→[28];best→fitness→as[Outcome]→center.spectrum.y→[29];best→fitness→as[Outcome]→center.spectrum.y→[30];best→fitness→as[Outcome]→center.spectrum.y→[31];best→fitness→as[Outcome]→center.spectrum.y→[32];best→fitness→as[Outcome]→center.spectrum.y→[33];best→fitness→as[Outcome]→center.spectrum.y→[34];best→fitness→as[Outcome]→center.spectrum.y→[35];best→fitness→as[Outcome]→center.spectrum.y→[36];best→fitness→as[Outcome]→center.spectrum.y→[37];best→fitness→as[Outcome]→center.spectrum.y→[38];best→fitness→as[Outcome]→center.spectrum.y→[39];best→fitness→as[Outcome]→center.spectrum.y→[40];best→fitness→as[Outcome]→center.spectrum.y→[41];best→fitness→as[Outcome]→center.spectrum.y→[42];best→fitness→as[Outcome]→center.spectrum.y→[43];best→fitness→as[Outcome]→center.spectrum.y→[44];best→fitness→as[Outcome]→center.spectrum.y→[45];best→fitness→as[Outcome]→center.spectrum.y→[46];best→fitness→as[Outcome]→center.spectrum.y→[47];best→fitness→as[Outcome]→center.spectrum.y→[48];best→fitness→as[Outcome]→center.spectrum.y→[49];best→fitness→as[Outcome]→center.spectrum.y→[50];best→fitness→as[Outcome]→center.spectrum.y→[51];best→fitness→as[Outcome]→center.spectrum.y→[52];best→fitness→as[Outcome]→center.spectrum.y→[53];best→fitness→as[Outcome]→center.spectrum.y→[54];best→fitness→as[Outcome]→center.spectrum.y→[55];best→fitness→as[Outcome]→center.spectrum.y→[56];best→fitness→as[Outcome]→center.spectrum.y→[57];best→fitness→as[Outcome]→center.spectrum.y→[58];best→fitness→as[Outcome]→center.spectrum.y→[59];best→fitness→as[Outcome]→center.spectrum.y→[60];best→fitness→as[Outcome]→center.spectrum.y→[61];best→fitness→as[Outcome]→center.spectrum.y→[62];best→fitness→as[Outcome]→center.spectrum.y→[63];best→fitness→as[Outcome]→center.spectrum.y→[64];best→fitness→as[Outcome]→center.spectrum.y→[65];best→fitness→as[Outcome]→center.spectrum.y→[66];best→fitness→as[Outcome]→center.spectrum.y→[67];best→fitness→as[Outcome]→center.spectrum.y→[68];best→fitness→as[Outcome]→center.spectrum.y→[69];best→fitness→as[Outcome]→center.spectrum.y→[70];best→fitness→as[Outcome]→center.spectrum.y→[71];best→fitness→as[Outcome]→center.spectrum.y→[72];best→fitness→as[Outcome]→center.spectrum.y→[73];best→fitness→as[Outcome]→center.spectrum.y→[74];best→fitness→as[Outcome]→center.spectrum.y→[75];best→fitness→as[Outcome]→center.spectrum.y→[76];best→fitness→as[Outcome]→center.spectrum.y→[77];best→fitness→as[Outcome]→center.spectrum.y→[78];best→fitness→as[Outcome]→center.spectrum.y→[79];best→fitness→as[Outcome]→center.spectrum.y→[80];best→fitness→as[Outcome]→center.spectrum.y→[81];best→fitness→as[Outcome]→center.spectrum.y→[82];best→fitness→as[Outcome]→center.spectrum.y→[83];best→fitness→as[Outcome]→center.spectrum.y→[84];best→fitness→as[Outcome]→center.spectrum.y→[85];best→fitness→as[Outcome]→center.spectrum.y→[86];best→fitness→as[Outcome]→center.spectrum.y→[87];best→fitness→as[Outcome]→center.spectrum.y→[88];best→fitness→as[Outcome]→center.spectrum.y→[89];best→fitness→as[Outcome]→center.spectrum.y→[90];best→fitness→as[Outcome]→center.spectrum.y→[91];best→fitness→as[Outcome]→center.spectrum.y→[92];best→fitness→as[Outcome]→center.spectrum.y→[93];best→fitness→as[Outcome]→center.spectrum.y→[94];best→fitness→as[Outcome]→center.spectrum.y→[95];best→fitness→as[Outcome]→center.spectrum.y→[96];best→fitness→as[Outcome]→center.spectrum.y→[97];best→fitness→as[Outcome]→center.spectrum.y→[98];best→fitness→as[Outcome]→center.spectrum.y→[99];best→solution→serialized\n");
        List<String> toWrite = new ArrayList<>();
        for (Robot<?> sol : solutions) {
            Outcome o = task.apply(sol);
            toWrite.add(printBodies(sol.getVoxels(), Objects::nonNull));
            toWrite.add(Grid.toString(o.getAveragePosture(), (Predicate<Boolean>)  b -> b,"|"));
            toWrite.add(String.valueOf(o.getDistance()));
            toWrite.addAll(o.getCenterPowerSpectrum(Outcome.Component.X, spectrumMinFreq, spectrumMaxFreq, spectrumSize).stream()
                    .map(Outcome.Mode::getStrength).map(String::valueOf)
                    .collect(Collectors.toList()));
            toWrite.addAll(o.getCenterPowerSpectrum(Outcome.Component.Y, spectrumMinFreq, spectrumMaxFreq, spectrumSize).stream()
                    .map(Outcome.Mode::getStrength).map(String::valueOf)
                    .collect(Collectors.toList()));
            writer.write("");
            toWrite.clear();
        }
        writer.close();
    }

}