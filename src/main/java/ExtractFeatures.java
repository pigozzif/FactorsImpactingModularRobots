import it.units.erallab.hmsrobots.core.objects.Robot;
import it.units.erallab.hmsrobots.tasks.locomotion.Locomotion;
import it.units.erallab.hmsrobots.tasks.locomotion.Outcome;
import it.units.erallab.hmsrobots.util.Grid;
import it.units.erallab.hmsrobots.util.RobotUtils;
import it.units.erallab.hmsrobots.util.SerializationUtils;
import org.dyn4j.dynamics.Settings;

import java.io.*;
import java.util.Objects;
import java.util.Random;
import java.util.function.Predicate;
import java.util.stream.Collectors;


public class ExtractFeatures {

    private static final String oldDir = "./output/";
    private static final String newDir = "./output_new/";
    private static final Locomotion locomotion = new Locomotion(30.0, Locomotion.createTerrain("flat"), new Settings());

    public static void main(String[] args) throws IOException {
        for (File file : Objects.requireNonNull((new File(oldDir)).listFiles())) {
            if (!file.getPath().contains("all")) {
                continue;
            }
            BufferedReader reader = new BufferedReader(new FileReader(file.getPath()));
            createNewFile(reader, file.getPath());
            reader.close();
        }
    }

    private static void createNewFile(BufferedReader reader, String name) throws IOException {
        BufferedWriter writer = new BufferedWriter(new FileWriter(name.replace(oldDir, newDir)));
        writer.write(reader.readLine() + ";shape.dynamic;compressed.frequency\n");
        String line;
        Robot<?> robot;
        Outcome outcome;
        while (true) {
            //try {
                line = reader.readLine();
                if (line == null) {
                    break;
                }
            //}
            //catch (IOException e) {
            //    break;
            //}
            String[] frags = line.split(";");
            robot = RobotUtils.buildRobotTransformation("identity", new Random(0))
                    .apply(SerializationUtils.deserialize(frags[frags.length - 1], Robot.class, SerializationUtils.Mode.valueOf(SerializationUtils.Mode.GZIPPED_JSON.name())));
            outcome = locomotion.apply(robot);
            writer.write(line + ";" + Grid.toString(outcome.getAveragePosture(), (Predicate<Boolean>) b -> b,"|") +
                    ";" + outcome.getCenterPowerSpectrum(Outcome.Component.Y, 0.0, 10.0, 100).stream()
                    .map(Outcome.Mode::getStrength).map(String::valueOf)
                    .collect(Collectors.joining("-")) + "\n");
        }
        writer.close();
    }

}
