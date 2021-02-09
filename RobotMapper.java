import it.units.erallab.hmsrobots.core.controllers.MultiLayerPerceptron;
import it.units.erallab.hmsrobots.core.objects.Robot;
import it.units.erallab.hmsrobots.core.sensors.*;

import java.util.ArrayList;
import java.util.List;
import java.util.function.Function;

public abstract class RobotMapper implements Function<List<Double>, Robot<?>> {
    protected final boolean heterogeneous;
    protected final boolean hasPositionSensor;
    protected final int width;
    protected final int height;
    protected final List<Sensor> sensors;
    protected final int[] innerNeurons;
    protected final int signals;
    protected static final double THRESHOLD = 0.0D;

    public RobotMapper(boolean heterogeneous, boolean position, int width, int height, List<Sensor> sensors, int[] innerNeurons, int signals) {
        this.heterogeneous = heterogeneous;
        this.hasPositionSensor = position;
        this.width = width;
        this.height = height;
        this.sensors = sensors;
        this.innerNeurons = innerNeurons;
        this.signals = signals;
    }

    public static int getGenotypeSize(boolean isHeterogeneous, boolean isPosition, List<Sensor> sensors, int[] inner, int signals, int w, int h) {
        int nOfInputs = signals * 4 + sensors.stream().mapToInt((s) -> s.domains().length).sum() + (isPosition ? 2 : 0);
        int nOfOutputs = signals * 4 + 1;
        int nOfWeights = MultiLayerPerceptron.countWeights(MultiLayerPerceptron.countNeurons(nOfInputs, inner, nOfOutputs));
        return isHeterogeneous ? w * h + nOfWeights * w * h : w * h + nOfWeights;
    }

    public int getGenotypeSize() {
        return getGenotypeSize(heterogeneous, hasPositionSensor, sensors, innerNeurons, signals, width, height);
    }

    public static List<Sensor> getSensors(String sensorConfig) {
        List<Sensor> sensors = new ArrayList<>() {{
                add(new Normalization(new Velocity(true, 5d, Velocity.Axis.X, Velocity.Axis.Y)));
                add(new Normalization(new AreaRatio())); }};
        if (sensorConfig.contains("touch")) {
            sensors.add(new Normalization(new Average(new Touch(), 0.5D)));
        }
        return sensors;
    }

    public static RobotMapper createMapper(String controller, int width, int height, List<Sensor> sensors, int[] innerNeurons, int signals) {
        return ("position".equals(controller) ? new DoublePositionMapper(width, height, sensors, innerNeurons, signals) : new DoubleMapper(controller.equals("heterogeneous"), width, height, sensors, innerNeurons, signals));
    }

    public static Robot<?> createMapperAndApplyFromSerialized(List<Double> serialized, int width, int height, List<Sensor> sensors, int[] innerNeurons, int signals) {
        if (serialized.size() == getGenotypeSize(true, false, sensors, innerNeurons, signals, width, height)) {
            return (new DoubleMapper(true, width, height, sensors, innerNeurons, signals)).apply(serialized);
        }
        else if (serialized.size() == getGenotypeSize(false, false, sensors, innerNeurons, signals, width, height)) {
            return (new DoubleMapper(false, width, height, sensors, innerNeurons, signals)).apply(serialized);
        }
        //System.out.println("here");
        return (new DoublePositionMapper(width, height, sensors, innerNeurons, signals)).apply(serialized);
    }

}
