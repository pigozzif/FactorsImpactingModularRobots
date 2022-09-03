package it.units.erallab.factors;

import it.units.erallab.hmsrobots.core.controllers.MultiLayerPerceptron;
import it.units.erallab.hmsrobots.core.objects.Robot;
import it.units.erallab.hmsrobots.core.sensors.*;

import java.util.ArrayList;
import java.util.List;
import java.util.function.Function;


public abstract class RobotMapper implements Function<List<Double>, Robot<?>> {

    protected final boolean heterogeneous;
    protected final boolean hasPositionSensor;
    protected final boolean direct;
    protected final int width;
    protected final int height;
    protected final List<Sensor> sensors;
    protected final int[] innerNeurons;
    protected final int signals;
    protected static final double DIRECT_THRESHOLD = 0.0D;
    protected static final double GAUSSIAN_THRESHOLD = 0.5D;
    protected static final int NUM_GAUSSIANS = 5;

    public RobotMapper(boolean heterogeneous, boolean position, boolean direct, int width, int height, List<Sensor> sensors, int[] innerNeurons, int signals) {
        this.heterogeneous = heterogeneous;
        this.hasPositionSensor = position;
        this.direct = direct;
        this.width = width;
        this.height = height;
        this.sensors = sensors;
        this.innerNeurons = innerNeurons;
        this.signals = signals;
    }

    public static int getMorphologySize(boolean direct, int w, int h) {
        return (direct) ? w * h : NUM_GAUSSIANS * 5;
    }

    public static int getGenotypeSize(boolean isHeterogeneous, boolean isPosition, boolean direct, List<Sensor> sensors, int[] inner, int signals, int w, int h) {
        int morphologySize = getMorphologySize(direct, w, h);
        int nOfInputs = signals * 4 + sensors.stream().mapToInt((s) -> s.domains().length).sum() + (isPosition ? 2 : 0);
        int nOfOutputs = signals * 4 + 1;
        int nOfWeights = MultiLayerPerceptron.countWeights(MultiLayerPerceptron.countNeurons(nOfInputs, inner, nOfOutputs));
        return isHeterogeneous ? morphologySize + nOfWeights * w * h : morphologySize + nOfWeights;
    }

    public int getGenotypeSize() {
        return getGenotypeSize(this.heterogeneous, this.hasPositionSensor, this.direct, this.sensors, this.innerNeurons, this.signals, this.width, this.height);
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

    public static RobotMapper createMapper(String representation, int width, int height, List<Sensor> sensors, int[] innerNeurons, int signals) {
        String controller = representation.split("-")[1];
        String morphology = representation.split("-")[0];
        return new DoubleMapper(controller.equals("heterogeneous"), morphology.equals("direct"), width, height, sensors, innerNeurons, signals);
    }

    public static Robot<?> createMapperAndApplyFromSerialized(List<Double> serialized, int width, int height, List<Sensor> sensors, int[] innerNeurons, int signals) {
        if (serialized.size() == getGenotypeSize(true, false, true, sensors, innerNeurons, signals, width, height)) {
            return (new DoubleMapper(true, true, width, height, sensors, innerNeurons, signals)).apply(serialized);
        }
        else if (serialized.size() == getGenotypeSize(false, false, true, sensors, innerNeurons, signals, width, height)) {
            return (new DoubleMapper(false, true, width, height, sensors, innerNeurons, signals)).apply(serialized);
        }
        else if (serialized.size() == getGenotypeSize(true, false, false, sensors, innerNeurons, signals, width, height)) {
            return (new DoubleMapper(true, false, width, height, sensors, innerNeurons, signals)).apply(serialized);
        }
        else if (serialized.size() == getGenotypeSize(false, false, false, sensors, innerNeurons, signals, width, height)) {
            return (new DoubleMapper(false, false, width, height, sensors, innerNeurons, signals)).apply(serialized);
        }
        else {
            throw new IllegalArgumentException(String.format("Unknown serialized size: %d", serialized.size()));
        }
    }

}
