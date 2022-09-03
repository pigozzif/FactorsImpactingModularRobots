package it.units.erallab.factors;

import it.units.erallab.hmsrobots.core.controllers.DistributedSensing;
import it.units.erallab.hmsrobots.core.controllers.MultiLayerPerceptron;
import it.units.erallab.hmsrobots.core.objects.Robot;
import it.units.erallab.hmsrobots.core.objects.SensingVoxel;
import it.units.erallab.hmsrobots.core.sensors.Sensor;
import it.units.erallab.hmsrobots.util.Grid;
import it.units.erallab.hmsrobots.util.Utils;
import java.util.Arrays;
import java.util.List;
import java.util.Objects;
import org.apache.commons.lang3.SerializationUtils;


public class DoubleMapper extends RobotMapper {

    public DoubleMapper(boolean heterogeneous, boolean direct, int width, int height, List<Sensor> sensors, int[] innerNeurons, int signals) {
        super(heterogeneous, false, direct, width, height, sensors, innerNeurons, signals);
    }

    @Override
    public Robot<?> apply(List<Double> genotype) {
        int nOfInputs = this.signals * 4 + this.sensors.stream().mapToInt(s -> s.domains().length).sum();
        int nOfOutputs = this.signals * 4 + 1;
        int nOfVoxelWeights = MultiLayerPerceptron.countWeights(MultiLayerPerceptron.countNeurons(nOfInputs, this.innerNeurons, nOfOutputs));
        SensingVoxel sensingVoxel = new SensingVoxel(this.sensors);
        Grid<SensingVoxel> body;
        if (this.direct) {
            body = this.createFromDirect(genotype);
        }
        else {
            body = this.createFromGMM(genotype);
        }
        // create an empty grid
        Grid<SensingVoxel> emptyBody = Grid.create(body.getW(), body.getH(), (x, y) -> null);
        if (body.equals(emptyBody)) {
            body.set(0, 0, SerializationUtils.clone(sensingVoxel)); // if the body is empty put a voxel (0,0)
        }
        // retain largest connected component
        body = Utils.gridLargestConnected(body, Objects::nonNull);
        // create a distributed controller
        DistributedSensing distributedSensing = new DistributedSensing(body, this.signals);
        for (Grid.Entry<SensingVoxel> entry : body) {
            MultiLayerPerceptron mlp = new MultiLayerPerceptron(
                    MultiLayerPerceptron.ActivationFunction.TANH,
                    nOfInputs,
                    this.innerNeurons,
                    nOfOutputs
            );
            // create an array of doubles from the list of the genotype
            List<Double> w = genotype.subList(getMorphologySize(this.direct, this.width, this.height), genotype.size());
            double[] weights = new double[w.size()];
            for (int i = 0; i < weights.length; i++) {
                weights[i] = w.get(i);
            }
            if (this.heterogeneous) {  // different weights for every voxel
                int from = entry.getX() * nOfVoxelWeights + entry.getY() * this.width * nOfVoxelWeights;
                int to = from + nOfVoxelWeights;
                double[] voxelWeights = Arrays.copyOfRange(weights, from, to);
                mlp.setParams(voxelWeights);
            }
            else {
                mlp.setParams(weights);
            }
            distributedSensing.getFunctions().set(entry.getX(), entry.getY(), mlp);
        }
        return new Robot<>(distributedSensing, SerializationUtils.clone(body));
    }

    private Grid<SensingVoxel> createFromDirect(List<Double> genotype) {
        SensingVoxel sensingVoxel = new SensingVoxel(this.sensors);
        int c = 0;
        Grid<SensingVoxel> body = Grid.create(this.width, this.height);
        // fill voxels only for entries > 0.5
        for (double entry : genotype) {
            if (c < this.width * this.height) {
                if (entry > DIRECT_THRESHOLD) {
                    body.set(c % this.width, c / this.width, SerializationUtils.clone(sensingVoxel));
                } else {
                    body.set(c % this.width, c / this.width, null);
                }
                c = c + 1;
            }
        }
        return body;
    }

    private Grid<SensingVoxel> createFromGMM(List<Double> genotype) {
        SensingVoxel sensingVoxel = new SensingVoxel(this.sensors);
        Grid<SensingVoxel> body = Grid.create(this.width, this.height);
        Grid<Double> values = Grid.create(this.width, this.height);
        for (Grid.Entry<Double> entry : values) {
            double value = 0.0;
            for (int i = 0; i < NUM_GAUSSIANS; i++) {
                double weight = genotype.get(5 * i);
                double mx = genotype.get(1 + 5 * i);
                double my = genotype.get(2 + 5 * i);
                double sxx = genotype.get(3 + 5 * i);
                double syy = genotype.get(4 + 5 * i);
                // normalization
                double x = (double) entry.getX() / (double) width;
                double y = (double) entry.getY() / (double) height;
                value += weight * (Math.exp(-0.5 * (Math.pow((x - mx), 2.0) / Math.pow(sxx, 2.0) + Math.pow((y - my), 2.0) / Math.pow(syy, 2.0))) / (2 * Math.PI * sxx * syy));
            }
            values.set(entry.getX(), entry.getY(), value);
        }
        // for each value of values if it is bigger tha a threshold puts a voxel in that position
        for (Grid.Entry<Double> entry : values) {
            if (entry.getValue() > GAUSSIAN_THRESHOLD) {
                body.set(entry.getX(), entry.getY(), SerializationUtils.clone(sensingVoxel));
            } else {
                body.set(entry.getX(), entry.getY(), null);
            }
        }
        return body;
    }

}