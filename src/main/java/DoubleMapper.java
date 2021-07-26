
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

    public DoubleMapper(boolean heterogeneous, int width, int height, List<Sensor> sensors, int[] innerNeurons, int signals) {
        super(heterogeneous, false, width, height, sensors, innerNeurons, signals);
    }

    @Override
    public Robot<?> apply(List<Double> genotype) {
        int nOfInputs = signals * 4 + sensors.stream().mapToInt(s -> s.domains().length).sum();
        int nOfOutputs = signals * 4 + 1;
        int nOfVoxelWeights = MultiLayerPerceptron.countWeights(MultiLayerPerceptron.countNeurons(nOfInputs, innerNeurons, nOfOutputs));

        SensingVoxel sensingVoxel = new SensingVoxel(sensors);

        int c = 0;
        Grid<SensingVoxel> body = Grid.create(width, height);
        // fill voxels only for entries > 0.5
        for (double entry : genotype) {
            if (c < width * height) {
                if (entry > THRESHOLD) {
                    body.set(c % width, c / width, SerializationUtils.clone(sensingVoxel));
                } else {
                    body.set(c % width, c / width, null);
                }
                c = c + 1;
            }
        }

        // create an empty grid
        Grid<SensingVoxel> emptyBody = Grid.create(body.getW(), body.getH(), (x, y) -> null);
        if (body.equals(emptyBody)) {
            body.set(0, 0, SerializationUtils.clone(sensingVoxel)); // if the body is empty put a voxel (0,0)
        }

        // retain largest connected component
        body = Utils.gridLargestConnected(body, Objects::nonNull);

        // create a distributed controller
        DistributedSensing distributedSensing = new DistributedSensing(body, signals);
        for (Grid.Entry<SensingVoxel> entry : body) {
            MultiLayerPerceptron mlp = new MultiLayerPerceptron(
                    MultiLayerPerceptron.ActivationFunction.TANH,
                    nOfInputs,
                    innerNeurons,
                    nOfOutputs
            );

            // create an array of doubles from the list of the genotype
            List<Double> w = genotype.subList(width * height, genotype.size());
            double[] weights = new double[w.size()];
            for (int i = 0; i < weights.length; i++) {
                weights[i] = w.get(i);
            }

            if (heterogeneous) {  // different weights for every voxel
                int from = entry.getX() * nOfVoxelWeights + entry.getY() * width * nOfVoxelWeights;
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

}