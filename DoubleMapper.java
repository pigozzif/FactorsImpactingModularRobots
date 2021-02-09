
import it.units.erallab.hmsrobots.core.controllers.DistributedSensing;
import it.units.erallab.hmsrobots.core.controllers.DistributedSensing;
import it.units.erallab.hmsrobots.core.controllers.MultiLayerPerceptron;
import it.units.erallab.hmsrobots.core.controllers.MultiLayerPerceptron.ActivationFunction;
import it.units.erallab.hmsrobots.core.objects.Robot;
import it.units.erallab.hmsrobots.core.objects.SensingVoxel;
import it.units.erallab.hmsrobots.core.sensors.Sensor;
import it.units.erallab.hmsrobots.util.Grid;
import it.units.erallab.hmsrobots.util.Utils;
import it.units.erallab.hmsrobots.util.Grid.Entry;
import java.util.Arrays;
import java.util.Iterator;
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
        if (false && genotype.size() != getGenotypeSize()) { // correct length of the genotype
            throw new IllegalArgumentException("Sensor list has wrong dimension");
        }

        SensingVoxel sensingVoxel = new SensingVoxel(sensors);

        int c = 0;
        Grid<SensingVoxel> body = Grid.create(width, height);
        for (double entry : genotype) {
            if (c < width * height) {  // < or <= ? < is correct
                if (entry > THRESHOLD) {
                    body.set(c % width, c / width, SerializationUtils.clone(sensingVoxel));
                } else {
                    body.set(c % width, c / width, null);
                }
                c = c + 1;
            }
        }

        // creates an empty grid
        Grid<SensingVoxel> emptyBody = Grid.create(body.getW(), body.getH(), (x, y) -> null);
        // checks if the robot is empty
        if (body.equals(emptyBody)) {
            body.set(0, 0, SerializationUtils.clone(sensingVoxel)); // if the body is empty puts a voxel (0,0)
        }

        // checks if the robot is connected
        body = Utils.gridLargestConnected(body, Objects::nonNull);

        // creates a distributed controller
        DistributedSensing distributedSensing = new DistributedSensing(body, signals);
        for (Grid.Entry<SensingVoxel> entry : body) {
            MultiLayerPerceptron mlp = new MultiLayerPerceptron(
                    MultiLayerPerceptron.ActivationFunction.TANH,
                    nOfInputs,
                    innerNeurons,
                    nOfOutputs
            );

            // creates an array of doubles from the list of the genotype
            List<Double> w = genotype.subList(width * height, genotype.size());
            double[] weights = new double[w.size()];
            for (int i = 0; i < weights.length; i++) {
                weights[i] = w.get(i);
            }

            if (heterogeneous) {
                int from = entry.getX() * nOfVoxelWeights + entry.getY() * width * nOfVoxelWeights; //  + width * height is not needed because i remove it before
                int to = from + nOfVoxelWeights;
                double[] voxelWeights = Arrays.copyOfRange(weights, from, to);
        /*
        System.out.println("parto da: " + from);
        System.out.println("fino a: " + to);
        for (int i = 0; i < voxelWeights.length; i++) {
          System.out.println("voxelWeights:[" + i + "] " + voxelWeights[i]);
        }
         */
                mlp.setParams(voxelWeights);
            } else {
                // i have to assign the correct subset of weights to this
                mlp.setParams(weights);
            }
            distributedSensing.getFunctions().set(entry.getX(), entry.getY(), mlp);
        }

        return new Robot<>(
                distributedSensing,
                body //SerializationUtils.clone(body) i think it is better not to copy this
        );
        /*int nOfInputs = this.signals * 4 + this.sensors.stream().mapToInt((s) -> {
            return s.domains().length;
        }).sum();
        int nOfOutputs = this.signals * 4 + 1;
        int nOfVoxelWeights = MultiLayerPerceptron.countWeights(MultiLayerPerceptron.countNeurons(nOfInputs, this.innerNeurons, nOfOutputs));
        if (falsegenotype.size() != this.getGenotypeSize()) {
            System.out.println(genotype.size() + " " + getGenotypeSize());
            throw new IllegalArgumentException("Sensor list has wrong dimension");
        } else {
            SensingVoxel sensingVoxel = new SensingVoxel(this.sensors);
            int c = 0;
            Grid<SensingVoxel> body = Grid.create(this.width, this.height);
            Iterator var8 = genotype.iterator();

            while(var8.hasNext()) {
                double entry = (Double)var8.next();
                if (c < this.width * this.height) {
                    if (entry > 0.0D) {
                        body.set(c % this.width, c / this.width, (SensingVoxel)SerializationUtils.clone(sensingVoxel));
                    } else {
                        body.set(c % this.width, c / this.width, (SensingVoxel) null);
                    }

                    ++c;
                }
            }

            Grid<SensingVoxel> emptyBody = Grid.create(body.getW(), body.getH(), (x, y) -> {
                return null;
            });
            if (body.equals(emptyBody)) {
                body.set(0, 0, (SensingVoxel)SerializationUtils.clone(sensingVoxel));
            }

            body = Utils.gridLargestConnected(body, Objects::nonNull);
            DistributedSensing distributedSensing = new DistributedSensing(body, this.signals);

            Entry entry;
            MultiLayerPerceptron mlp;
            for(Iterator var10 = body.iterator(); var10.hasNext(); distributedSensing.getFunctions().set(entry.getX(), entry.getY(), mlp)) {
                entry = (Entry)var10.next();
                mlp = new MultiLayerPerceptron(ActivationFunction.TANH, nOfInputs, this.innerNeurons, nOfOutputs);
                List<Double> w = genotype.subList(this.width * this.height, genotype.size());
                double[] weights = new double[w.size()];

                int from;
                for(from = 0; from < weights.length; ++from) {
                    weights[from] = (Double)w.get(from);
                }

                if (this.heterogeneous) {
                    from = entry.getX() * nOfVoxelWeights + entry.getY() * this.width * nOfVoxelWeights;
                    int to = from + nOfVoxelWeights;
                    double[] voxelWeights = Arrays.copyOfRange(weights, from, to);
                    mlp.setParams(voxelWeights);
                } else {
                    mlp.setParams(weights);
                }
            }

            return new Robot(distributedSensing, body);
        }*/
    }
}