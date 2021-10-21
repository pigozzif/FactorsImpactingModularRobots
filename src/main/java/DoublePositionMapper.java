
import it.units.erallab.hmsrobots.core.controllers.DistributedSensing;
import it.units.erallab.hmsrobots.core.controllers.MultiLayerPerceptron;
import it.units.erallab.hmsrobots.core.controllers.MultiLayerPerceptron.ActivationFunction;
import it.units.erallab.hmsrobots.core.objects.Robot;
import it.units.erallab.hmsrobots.core.objects.SensingVoxel;
import it.units.erallab.hmsrobots.core.sensors.Sensor;
import it.units.erallab.hmsrobots.core.sensors.Sensor.Domain;
import it.units.erallab.hmsrobots.util.Grid;
import it.units.erallab.hmsrobots.util.Utils;
import it.units.erallab.hmsrobots.util.Grid.Entry;
import java.util.Iterator;
import java.util.List;
import java.util.Objects;
import java.util.stream.Collectors;
import java.util.stream.Stream;
import org.apache.commons.lang3.SerializationUtils;


public class DoublePositionMapper extends RobotMapper {

    public DoublePositionMapper(int width, int height, List<Sensor> sensors, int[] innerNeurons, int signals) {
        super(false, true, true, width, height, sensors, innerNeurons, signals);
    }

    public Robot<?> apply(List<Double> genotype) {
        int nOfInputs = this.signals * 4 + this.sensors.stream().mapToInt((s) -> s.domains().length).sum() + 2;
        int nOfOutputs = this.signals * 4 + 1;
        if (genotype.size() != this.getGenotypeSize()) {
            throw new IllegalArgumentException("Sensor list has wrong dimension");
        } else {
            int c = 0;
            Grid<Boolean> shape = Grid.create(this.width, this.height);
            Iterator var6 = genotype.iterator();

            while(var6.hasNext()) {
                double entry = (Double)var6.next();
                if (c < this.width * this.height) {
                    if (entry > 0.0D) {
                        shape.set(c % this.width, c / this.width, true);
                    } else {
                        shape.set(c % this.width, c / this.width, false);
                    }

                    ++c;
                }
            }

            if (shape.values().stream().noneMatch((b) -> {
                return b;
            })) {
                shape = Grid.create(1, 1, true);
            }

            Grid<SensingVoxel> body = Grid.create(this.width, this.height, (x, y) -> {
                return new SensingVoxel((List)Stream.concat(this.sensors.stream().map(SerializationUtils::clone), List.of(new Constant(new double[]{(double)x / (double)this.width, (double)y / (double)this.height}, new Domain[]{Domain.of(0.0D, 1.0D), Domain.of(0.0D, 1.0D)})).stream()).collect(Collectors.toList()));
            });
            Iterator var15 = shape.iterator();

            while(var15.hasNext()) {
                Entry<Boolean> entry = (Entry)var15.next();
                if (!(Boolean)entry.getValue()) {
                    body.set(entry.getX(), entry.getY(), (SensingVoxel) null);
                }
            }

            body = Utils.gridLargestConnected(body, Objects::nonNull);
            DistributedSensing distributedSensing = new DistributedSensing(body, this.signals);
            Iterator var17 = body.iterator();

            while(var17.hasNext()) {
                Entry<SensingVoxel> entry = (Entry)var17.next();
                MultiLayerPerceptron mlp = new MultiLayerPerceptron(ActivationFunction.TANH, nOfInputs, this.innerNeurons, nOfOutputs);
                List<Double> w = genotype.subList(this.width * this.height, genotype.size());
                double[] weights = new double[w.size()];

                for(int i = 0; i < weights.length; ++i) {
                    weights[i] = (Double)w.get(i);
                }

                mlp.setParams(weights);
                distributedSensing.getFunctions().set(entry.getX(), entry.getY(), mlp);
            }

            return new Robot<>(distributedSensing, SerializationUtils.clone(body));
        }
    }

}
