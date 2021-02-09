import it.units.erallab.hmsrobots.core.objects.Voxel;
import it.units.erallab.hmsrobots.core.sensors.Sensor;
import it.units.erallab.hmsrobots.core.sensors.Sensor.Domain;

public class Constant implements Sensor {
    private final double[] values;
    private final Domain[] domains;

    public Constant(double[] values, Domain[] domains) {
        if (values.length != domains.length) {
            throw new IllegalArgumentException("values and domains lengths are different");
        } else {
            this.values = values;
            this.domains = domains;
        }
    }

    public Domain[] domains() {
        return this.domains;
    }

    public double[] sense(Voxel voxel, double t) {
        return this.values;
    }
}
