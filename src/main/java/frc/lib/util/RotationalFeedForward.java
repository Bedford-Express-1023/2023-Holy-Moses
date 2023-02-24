package frc.lib.util;

import edu.wpi.first.math.controller.SimpleMotorFeedforward;

public class RotationalFeedForward extends SimpleMotorFeedforward {
    public double kg;
    public RotationalFeedForward(double ks, double kv, double ka, double kg) {
        super(ks, kv, ka);
        this.kg = kg;
    }

    @Override
    public double calculate(double velocity, double acceleration, double angle) {
        return ks * Math.signum(velocity) + kv * velocity + ka * acceleration + kg * -Math.cos(angle * Math.PI/180);
    }

}