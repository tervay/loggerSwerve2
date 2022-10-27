package frc.robot;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;

public class PIDFFController extends PIDController {
    private PIDFFGains gains;
    private SimpleMotorFeedforward feedforward;

    public PIDFFController(PIDFFGains gains) {
        super(gains.kP.get(), gains.kI.get(), gains.kD.get());

        this.gains = gains;
        feedforward = gains.createWpilibFeedforward();
    }

    @Override
    public double calculate(double measurement, double setpoint) {
        setSetpoint(setpoint);
        return calculate(measurement);
    }

    @Override
    public double calculate(double measurement) {
        if (Constants.tuningMode) {
            if (gains.hasChanged(this, feedforward)) {
                System.out.println("New Gains changed. Setting kP to " + gains.kP.get());
                setP(gains.kP.get());
                setI(gains.kI.get());
                setD(gains.kD.get());
                feedforward = gains.createWpilibFeedforward();

                System.out.println(getP());
                System.out.println(gains.kP.get());
            }

        }

        return super.calculate(measurement) + feedforward.calculate(getSetpoint());
    }
}
