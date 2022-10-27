package frc.robot.subsystems.swerve;

import edu.wpi.first.math.geometry.Rotation2d;
import frc.robot.PIDFFController;
import frc.robot.PIDFFGains;
import frc.robot.Robot;

public class HeadingController {
    private static HeadingController instance;
    private Rotation2d setpoint;
    private PIDFFController controller;

    private HeadingController() {
        controller = new PIDFFController(PIDFFGains.builder("Heading").build());
        controller.enableContinuousInput(-180, 180);

        setpoint = Robot.swerve.getPose().getRotation();
    }

    public static HeadingController getInstance() {
        if (instance == null) {
            instance = new HeadingController();
        }

        return instance;
    }

    public double getRotationalSpeedDegPerSec() {
        double output = 0;
        if (!controller.atSetpoint()) {
            output = controller.calculate(
                Robot.swerve.getPose().getRotation().getDegrees(),
                setpoint.getDegrees()
            );
        }

        return output;
    }
}
