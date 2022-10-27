package frc.robot.subsystems.swerve.module;

import edu.wpi.first.math.geometry.Translation2d;
import frc.robot.PIDFFGains;
import lombok.Builder;
import lombok.Getter;
import lombok.NonNull;

@Builder
public class ModuleConfig {
    @Getter
    @NonNull
    private String name;
    @Getter
    @NonNull
    private Integer driveMotorCANId;
    @Getter
    @NonNull
    private Integer azimuthMotorCANid;

    @Getter
    @NonNull
    private PIDFFGains azimuthGains;

    @Getter
    @NonNull
    private PIDFFGains drivingGains;

    @Getter
    @NonNull
    private Double encoderOffset;
    @Getter
    @NonNull
    private Translation2d location;
}
