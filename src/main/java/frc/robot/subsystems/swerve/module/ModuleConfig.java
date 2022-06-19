package frc.robot.subsystems.swerve.module;

import edu.wpi.first.math.geometry.Translation2d;
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
    private Double drivingKS;
    @Getter
    @NonNull
    private Double drivingKV;
    @Getter
    @NonNull
    private Double drivingKP;
    @Getter
    @NonNull
    private Double azimuthKP;
    @Getter
    @NonNull
    private Double encoderOffset;
    @Getter
    @NonNull
    private Translation2d location;
}
