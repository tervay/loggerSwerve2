package frc.robot.subsystems.swerve.module;

import lombok.Builder;
import lombok.Getter;
import lombok.NonNull;

@Builder
public class ModuleContainer {
    @Getter
    @NonNull
    private Module frontLeft;

    @Getter
    @NonNull
    private Module frontRight;

    @Getter
    @NonNull
    private Module backLeft;

    @Getter
    @NonNull
    private Module backRight;
}
