package frc.robot.subsystems.swerve.module;

import java.util.List;

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

    public List<Module> asList() {
        return List.of(frontLeft, frontRight, backLeft, backRight);
    }
}
