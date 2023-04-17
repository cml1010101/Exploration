package frc.lib.subsystems.arm;

import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Translation3d;
import frc.lib.subsystems.arm.Arm.ArmState;

public interface ArmSolver {
    public ArmState solve(Arm arm, Translation3d endPoint, Rotation3d endOrientation);
}
