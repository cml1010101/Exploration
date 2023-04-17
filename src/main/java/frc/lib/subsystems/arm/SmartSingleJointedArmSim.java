package frc.lib.subsystems.arm;

import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N2;
import edu.wpi.first.math.system.LinearSystem;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.wpilibj.simulation.SingleJointedArmSim;

public class SmartSingleJointedArmSim extends SingleJointedArmSim {
    public SmartSingleJointedArmSim(LinearSystem<N2, N1, N1> plant, DCMotor gearbox, double gearing,
            double armLengthMeters, double minAngleRads, double maxAngleRads, boolean simulateGravity) {
        super(plant, gearbox, gearing, armLengthMeters, minAngleRads, maxAngleRads, simulateGravity);
    }
    public LinearSystem<N2, N1, N1> getPlant()
    {
        return super.m_plant;
    }
    public void loadNewPlant(LinearSystem<N2, N1, N1> plant)
    {
        for (int i = 0; i < 2; i++)
        {
            for (int j = 0; j < 2; j++)
                super.m_plant.getA().set(i, j, plant.getA(i, j));
            super.m_plant.getB().set(i, 0, plant.getB(i, 0));
            super.m_plant.getC().set(0, i, plant.getC(0, i));
        }
        super.m_plant.getD().set(0, 0, plant.getD(0, 0));
    }
}
