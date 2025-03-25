package frc.robot.subsystems;

import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkLowLevel.MotorType;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.RollerConstants;

public class Roller extends SubsystemBase {
    private final SparkMax rollerMotor;

    public Roller() {
        rollerMotor = new SparkMax(RollerConstants.rollerMotorID, MotorType.kBrushed);
    }

    public void setDutyCycle(double speed) {
        rollerMotor.set(speed);
    }
}
