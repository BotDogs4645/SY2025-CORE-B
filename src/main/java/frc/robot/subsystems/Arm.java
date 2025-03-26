package frc.robot.subsystems;

import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkLowLevel.MotorType;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.ArmConstants;

public class Arm extends SubsystemBase {
    private final SparkMax wheelMotor;
    private final SparkMax pivotMotor;

    public Arm() {
        wheelMotor = new SparkMax(ArmConstants.WheelMotorID, MotorType.kBrushed);
        pivotMotor = new SparkMax(ArmConstants.pivotMotorID, MotorType.kBrushed);
    }

    public void setDutyCycle(double speed) {
        wheelMotor.set(speed);
    }

    public void setPivotDutyCycle(double speed) {
        pivotMotor.set(speed);
    }
}
