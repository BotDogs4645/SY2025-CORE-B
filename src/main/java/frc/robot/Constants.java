package frc.robot;

import static edu.wpi.first.units.Units.Inches;
import static edu.wpi.first.units.Units.Meters;

import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.wpilibj.RobotBase;

public class Constants {
    public static final Mode simMode = Mode.SIM;
    public static final Mode currentMode = RobotBase.isReal() ? Mode.REAL : simMode;

    public static final int driveControllerId = 0;
    public static final int pdhCANId = 1;

    public static class ArmConstants {
        public static final int pivotMotorID = 8;
        public static final int WheelMotorID = 9;
    }

    public static class RollerConstants {
        public static final int rollerMotorID = 1;
        public static final double wheelDutyCycle = 0.5;
    }

    public static class DrivetrainConstants {
        // Smart current limit (amps) per motor
        public static final int currentLimit = 60;

        public static final double p = 100;
        public static final double i = 0;
        public static final double d = 5;        

        public static final int leftMotorAID = 2;
        public static final int leftMotorBID = 5;
        public static final int rightMotorAID = 3;
        public static final int rightMotorBID = 4;

        public static final int pigeonID = 7;

        public static final Distance trackWidth = Inches.of(21.5);
        public static final Distance wheelRadius = Inches.of(3.0);
        public static final double motorReduction = 10.71;

        public static final double positionConversion = Math.PI * 2 * wheelRadius.in(Meters);
        public static final double velocityConversion = Math.PI * 2 * wheelRadius.in(Meters) / 60;
    }

    public static enum Mode {
        REAL, // Running on a real robot.
        SIM, // Running a physics simulator.
        REPLAY // Replaying from a log file.
    }
}
