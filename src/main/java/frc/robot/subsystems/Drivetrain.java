package frc.robot.subsystems;

import static edu.wpi.first.units.Units.Meters;

import java.io.IOException;
import java.util.List;

import org.json.simple.parser.ParseException;
import org.littletonrobotics.junction.AutoLogOutput;
import org.littletonrobotics.junction.Logger;

import com.ctre.phoenix6.hardware.Pigeon2;
import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.config.RobotConfig;
import com.pathplanner.lib.controllers.PPLTVController;
import com.pathplanner.lib.util.DriveFeedforwards;
import com.pathplanner.lib.util.PathPlannerLogging;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkBase;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.ClosedLoopConfig.FeedbackSensor;
import com.revrobotics.spark.config.SparkBaseConfig;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkMaxConfig;

import edu.wpi.first.math.estimator.DifferentialDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.DifferentialDriveKinematics;
import edu.wpi.first.math.kinematics.DifferentialDriveWheelSpeeds;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.DrivetrainConstants;

public class Drivetrain extends SubsystemBase {
    private final SparkMax leftA;
    private final SparkMax leftB;
    private final SparkMax rightA;
    private final SparkMax rightB;

    private final RelativeEncoder leftEncoder;
    private final RelativeEncoder rightEncoder;

    private final SparkClosedLoopController leftController;
    private final SparkClosedLoopController rightController;

    private final Pigeon2 gyro;

    public final DifferentialDrive drive;
    public final DifferentialDriveKinematics kinematics;
    public final DifferentialDrivePoseEstimator poseEstimator;

    public Drivetrain() {
        leftA = new SparkMax(DrivetrainConstants.leftMotorAID, MotorType.kBrushless);
        leftB = new SparkMax(DrivetrainConstants.leftMotorBID, MotorType.kBrushless);
        rightA = new SparkMax(DrivetrainConstants.rightMotorAID, MotorType.kBrushless);
        rightB = new SparkMax(DrivetrainConstants.rightMotorBID, MotorType.kBrushless);

        configureMotor(leftA, false, null);
        configureMotor(leftB, false, leftA);
        configureMotor(rightA, true, null);
        configureMotor(rightB, true, rightA);

        leftEncoder = leftA.getEncoder();
        rightEncoder = rightA.getEncoder();
        leftController = leftA.getClosedLoopController();
        rightController = rightA.getClosedLoopController();

        gyro = new Pigeon2(DrivetrainConstants.pigeonID);

        kinematics = new DifferentialDriveKinematics(DrivetrainConstants.trackWidth.in(Meters));
        drive = new DifferentialDrive(leftA, rightA);
        poseEstimator = new DifferentialDrivePoseEstimator(kinematics, new Rotation2d(), 0, 0, new Pose2d());
    
        createAutoBuilder();
    }
    
    private void configureMotor(SparkMax motor, boolean invert, SparkBase follow) {
        SparkBaseConfig config = new SparkMaxConfig();
        config.smartCurrentLimit(DrivetrainConstants.currentLimit);
        config.inverted(invert);
        config.idleMode(IdleMode.kBrake);

        if (follow != null) {
            config.follow(follow);
        }

        config.encoder.positionConversionFactor(1.0/DrivetrainConstants.motorReduction);
        config.encoder.velocityConversionFactor(1.0/DrivetrainConstants.motorReduction);

        config.closedLoop.feedbackSensor(FeedbackSensor.kPrimaryEncoder);
        config.closedLoop.pid(DrivetrainConstants.p, DrivetrainConstants.i, DrivetrainConstants.d);
        config.closedLoop.outputRange(-1, 1);

        motor.configure(config, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    }

    private void createAutoBuilder() {
        try {
            AutoBuilder.configure(
                this::getPose, 
                poseEstimator::resetPose,
                () -> kinematics.toChassisSpeeds(
                    new DifferentialDriveWheelSpeeds(
                        getLeftVelocityMetersPerSec(),
                        getRightPositionMeters()
                    )
                ),
                this::runClosedLoop, 
                new PPLTVController(0.02), 
                RobotConfig.fromGUISettings(), 
                () -> DriverStation.getAlliance().orElse(Alliance.Blue) == Alliance.Red, 
                this
            );
        } catch (IOException | ParseException e) {
            e.printStackTrace();
        }
        PathPlannerLogging.setLogActivePathCallback(
            (List<Pose2d> path) -> Logger.recordOutput("Drivetrain/ActivePath", path.toArray(new Pose2d[0]))
        );
        PathPlannerLogging.setLogTargetPoseCallback(
            (Pose2d pose) -> Logger.recordOutput("Drivetrain/TargetPose", pose)
        );
    }

    @Override
    public void periodic() {
        poseEstimator.update(gyro.getRotation2d(), getLeftPositionMeters(), getRightPositionMeters());

        Logger.recordOutput("Drivetrain/Outputs/LeftA", leftA.getAppliedOutput());
        Logger.recordOutput("Drivetrain/Outputs/LeftB", leftB.getAppliedOutput());
        Logger.recordOutput("Drivetrain/Outputs/RightA", rightA.getAppliedOutput());
        Logger.recordOutput("Drivetrain/Outputs/RightB", rightB.getAppliedOutput());
        Logger.recordOutput("Drivetrain/Currents/LeftA", leftA.getAppliedOutput());
        Logger.recordOutput("Drivetrain/Currents/LeftB", leftB.getAppliedOutput());
        Logger.recordOutput("Drivetrain/Currents/RightA", rightA.getAppliedOutput());
        Logger.recordOutput("Drivetrain/Currents/RightB", rightB.getAppliedOutput());
    }

    public void runClosedLoop(ChassisSpeeds speeds, DriveFeedforwards feedforwards) {
        var wheelSpeeds = kinematics.toWheelSpeeds(speeds);
        runClosedLoop(wheelSpeeds.leftMetersPerSecond, wheelSpeeds.rightMetersPerSecond);
    }

    public void runClosedLoop(double leftMetersPerSec, double rightMetersPerSec) {
        Logger.recordOutput("Drivetrain/LeftVelocitySetpoint", leftMetersPerSec);
        Logger.recordOutput("Drivetrain/RightVelocitySetpoint", rightMetersPerSec);

        leftController.setReference(
            leftMetersPerSec / DrivetrainConstants.velocityConversion, 
            ControlType.kVelocity
        );

        rightController.setReference(
            rightMetersPerSec / DrivetrainConstants.velocityConversion, 
            ControlType.kVelocity
        );
    }

    @AutoLogOutput
    public Pose2d getPose() {
      return poseEstimator.getEstimatedPosition();
    }

    @AutoLogOutput
    public double getLeftPositionMeters() {
        return leftEncoder.getPosition() * DrivetrainConstants.positionConversion;
    }

    @AutoLogOutput
    public double getLeftVelocityMetersPerSec() {
        return leftEncoder.getVelocity() * DrivetrainConstants.velocityConversion;
    }

    @AutoLogOutput
    public double getRightPositionMeters() {
        return rightEncoder.getPosition() * DrivetrainConstants.positionConversion;
    }

    @AutoLogOutput
    public double getRightVelocityMetersPerSec() {
        return rightEncoder.getVelocity() * DrivetrainConstants.velocityConversion;
    }
}
