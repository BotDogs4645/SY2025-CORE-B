package frc.robot.commands;

import java.util.function.Supplier;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Drivetrain;

public class TeleopDrive extends Command {
    private final Supplier<Double> forwardSupplier;
    private final Supplier<Double> turnSupplier;

    private final Drivetrain drivetrain;

    public TeleopDrive(Drivetrain drivetrain, Supplier<Double> forwardSupplier, Supplier<Double> turnSupplier) {
        this.forwardSupplier = forwardSupplier;
        this.turnSupplier = turnSupplier;

        this.drivetrain = drivetrain;
        this.addRequirements(drivetrain);
    }

    @Override
    public void execute() {
        double driveValue = MathUtil.applyDeadband(forwardSupplier.get(), 0.1);
        double turnValue = MathUtil.applyDeadband(turnSupplier.get(), 0.1);
        Logger.recordOutput("Drivetrain/ControllerDrive", driveValue);
        Logger.recordOutput("Drivetrain/ControllerTurn", turnValue);
        drivetrain.drive.arcadeDrive(driveValue, turnValue);
    }
}
