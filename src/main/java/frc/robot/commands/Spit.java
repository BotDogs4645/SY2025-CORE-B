package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.RollerConstants;
import frc.robot.subsystems.Roller;

public class Spit extends Command {
    private final Roller roller;
    
    public Spit(Roller roller) {
        this.roller = roller;
        addRequirements(roller);
    }

    @Override
    public void initialize() {
        roller.setDutyCycle(RollerConstants.wheelDutyCycle);
    }

    @Override
    public void end(boolean interrupted) {
        roller.setDutyCycle(0);
    }
}
