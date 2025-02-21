package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Shooter;

public class AutoIntake extends Command{
    Shooter shooter;
  
    public AutoIntake(Shooter shooter){
       this.shooter = shooter;
       addRequirements(shooter);
    }

    @Override
    public void initialize() {}

    // Called every time the scheduler runs while the command is scheduled.
    @Override
    public void execute() {
        if (shooter.isEntrySensorBlocked())
            shooter.shoot(.25);
        else if (shooter.isExitSensorBlocked())
            shooter.shoot(0);
        else 
            shooter.shoot(.15);
    }

    // Called once the command ends or is interrupted.
    @Override
    public void end(boolean interrupted) {}

    // Returns true when the command should end.
    @Override
    public boolean isFinished() {
        return false; // Has no end condition
    }
}
