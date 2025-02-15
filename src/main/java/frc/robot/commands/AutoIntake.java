package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Shooter;

public class AutoIntake extends Command{
    Shooter auto;
  
    public AutoIntake(Shooter auto){
       this.auto = auto;
       addRequirements(auto);
    }

    @Override
    public void initialize() {
        
    }

    // Called every time the scheduler runs while the command is scheduled.
    @Override
    public void execute() {
        if (auto.isSlowSensorBlocked()) {
            auto.shoot(.025);
        }
        else if (auto.isStopSensorBlocked()) {
            auto.shoot(0);
        }
        else {
            auto.shoot(.5);
        }
         
        
    }

    // Called once the command ends or is interrupted.
    @Override
    public void end(boolean interrupted) {
        
    }

    // Returns true when the command should end.
    @Override
    public boolean isFinished() {
        return false; // Has no end condition
    }
    
}
