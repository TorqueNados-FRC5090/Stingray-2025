package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Funnel;

public class FunnelDrop extends Command{
    Funnel funnel;


   public FunnelDrop(Funnel funnel){
  this.funnel = funnel;

  addRequirements(funnel);
   
    }

    

    @Override
    public void initialize() {
    
    }

    // Called every time the scheduler runs while the command is scheduled.
    @Override
    public void execute() {
      funnel.unlatch();
    }

    // Called once the command ends or is interrupted.
    @Override
    public void end(boolean interrupted) {
       funnel.zero();
    }

    // Returns true when the command should end.
    @Override
    public boolean isFinished() {
        return false; // Has no end condition
    }
}
