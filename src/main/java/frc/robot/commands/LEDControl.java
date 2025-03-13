package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.RobotContainer;
import frc.robot.Constants.LEDConstants.LEDColor;
import frc.robot.subsystems.Candle;
import frc.robot.subsystems.Elevator;
import frc.robot.subsystems.Shooter;
import frc.robot.Constants.ElevatorConstants.*;

public class LEDControl extends Command{
    
    Shooter shooter;
    Candle candle;
    Elevator elevator;

    public LEDControl(Candle candle, RobotContainer robot){
        shooter = robot.shooter;
        elevator = robot.elevator;
        this.candle = candle;

        addRequirements(candle);
    }

    @Override
    public void initialize() {}

    // Called every time the scheduler runs while the command is scheduled.
    @Override
    public void execute() {
        if (readyToShoot())
            candle.setAll(LEDColor.GREEN);
        else if (shooter.isExitSensorBlocked())
            candle.setAll(LEDColor.ORANGE);
        else
            candle.setAll(LEDColor.BLUE);
    }

    private boolean readyToShoot() {
        return shooter.isExitSensorBlocked() && elevator.getTargetPosition() != ElevatorPosition.ZERO && elevator.atSetpoint();
    }

    // Called once the command ends or is interrupted.
    @Override
    public void end(boolean interrupted) {}

    // Returns true when the command should end.
    @Override
    public boolean isFinished() {
        return false; // Has no end condition
    }

    @Override
    public boolean runsWhenDisabled() {
        return true;
    }
}
