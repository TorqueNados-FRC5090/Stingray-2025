package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.ElevatorConstants.ElevatorPosition;
import frc.robot.subsystems.Elevator;

public class SetElevatorTarget extends Command{
    Elevator elevator;
    ElevatorPosition target;
  
    public SetElevatorTarget(Elevator elevator, ElevatorPosition targetPosition){
       this.elevator = elevator;
       this.target = targetPosition;

       addRequirements(elevator);
    }


    @Override // Sets the elevator's target when the command starts
    public void initialize() { elevator.setTargetPosition(target); }

    @Override // Ends the command once the elevator has reached its target
    public boolean isFinished() { return elevator.atSetpoint(); }
}
