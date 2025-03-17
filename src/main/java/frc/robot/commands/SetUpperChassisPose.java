package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Constants.UpperChassisPose;
import frc.robot.subsystems.Elevator;
import frc.robot.subsystems.Pivot;

public class SetUpperChassisPose extends SequentialCommandGroup {

    /** Creates a command that does the following:
     * <p> 1. Move the pivot to zero </p>
     * <p> 2. Move the elevator to the height associated with the given pose </p>
     * <p> 3. Move the pivot to the angle associated with the given pose </p>
     */
    public SetUpperChassisPose(Elevator elevator, Pivot pivot, UpperChassisPose pose) {
        addCommands(
            new SetPivotPose(pivot, UpperChassisPose.ZERO),
            new SetElevatorPose(elevator, pose),
            new SetPivotPose(pivot, pose)
        );
    }
    

    private class SetElevatorPose extends Command {
        private Elevator elevator;
        private UpperChassisPose target;
    
        public SetElevatorPose(Elevator elevator, UpperChassisPose targetPosition){
            this.elevator = elevator;
            this.target = targetPosition;

            addRequirements(elevator);
        }


        @Override // Sets the elevator's target when the command starts
        public void initialize() { elevator.setTarget(target); }

        @Override // Ends the command once the elevator has reached its target
        public boolean isFinished() { 
            return elevator.atSetpoint(); 
        }
    }

    private class SetPivotPose extends Command {
        private Pivot pivot;
        private UpperChassisPose target;
    
        public SetPivotPose(Pivot pivot, UpperChassisPose targetPosition){
            this.pivot = pivot;
            this.target = targetPosition;

            addRequirements(pivot);
        }


        @Override // Sets the elevator's target when the command starts
        public void initialize() { pivot.setTarget(target); }

        @Override // Ends the command once the elevator has reached its target
        public boolean isFinished() { 
            return pivot.atSetpoint(); 
        }
    }
}
