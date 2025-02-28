package frc.robot;

import edu.wpi.first.cameraserver.CameraServer;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import frc.robot.Constants.ElevatorConstants.ElevatorPosition;
import edu.wpi.first.wpilibj.TimedRobot;


public class Robot extends TimedRobot {
    private RobotContainer robotContainer;
    private Command autonCommand;

    @Override
    public void robotInit() {
        // Start the camera feed
        CameraServer.startAutomaticCapture();

        // Construct the robot container
        robotContainer = new RobotContainer();

        // Cancel any commands that may have persisted through power off or redeploy
        CommandScheduler.getInstance().cancelAll();
    }

    @Override
    public void robotPeriodic() {    
        // Always run the command scheduler to allow it to function
        CommandScheduler.getInstance().run();
    }

    @Override
    public void disabledExit() {
        // Reset the elevator's setpoint to zero on enable
        robotContainer.elevator.setTargetPosition(ElevatorPosition.ZERO);
    }

    @Override
    public void autonomousInit() {
        // Get the command to be used in auton
        autonCommand = robotContainer.getAutonomousCommand();
        // Schedule the command if there is one
        if (autonCommand != null)
            autonCommand.schedule();
    }
    
    @Override
    public void teleopInit() {
        // This makes sure that the autonomous command stops when teleop starts
        if (autonCommand != null)
            autonCommand.cancel();
    }

    @Override
    public void testInit() {
      // Kill any active commands when entering test mode
      CommandScheduler.getInstance().cancelAll();
    }
}