package frc.robot;

import edu.wpi.first.cameraserver.CameraServer;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import frc.robot.Constants.ElevatorConstants.ElevatorPosition;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.XboxController;


public class Robot extends TimedRobot {
    private RobotContainer robotContainer;
    private Command autonCommand;
    private XboxController testingController;

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
        robotContainer.elevator.setElevatorTarget(ElevatorPosition.ZERO);
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
            autonCommand
            .cancel();
    }

    @Override
    public void testInit() {
      // Kill any active commands when entering test mode
      CommandScheduler.getInstance().cancelAll();
      // Initialize a testing controller
      testingController = new XboxController(3);
    }
    
    @Override
    public void testPeriodic() {
        // Climber Controls
        if(testingController.getBButtonPressed())
            robotContainer.climber.manual(.25);
        else if (testingController.getBButtonReleased())
            robotContainer.climber.manual(0);
        
        if(testingController.getXButtonPressed())
            robotContainer.climber.manual(-.25);
        else if (testingController.getXButtonReleased())
            robotContainer.climber.manual(0);

        if(testingController.getAButtonPressed())
            robotContainer.climber.resetEncoder();

        // Pivot Controls
        if(testingController.getLeftBumperButtonPressed())
            robotContainer.elevator.manualPivot(.1);
        else if(testingController.getLeftBumperButtonReleased())
            robotContainer.elevator.manualPivot(0);
      
        if(testingController.getRightBumperButtonPressed())
            robotContainer.elevator.manualPivot(-.1);
        else if(testingController.getRightBumperButtonReleased())
            robotContainer.elevator.manualPivot(0);

        if(testingController.getYButtonPressed())
            robotContainer.elevator.resetPivotEncoder();
    }
}