package frc.robot;

import edu.wpi.first.cameraserver.CameraServer;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import frc.robot.Constants.ElevatorConstants.ElevatorPosition;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;


public class Robot extends TimedRobot {
    private RobotContainer robotContainer;
    private Command autonCommand;
    private XboxController testingController;
    private boolean useLimelight = true;

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
        CommandScheduler.getInstance().run();

        /*
        * This example of adding Limelight is very simple and may not be sufficient for on-field use.
        * Users typically need to provide a standard deviation that scales with the distance to target
        * and changes with number of tags available.
        *
        * This example is sufficient to show that vision integration is possible, though exact implementation
        * of how to use vision should be tuned per-robot and to the team's specification.
        */
        if (useLimelight) {
            var driveState = robotContainer.drivetrain.getState();
            double headingDeg = robotContainer.drivetrain.getState().Pose.getRotation().getDegrees();
            double omegaRps = Units.radiansToRotations(driveState.Speeds.omegaRadiansPerSecond);            
        
            robotContainer.frontLimelight.SetRobotOrientation(headingDeg, omegaRps, 0, 0, 0, 0);
            var llMeasurement = robotContainer.frontLimelight.getBotPoseEstimate_wpiBlue();
            if (llMeasurement != null && llMeasurement.tagCount > 0 && Math.abs(omegaRps) < 2.0) {
                robotContainer.drivetrain.addVisionMeasurement(llMeasurement.pose, llMeasurement.timestampSeconds);
            }
            SmartDashboard.putNumber("Limelight pose X", llMeasurement.pose.getX());
            SmartDashboard.putNumber("Limelight pose Y",  llMeasurement.pose.getY());
            SmartDashboard.putNumber("Drive pose X", robotContainer.drivetrain.getState().Pose.getX());
            SmartDashboard.putNumber("Drive pose Y",  robotContainer.drivetrain.getState().Pose.getY());
            SmartDashboard.putNumber("Heading Degrees", robotContainer.drivetrain.getState().Pose.getRotation().getDegrees());
        }
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
      // Initialize a testing controller
      testingController = new XboxController(3);
    }
    
    @Override
    public void testPeriodic() {
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
    }
}