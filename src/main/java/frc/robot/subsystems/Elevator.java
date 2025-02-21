package frc.robot.subsystems;

import static frc.robot.Constants.ElevatorConstants.*;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.trajectory.TrapezoidProfile.Constraints;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import frc.robot.Constants.ElevatorConstants.ElevatorPosition;

public class Elevator extends SubsystemBase {
     // Declare variables
    public SparkMax leadMotor;
    public SparkMax followerMotor; 
    public ProfiledPIDController elevatorPID;
    public ElevatorPosition pos;
    
    // Constructor 
    public Elevator() {
        leadMotor = new SparkMax(LEFT_MOTOR_ID, MotorType.kBrushless);
        followerMotor = new SparkMax(RIGHT_MOTOR_ID, MotorType.kBrushless);
        elevatorPID = new ProfiledPIDController(P_GAIN, 0, 0, 
            new Constraints(100, 30));
        elevatorPID.setTolerance(1);
        
        // Configure the elevator motors   
        SparkMaxConfig leaderConfig = new SparkMaxConfig();
        leaderConfig.idleMode(IdleMode.kCoast);
        leaderConfig.encoder.positionConversionFactor(ELEVATOR_RATIO);
        leadMotor.configure(leaderConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
        
        SparkMaxConfig followConfig = new SparkMaxConfig();
        followConfig.follow(leadMotor, true)
            .idleMode(IdleMode.kCoast);
        followerMotor.configure(followConfig, ResetMode.kResetSafeParameters , PersistMode.kPersistParameters);
    }

    
    /** @return A command that tells the elevator to go to a position */
    public Command elevateToPosition(ElevatorPosition pos) {
        return run(() -> driveElevatorToPosition(pos));
    }

    // Getters
    public double getPosition() { return leadMotor.getEncoder().getPosition(); }
    public boolean atSetpoint() { return elevatorPID.atSetpoint(); }

    /** Sends voltage to the elevator to drive it to a position */
    private void driveElevatorToPosition(ElevatorPosition pos){
        double pidout = elevatorPID.calculate(leadMotor.getEncoder().getPosition(), pos.getHeight());
        leadMotor.setVoltage(pidout * RobotController.getBatteryVoltage());
    }
    

    @Override
    public void periodic(){
       SmartDashboard.putNumber("Elevator Height", getPosition());
       SmartDashboard.putBoolean("Elevator at Setpoint", atSetpoint());
    }
}