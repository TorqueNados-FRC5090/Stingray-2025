package frc.robot.subsystems;

import static frc.robot.Constants.ElevatorConstants.ElevatorFactor;
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
    public Elevator(int leadMotorID, int followerMotorID) {
        leadMotor = new SparkMax(leadMotorID, MotorType.kBrushless);
        followerMotor = new SparkMax(followerMotorID, MotorType.kBrushless);
        elevatorPID = new ProfiledPIDController(.2, 0, 0, 
            new Constraints(1000, 30));
        elevatorPID.setTolerance(1);
        
        // Configure the elevator motors   
        SparkMaxConfig leaderConfig = new SparkMaxConfig();
        leaderConfig.idleMode(IdleMode.kCoast);
        leaderConfig.encoder.positionConversionFactor(1/ElevatorFactor);
        leadMotor.configure(leaderConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
        
        SparkMaxConfig followConfig = new SparkMaxConfig();
        followConfig.follow(leadMotorID, true)
            .idleMode(IdleMode.kCoast);
        followerMotor.configure(followConfig, ResetMode.kResetSafeParameters , PersistMode.kPersistParameters);
    }

    public double getPosition() { return leadMotor.getEncoder().getPosition(); }
    public boolean atSetpoint() { return elevatorPID.atSetpoint(); }

    // Moves the elevator to a specific position
    public void elevateToPosition(ElevatorPosition pos){
        double pidout = elevatorPID.calculate(leadMotor.getEncoder().getPosition(), pos.getHeight());
        leadMotor.setVoltage(pidout * RobotController.getBatteryVoltage());
    }

    public Command elevateCommand(ElevatorPosition pos) {
        return run(() -> elevateToPosition(pos));
    }
 
    // Print the height to the smartdashboard
    @Override
    public void periodic(){
       SmartDashboard.putNumber("Elevator Height", getPosition());
       SmartDashboard.putBoolean("Elevator at Setpoint", atSetpoint());
    }
}