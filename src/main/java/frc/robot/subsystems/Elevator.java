package frc.robot.subsystems;

import static frc.robot.Constants.ElevatorConstants.*;
import static frc.robot.Constants.SubsystemIDs.ELEVATOR_LEFT_MOTOR_ID;
import static frc.robot.Constants.SubsystemIDs.ELEVATOR_RIGHT_MOTOR_ID;

import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.trajectory.TrapezoidProfile.Constraints;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;

public class Elevator extends SubsystemBase {
     // Declare variables
    public SparkMax leadMotor;
    public SparkMax followerMotor; 
    public ProfiledPIDController elevatorPID;
    public ElevatorPosition targetPosition = ElevatorPosition.ZERO;
    

    // Constructor 
    public Elevator() {
        leadMotor = new SparkMax(ELEVATOR_LEFT_MOTOR_ID, MotorType.kBrushless);
        followerMotor = new SparkMax(ELEVATOR_RIGHT_MOTOR_ID, MotorType.kBrushless);
        elevatorPID = new ProfiledPIDController(P_GAIN, 0, D_GAIN, 
            new Constraints(VEL_LIMIT, ACCEL_LIMIT));
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


    // Getters
    public double getHeight() { return leadMotor.getEncoder().getPosition(); }
    public boolean atSetpoint() { return elevatorPID.atSetpoint(); }
    public ElevatorPosition getTargetPosition() { return targetPosition; }

    public void setTargetPosition(ElevatorPosition pos) { targetPosition = pos; }

    /** Sends voltage to the elevator to drive it to a position */
    private void driveElevator() {
        double pidout = elevatorPID.calculate(this.getHeight(), targetPosition.getHeight());
        leadMotor.setVoltage(pidout * RobotController.getBatteryVoltage());
    }
    

    @Override
    public void periodic(){
<<<<<<< HEAD
        SmartDashboard.putNumber("Elevator Height", getHeight());
=======
        driveElevator();

        SmartDashboard.putNumber("Elevator Height", getHeight());
        SmartDashboard.putString("Elevator Target Position", getTargetPosition().toString());
>>>>>>> e0b442049bdcad4d294a02a2e6b99ae2f9bebe1f
        SmartDashboard.putBoolean("Elevator at Setpoint", atSetpoint());
    }
}