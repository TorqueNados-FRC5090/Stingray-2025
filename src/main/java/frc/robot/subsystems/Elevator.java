package frc.robot.subsystems;

import static frc.robot.Constants.ElevatorConstants.*;
import static frc.robot.Constants.SubsystemIDs.ELEVATOR_LEFT_MOTOR_ID;
import static frc.robot.Constants.SubsystemIDs.ELEVATOR_RIGHT_MOTOR_ID;
import static frc.robot.Constants.SubsystemIDs.PIVOT_MOTOR_ID;

import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
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
    public SparkMax elevatorLeader;
    public SparkMax elevatorFollower; 
    public TalonFX pivotMotor;
    public ProfiledPIDController elevatorPID;
    public ElevatorPosition elevatorTarget = ElevatorPosition.ZERO;
    

    // Constructor 
    public Elevator() {
        // Elevator init
        elevatorLeader = new SparkMax(ELEVATOR_LEFT_MOTOR_ID, MotorType.kBrushless);
        elevatorFollower = new SparkMax(ELEVATOR_RIGHT_MOTOR_ID, MotorType.kBrushless);

        elevatorPID = new ProfiledPIDController(P_GAIN, 0, D_GAIN, 
            new Constraints(VEL_LIMIT, ACCEL_LIMIT));
        elevatorPID.setTolerance(1);
        
        // Elevator config 
        SparkMaxConfig leaderConfig = new SparkMaxConfig();
        leaderConfig.idleMode(IdleMode.kCoast);
        leaderConfig.encoder.positionConversionFactor(ELEVATOR_RATIO);
        elevatorLeader.configure(leaderConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
        
        SparkMaxConfig followConfig = new SparkMaxConfig();
        followConfig.follow(elevatorLeader, true)
            .idleMode(IdleMode.kCoast);
        elevatorFollower.configure(followConfig, ResetMode.kResetSafeParameters , PersistMode.kPersistParameters);

        // Pivot init and config
        pivotMotor = new TalonFX(PIVOT_MOTOR_ID);
        Slot0Configs pivotPIDConfig = new Slot0Configs();
        pivotPIDConfig.kP = 0;
        pivotMotor.getConfigurator().apply(pivotPIDConfig);
    }


    // Getters
    public double getHeight() { return elevatorLeader.getEncoder().getPosition(); }
    public boolean atSetpoint() { return elevatorPID.atSetpoint(); }
    public ElevatorPosition getElevatorTarget() { return elevatorTarget; }
    
    public void resetPivotEncoder(){ pivotMotor.setPosition(0); }
    public void manualPivot(double speed) {
        pivotMotor.set(speed);
    }

    public void setElevatorTarget(ElevatorPosition pos) { 
        elevatorTarget = pos; 
        PositionVoltage pivotRequest = new PositionVoltage(pos.getAngle()).withSlot(0);
        pivotMotor.setControl(pivotRequest);
    }

    /** Sends voltage to the elevator to drive it to a position */
    private void driveElevator() {
        double pidout = elevatorPID.calculate(this.getHeight(), elevatorTarget.getHeight());
        elevatorLeader.setVoltage(pidout * RobotController.getBatteryVoltage());
    }
    

    @Override
    public void periodic(){
        driveElevator();

        SmartDashboard.putNumber("Elevator Height", getHeight());
        SmartDashboard.putString("Elevator Target Position", getElevatorTarget().toString());
        SmartDashboard.putBoolean("Elevator at Setpoint", atSetpoint());

        SmartDashboard.putNumber("Pivot Angle", pivotMotor.getPosition().getValueAsDouble());
    }
}