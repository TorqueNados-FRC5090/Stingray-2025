package frc.robot.subsystems;

import static frc.robot.Constants.ElevatorConstants.*;
import static frc.robot.Constants.SubsystemIDs.ELEVATOR_LEFT_MOTOR_ID;
import static frc.robot.Constants.SubsystemIDs.ELEVATOR_RIGHT_MOTOR_ID;

import com.ctre.phoenix6.configs.MotionMagicConfigs;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.trajectory.TrapezoidProfile.Constraints;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.UpperChassisPose;

import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;

public class Elevator extends SubsystemBase {
     // Declare variables
    public TalonFX elevatorLeader;
    public TalonFX elevatorFollower; 
    public UpperChassisPose target = UpperChassisPose.ZERO;
    

    // Constructor 
    public Elevator() {
        // Elevator init
        elevatorLeader = new TalonFX(ELEVATOR_LEFT_MOTOR_ID);
        elevatorFollower = new TalonFX(ELEVATOR_RIGHT_MOTOR_ID);

       
        // Elevator config 
        TalonFXConfiguration leaderConfig = new TalonFXConfiguration();
        leaderConfig.MotorOutput.NeutralMode = NeutralModeValue.Brake;
        elevatorLeader.getConfigurator().apply(leaderConfig);
         
      //Elevator PID config
       Slot0Configs elevatorPIDConfig = new Slot0Configs();
        elevatorPIDConfig.kP = P_GAIN;
        elevatorPIDConfig.kD = D_GAIN;
        elevatorLeader.getConfigurator().apply(elevatorPIDConfig);
   
        MotionMagicConfigs motionMagicConfigs = new TalonFXConfiguration().MotionMagic;
      
        // Velocity is in RPS
        motionMagicConfigs.MotionMagicCruiseVelocity = 100;

        //Acceleration is in RPS/S
        motionMagicConfigs.MotionMagicAcceleration = 60;

        // Jerk is RPS/S/S
        motionMagicConfigs.MotionMagicJerk = 1600;
        

        elevatorLeader.getConfigurator().apply(motionMagicConfigs);

        
        TalonFXConfiguration followConfig = new TalonFXConfiguration();
        followConfig.MotorOutput.NeutralMode = NeutralModeValue.Brake;
    elevatorFollower.getConfigurator().apply(followConfig);
    elevatorFollower.setControl(new Follower(elevatorLeader.getDeviceID(), true));
    }


    // Getters
    public double getHeight() { return elevatorLeader.getPosition().getValueAsDouble(); }
    public double getVelocity() { return elevatorLeader.getVelocity().getValueAsDouble(); }
    public UpperChassisPose getTargetPosition() { return target; }
    public boolean atSetpoint() {
        return Math.abs(getHeight() - target.getHeight()) <= 1;
    }

    public void setTarget(UpperChassisPose pos) { 
        PositionVoltage elevatorRequest = new PositionVoltage(pos.getHeight()).withSlot(0);
        elevatorLeader.setControl(elevatorRequest); 
    }
     
     

    @Override
    public void periodic(){
        

        SmartDashboard.putNumber("Elevator Height", getHeight());
        SmartDashboard.putString("Elevator Target Position", getTargetPosition().toString());
        SmartDashboard.putBoolean("Elevator at Setpoint", atSetpoint());
        SmartDashboard.putNumber("Elevator Velocity", getVelocity());
    }
}