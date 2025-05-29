package frc.robot.subsystems;

import static frc.robot.Constants.ElevatorConstants.*;
import static frc.robot.Constants.SubsystemIDs.ELEVATOR_LEFT_MOTOR_ID;
import static frc.robot.Constants.SubsystemIDs.ELEVATOR_RIGHT_MOTOR_ID;
import com.ctre.phoenix6.configs.MotionMagicConfigs;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.hardware.CANdi;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.UpperChassisPose;


public class Elevator extends SubsystemBase {
     // Declare variables
    public TalonFX elevatorLeader;
    public TalonFX elevatorFollower; 
    public UpperChassisPose target = UpperChassisPose.ZERO;
    public CANdi canDi;
    

    // Constructor 
    public Elevator() {
    
        canDi = new CANdi(34); 
    
        // Elevator init
        elevatorLeader = new TalonFX(ELEVATOR_LEFT_MOTOR_ID);
        elevatorFollower = new TalonFX(ELEVATOR_RIGHT_MOTOR_ID);
        
       
        // Elevator config 
        TalonFXConfiguration leaderConfig = new TalonFXConfiguration();
        leaderConfig.MotorOutput.NeutralMode = NeutralModeValue.Coast;
        elevatorLeader.getConfigurator().apply(leaderConfig);
         
      //Elevator PID config
       Slot0Configs elevatorPIDConfig = new Slot0Configs();
        elevatorPIDConfig.kP = P_GAIN;
        elevatorPIDConfig.kD = D_GAIN; 
        elevatorPIDConfig.kG = G_GAIN;
        elevatorLeader.getConfigurator().apply(elevatorPIDConfig);
   
        MotionMagicConfigs motionMagicConfigs = new TalonFXConfiguration().MotionMagic;
      
        // Velocity is in RPS
        motionMagicConfigs.MotionMagicCruiseVelocity = VEL_LIMIT;

        //Acceleration is in RPS/S
        motionMagicConfigs.MotionMagicAcceleration = ACCEL_LIMIT;

        // Jerk is RPS/S/S
        motionMagicConfigs.MotionMagicJerk = JERK_LIMIT;
        

        elevatorLeader.getConfigurator().apply(motionMagicConfigs);

        
        TalonFXConfiguration followConfig = new TalonFXConfiguration();
        followConfig.MotorOutput.NeutralMode = NeutralModeValue.Coast;
    elevatorFollower.getConfigurator().apply(followConfig);
    elevatorFollower.setControl(new Follower(elevatorLeader.getDeviceID(), true));
    }


    // Getters
    public double getHeight() { return elevatorLeader.getPosition().getValueAsDouble(); }
    public double getVelocity() { return elevatorLeader.getVelocity().getValueAsDouble(); }
    public UpperChassisPose getTargetPosition() { return target; }
    public boolean atSetpoint() {
        return Math.abs(getHeight() - target.getHeight()) <= 2;
    }
    public boolean isPressed(){
    if(canDi.getS1Closed().getValue()){
        return true;
    }
    else 
        return false;
    }

 



    public void setTarget(UpperChassisPose pos) { 
        PositionVoltage elevatorRequest = new PositionVoltage(pos.getHeight()).withSlot(0);
        elevatorLeader.setControl(elevatorRequest); 
    }


     
     

    @Override
    public void periodic(){
         if( isPressed())
                elevatorLeader.setPosition(0.0);   
        SmartDashboard.putNumber("Elevator Height", getHeight());
        SmartDashboard.putString("Elevator Target Position",getTargetPosition().toString());
        SmartDashboard.putBoolean("Elevator at Setpoint", atSetpoint());
        SmartDashboard.putNumber("Elevator Velocity", getVelocity());
        SmartDashboard.putBoolean("Pressed", isPressed());
    }
}