package frc.robot.subsystems;

import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkMaxConfig;
import static frc.robot.Constants.ElevatorConstants.ElevatorFactor;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;

import frc.robot.Constants.ElevatorConstants.ElevatorPosition;
import frc.robot.wrappers.GenericPID;


public class Elevator extends SubsystemBase {
     // Declare variables
    public SparkMax leadMotor;
    public SparkMax followerMotor; 
    public GenericPID elevatorPID;
    public ElevatorPosition pos;
    


    // Constructor 
    public Elevator(int leadMotorID, int followerMotorID, double p) {
        leadMotor = new SparkMax(leadMotorID, MotorType.kBrushless);
        followerMotor = new SparkMax(followerMotorID, MotorType.kBrushless);
       
        elevatorPID = new GenericPID(leadMotor, ControlType.kPosition, p);
        
            // configuration for leader motor   
        SparkMaxConfig leaderConfig = new SparkMaxConfig();
        leaderConfig.encoder.positionConversionFactor(ElevatorFactor);
            
            // configuration for follower motor
        SparkMaxConfig followConfig = new SparkMaxConfig();
        followConfig.follow(leadMotorID, true);
        
        followerMotor.configure(followConfig, ResetMode.kResetSafeParameters , PersistMode.kPersistParameters);
    }

    // moves the elevator to a specific position
    public void elevateTo(ElevatorPosition pos){
        elevatorPID.activate(pos.getAngle());     
    }
 
    // gives us the height
    @Override
    public void periodic(){
       SmartDashboard.putNumber(getName(), elevatorPID.getMeasurement());
    }
}