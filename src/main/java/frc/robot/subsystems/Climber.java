package frc.robot.subsystems;

import static frc.robot.Constants.ServoPorts.CLIMBER_SERVO_PORT;
import com.revrobotics.servohub.ServoChannel;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkMaxConfig;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.ClimberConstants.ClimberPosition;
import frc.robot.wrappers.GenericPID;
import frc.robot.wrappers.ServoManager;

public class Climber extends SubsystemBase {
    
    SparkMax climbMotor;
    GenericPID climberPID;
    ClimberPosition pos;

    ServoChannel climberServo;
    

    public Climber(int climberID, double p){
       climbMotor = new SparkMax(climberID, MotorType.kBrushless);
       SparkMaxConfig config = new SparkMaxConfig();
       climbMotor.configure(config, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
       climberPID = new GenericPID(climbMotor, ControlType.kPosition, p);

        climberServo = ServoManager.getInstance().getServoInPort(CLIMBER_SERVO_PORT);
    }

    public Command unlatchClimber(){
        return this.runEnd(
            () -> lockClimber(),
            () -> unlockClimber()
        );
    }
   // moves climber to any position
    public void moveClimber(double speed){
        climbMotor.set(speed);
    }

    // moves climber to setpoints
    public void climbTo(ClimberPosition pos){
        climberPID.activate(pos.getAngle());
    }

    public void stop(){
        climbMotor.set(0);
    }

    public void lockClimber(){
        climberServo.setPulseWidth(2000);
    }

    public void unlockClimber(){
        climberServo.setPulseWidth(500);
    }

    @Override
    public void periodic(){
       SmartDashboard.putNumber(getName(), climberPID.getMeasurement()); 
       SmartDashboard.putNumber("Climber Servo", climberServo.getPulseWidth());
    }  
}
