package frc.robot.subsystems;

import com.revrobotics.servohub.ServoChannel;
import static frc.robot.Constants.ClimberConstants.*;
import static frc.robot.Constants.ServoPorts.CLIMBER_SERVO_PORT;

import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.ClimberConstants.ClimberPosition;
import frc.robot.wrappers.GenericPID;
import frc.robot.wrappers.ServoManager;

public class Climber extends SubsystemBase {
    SparkMax climbMotor;
    GenericPID climberPID;
    ServoChannel climberServo;
    
    // Constructor
    public Climber() {
       climbMotor = new SparkMax(MOTOR_ID, MotorType.kBrushless);
       climberPID = new GenericPID(climbMotor, ControlType.kPosition, P_GAIN);

       SparkMaxConfig config = new SparkMaxConfig();
       config.idleMode(IdleMode.kCoast);
       config.encoder.positionConversionFactor(CLIMBER_RATIO);
       climbMotor.configure(config, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

       climberServo = ServoManager.getInstance().getServoInPort(CLIMBER_SERVO_PORT);
    }

    
    
    /** @return A command that moves the climber to a position */
    public Command climbToPosition(ClimberPosition pos) {
        return runOnce(() -> driveClimberToPosition(pos));
    }
    
    public Command unlatchClimber(){
        return this.runEnd(
            () -> lockClimber(),
            () -> unlockClimber()
        );
    }


    /** Drives the climber to a position using the PID controller on its controller */
    public void driveClimberToPosition(ClimberPosition pos) { 
        climberServo.setPulseWidth(pos.getServoPos());
        climberPID.activate(pos.getAngle()); 
    }
    
    public void lockClimber(){
        climberServo.setPulseWidth(2000);
    }
    
    public void unlockClimber(){
        climberServo.setPulseWidth(500);
    }


    @Override
    public void periodic(){
        SmartDashboard.putNumber("Climber Angle", climberPID.getMeasurement()); 
        SmartDashboard.putNumber("Climber Servo", climberServo.getPulseWidth());
    }  
}
