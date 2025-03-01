package frc.robot.subsystems;

import com.revrobotics.servohub.ServoChannel;
import static frc.robot.Constants.ClimberConstants.*;
import static frc.robot.Constants.SubsystemIDs.CLIMBER_MOTOR_ID;
import static frc.robot.Constants.SubsystemIDs.CLIMBER_SERVO_PORT;

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
        climbMotor = new SparkMax(CLIMBER_MOTOR_ID, MotorType.kBrushless);
        climberPID = new GenericPID(climbMotor, ControlType.kPosition, P_GAIN);

        SparkMaxConfig config = new SparkMaxConfig();
        config.idleMode(IdleMode.kBrake)
            .inverted(true);
        config.encoder.positionConversionFactor(CLIMBER_RATIO);
        climbMotor.configure(config, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

        climberServo = ServoManager.getInstance().getServoInPort(CLIMBER_SERVO_PORT);
    }

    /** Puts servo in position, then moves the climber  */
    public Command climbToPosition(ClimberPosition pos){
        return runOnce(() -> climberPID.activate(pos.getAngle()));
    }
    
    public void manual(double speed) {climbMotor.set(speed);}
    public void resetEncoder() {climbMotor.getEncoder().setPosition(0);}
   
    @Override
    public void periodic(){
        SmartDashboard.putNumber("Climber Angle", climberPID.getMeasurement()); 
    }  
}
