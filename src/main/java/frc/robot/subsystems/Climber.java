package frc.robot.subsystems;

import static frc.robot.Constants.ClimberConstants.CLIMBER_RATIO;

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

public class Climber extends SubsystemBase {
    SparkMax climbMotor;
    GenericPID climberPID;
    
    // Constructor
    public Climber(int climberID) {
       climbMotor = new SparkMax(climberID, MotorType.kBrushless);
       climberPID = new GenericPID(climbMotor, ControlType.kPosition, .27);

       SparkMaxConfig config = new SparkMaxConfig();
       config.encoder.positionConversionFactor(CLIMBER_RATIO);
       climbMotor.configure(config, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    }


    /** @return A command that moves the climber to a position */
    public Command climbToPosition(ClimberPosition pos) {
        return runOnce(() -> driveClimberToPosition(pos));
    }


    /** Drives the climber to a position using the PID controller on its controller */
    public void driveClimberToPosition(ClimberPosition pos) { climberPID.activate(pos.getAngle()); }

    
    @Override
    public void periodic() {
       SmartDashboard.putNumber("Climber Angle", climberPID.getMeasurement()); 
    }
}
