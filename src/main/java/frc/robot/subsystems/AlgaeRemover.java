package frc.robot.subsystems;

import static frc.robot.Constants.AlgaeConstants.*;
import static frc.robot.Constants.SubsystemIDs.ALGAE_LEFT_MOTOR_ID;
import static frc.robot.Constants.SubsystemIDs.ALGAE_RIGHT_MOTOR_ID;

import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;

import frc.robot.Constants.AlgaeConstants.AlgaePosition;
import frc.robot.wrappers.GenericPID;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;


public class AlgaeRemover extends SubsystemBase{
    SparkMax leftMotor;
    SparkMax rightMotor;
    GenericPID leftPID;
    GenericPID rightPID;

    public AlgaeRemover(){
        leftMotor = new SparkMax(ALGAE_LEFT_MOTOR_ID, MotorType.kBrushless);
        rightMotor = new SparkMax(ALGAE_RIGHT_MOTOR_ID, MotorType.kBrushless);
        leftPID = new GenericPID(leftMotor, ControlType.kPosition, P_GAIN);
        rightPID = new GenericPID(rightMotor, ControlType.kPosition, P_GAIN);

        SparkMaxConfig leaderMotorConfig = new SparkMaxConfig();
        leaderMotorConfig.idleMode(IdleMode.kBrake)
            .inverted(true);
        leaderMotorConfig.encoder.positionConversionFactor(ALGAE_RATIO);
        leftMotor.configure(leaderMotorConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

        SparkMaxConfig followMotorConfig = new SparkMaxConfig();
        followMotorConfig.inverted(false)
            .idleMode(IdleMode.kBrake);
        followMotorConfig.encoder.positionConversionFactor(ALGAE_RATIO);
        rightMotor.configure(followMotorConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

    }

    public double getLeftPosition() { return leftMotor.getEncoder().getPosition(); }
    public boolean leftAtSetpoint() { return leftPID.atSetpoint(1); }
    public double getRightPosition() { return rightMotor.getEncoder().getPosition(); }
    public boolean rightAtSetpoint() { return rightPID.atSetpoint(1); }

    public void commandTwoPID(AlgaePosition pos){
        leftPID.activate(pos.getAngle());
        rightPID.activate(pos.getAngle());
    }

    public Command AlgaeRemoveArmOUt(AlgaePosition pos){
        return runEnd(
            () -> commandTwoPID(pos),
            () -> commandTwoPID(AlgaePosition.ZERO));
    }
    
     @Override
    public void periodic(){
        SmartDashboard.putNumber("Algae Left Position", getLeftPosition());
        SmartDashboard.putBoolean("Algae Left at Setpoint", leftAtSetpoint());
        SmartDashboard.putNumber("Algae Right Position", getRightPosition());
        SmartDashboard.putBoolean("Algae Right at Setpoint", rightAtSetpoint());
    }
}
