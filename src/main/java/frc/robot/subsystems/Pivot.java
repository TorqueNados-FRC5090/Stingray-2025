package frc.robot.subsystems;

import static frc.robot.Constants.SubsystemIDs.PIVOT_MOTOR_ID;

import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.UpperChassisPose;

public class Pivot extends SubsystemBase{
    TalonFX pivotMotor;
    UpperChassisPose target;

    public Pivot() {
        // Pivot init and config
        pivotMotor = new TalonFX(PIVOT_MOTOR_ID);
        Slot0Configs pivotPIDConfig = new Slot0Configs();
        pivotPIDConfig.kP = 0.3;
        pivotPIDConfig.kD = .005;
        pivotMotor.getConfigurator().apply(pivotPIDConfig);
    }

    public double getAngle() { return pivotMotor.getPosition().getValueAsDouble(); }
    public boolean atSetpoint() {
        return Math.abs(getAngle() - target.getAngle()) <= 1;
    }

    public void resetPivotEncoder(){ pivotMotor.setPosition(0); }
    public void manualPivot(double speed) {
        pivotMotor.set(speed);
    }

    public void setTarget(UpperChassisPose pose) {
        PositionVoltage pivotRequest = new PositionVoltage(pose.getAngle()).withSlot(0);
        pivotMotor.setControl(pivotRequest);
    }

    @Override
    public void periodic() {
        SmartDashboard.putNumber("Pivot Angle", pivotMotor.getPosition().getValueAsDouble());
    }
}
