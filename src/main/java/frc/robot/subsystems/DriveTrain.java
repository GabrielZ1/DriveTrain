package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.TalonFXControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonFX;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class DriveTrain extends SubsystemBase {
    TalonFX motorFrontLeft = new TalonFX(Constants.DriveTrainConstants.kDriveTrainMotorFrontLeft);
    TalonFX motorFrontRight = new TalonFX(Constants.DriveTrainConstants.kDriveTrainMotorFrontRight);
    TalonFX motorRearLeft = new TalonFX(Constants.DriveTrainConstants.kDriveTrainMotorRearLeft);
    TalonFX motorRearRight = new TalonFX(Constants.DriveTrainConstants.kDriveTrainMotorRearRight);

    public DriveTrain() {
        motorFrontRight.setInverted(true);
        motorRearRight.setInverted(true);
    }

    public void stop() {
        motorFrontLeft.set(TalonFXControlMode.PercentOutput, 0);
        motorRearLeft.set(TalonFXControlMode.PercentOutput, 0);
        motorFrontRight.set(TalonFXControlMode.PercentOutput, 0);
        motorRearRight.set(TalonFXControlMode.PercentOutput, 0);
    }

    public void setLeftSpeed(double speed) {
        motorFrontLeft.set(TalonFXControlMode.PercentOutput, speed);
        motorRearLeft.set(TalonFXControlMode.PercentOutput, speed);
    }

    public void setRightSpeed(double speed) {
        motorFrontRight.set(TalonFXControlMode.PercentOutput, speed);
        motorRearRight.set(TalonFXControlMode.PercentOutput, speed);
    }

    public double getPosition() {
        return motorFrontRight.getSelectedSensorPosition() * 2.75 * Math.PI / 39.37;

    }

    @Override
    public void periodic() {
        System.out.println(motorFrontRight.getSelectedSensorPosition());
    }

}
