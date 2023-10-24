package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class Shooter extends SubsystemBase {
    CANSparkMax motorOne = new CANSparkMax(Constants.ShooterConstants.kShooterMotorOne, MotorType.kBrushless);
    CANSparkMax motorTwo = new CANSparkMax(Constants.ShooterConstants.kShooterMotorTwo, MotorType.kBrushless);

    public Shooter() {
        motorTwo.setInverted(true);
    }

    public void stop() {
        motorOne.set(0);
        motorTwo.set(0);
    }

    public void setSpeed(double speed) {
        motorOne.set(speed);
        motorTwo.set(speed);
    }
}
