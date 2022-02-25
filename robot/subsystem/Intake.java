package frc.robot.subsystem;

import edu.wpi.first.wpilibj.motorcontrol.Talon;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;


public class Intake extends SubsystemBase{
    public static TalonSRX intakeMotor;

    public Intake() {
        intakeMotor = new TalonSRX(5);
    }

    public void rollOut() {
        intakeMotor.set(ControlMode.PercentOutput, 0.75);
    }

    public void rollIn() {
        intakeMotor.set(ControlMode.PercentOutput, -0.75);
    }

    public void stopRolling() {
        intakeMotor.set(ControlMode.PercentOutput, 0);
    }
}
