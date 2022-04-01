package frc.robot.subsystem;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;


public class Intake extends SubsystemBase{
    public static TalonSRX intakeMotor;


    private static Intake instance = null;
    public static Intake getInstance() {
        if (instance == null) {
            instance = new Intake();
        }
        return instance;
    }

    public Intake() {
        intakeMotor = new TalonSRX(5);
    }


    public void intakePeriodic() {
        if (OI.getInstance().armController.getRawButton(5)) {
            rollIn(0.675);
        } else if (OI.getInstance().armController.getRawButton(6)) {
            rollOut(1);
        } else {
            stop();
        }
    }


    public void rollOut(double power) {
        intakeMotor.set(ControlMode.PercentOutput, power);
    }

    public void rollIn(double power) {
        intakeMotor.set(ControlMode.PercentOutput, -power);
    }

    public void stop() {
        intakeMotor.set(ControlMode.PercentOutput, 0);
    }
}