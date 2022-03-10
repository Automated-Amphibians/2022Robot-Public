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
            rollIn();
        } else if (OI.getInstance().armController.getRawButton(6)) {
            rollOut();
        } else {
            stop();
        }
    }


    public void rollOut() {
        intakeMotor.set(ControlMode.PercentOutput, 0.75);
    }

    public void rollIn() {
        intakeMotor.set(ControlMode.PercentOutput, -0.75);
    }

    public void stop() {
        intakeMotor.set(ControlMode.PercentOutput, 0);
    }
}