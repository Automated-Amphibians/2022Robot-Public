package frc.robot.subsystem;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

import com.ctre.phoenix.motorcontrol.ControlMode;
//import com.ctre.phoenix.motorcontrol.ControlMode;
import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
//import com.revrobotics.jni.CANSparkMaxJNI;

public class Arm extends SubsystemBase{
    public CANSparkMax armMotor;
    public RelativeEncoder armEncoder;


    private static Arm instance = null;
    double power;
    double powerFactor;
    public static Arm getInstance() {
        if (instance == null) {
            instance = new Arm();
        }
        return instance;
    }

    public Arm() {
        armMotor = new CANSparkMax(10, MotorType.kBrushless);
        armEncoder = armMotor.getEncoder();
        armMotor.setIdleMode(IdleMode.kBrake);
       
    }

    public void armInit(){
        armEncoder.setPosition(0);
    }

    public void armPeriodic() {
        powerFactor = 0.25;
        power = -OI.getInstance().armController.getRawAxis(5) * powerFactor;
       

        if (OI.getInstance().armController.getRawButton(5)) {
            power = -0.2;
        } else if (armEncoder.getPosition() <= 4 && armEncoder.getPosition() > 0) {
            if (power < 0) {
                powerFactor = 0.025;
            } 
        } else if (armEncoder.getPosition() <= 0) {
            if (power < 0) {
                powerFactor = 0;
            }
        } else if (armEncoder.getPosition() >= 30) {
            if (power > 0) {
                power = 0;
            }
        }

        armMotor.set(power);
    }


    public void armUp() {
        // armMotor.set(0.1);
    }
}
