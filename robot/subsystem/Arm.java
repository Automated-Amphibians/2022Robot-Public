package frc.robot.subsystem;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import com.revrobotics.jni.CANSparkMaxJNI;

public class Arm extends SubsystemBase{
    public CANSparkMax armMotor;
    public RelativeEncoder armEncoder;
    

    private static Arm instance = null;
    public static Arm getInstance() {
        if (instance == null) {
            instance = new Arm();
        }
        return instance;
    }

    public Arm() {
        armMotor = new CANSparkMax(6, MotorType.kBrushless);
        armEncoder = armMotor.getEncoder();
        //armEncoder.setPositionConversionFactor(1/16);
    }

    public void armUp() {
        // armMotor.set(0.1); ?
        armMotor.
    }
}
