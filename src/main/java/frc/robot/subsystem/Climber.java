package frc.robot.subsystem;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import com.revrobotics.RelativeEncoder;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.VictorSPX;

public class Climber extends SubsystemBase{
    public CANSparkMax rotationMotor;
    public RelativeEncoder rMEncoder;
    public VictorSPX extensionMotor;

    public double climberDemand;
    

    private static Climber instance = null;

    public static Climber getInstance() {
        if (instance == null) {
            instance = new Climber();
        }
        return instance;
    }

    public Climber() {
        rotationMotor = new CANSparkMax(11, MotorType.kBrushless);
        rMEncoder = rotationMotor.getEncoder();
        extensionMotor = new VictorSPX(6);
    }

    public void resetEncoder() {
        rMEncoder.setPosition(0);
    }

    public void climberPeriodic() {
        climberDemand = OI.getInstance().armController.getRawAxis(1);    // Check if this works first
        climberDemand = Math.abs(climberDemand) < 0.01 ? 0 : climberDemand;
        System.out.println("Climber Demand: " + climberDemand);


        extensionMotor.set(ControlMode.PercentOutput, climberDemand);

        /*
        if (OI.getInstance().armController.getPOV() == 0) {
            // Extend the climber
            extensionMotor.set(ControlMode.PercentOutput, 0.3);
        } else if (OI.getInstance().armController.getPOV() == 180) {
            // Shrink the climber
            extensionMotor.set(ControlMode.PercentOutput, -0.3);
        } else if (OI.getInstance().armController.getRawButton(2)) {
            extensionMotor.set(ControlMode.PercentOutput, -0.9);
        } else {
            // Stop the climber extension
            extensionMotor.set(ControlMode.PercentOutput, 0);
        }
        */
        

        if (OI.getInstance().drivetrainController.getPOV() == 90) {
            // Rotate climber toward the robot
            rotationMotor.set(0.15);
        } else if (OI.getInstance().drivetrainController.getPOV() == 270 && rMEncoder.getPosition() > 0) {
            // Rotate the climber away from the robot
            rotationMotor.set(-0.1);
        } else {
            // Stop rotating
            rotationMotor.set(0);
        }
    }
}