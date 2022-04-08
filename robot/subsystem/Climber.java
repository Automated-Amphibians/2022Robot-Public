package frc.robot.subsystem;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

// import com.ctre.phoenix.motorcontrol.ControlMode;
// import com.ctre.phoenix.motorcontrol.can.VictorSPX;

public class Climber extends SubsystemBase{
    public CANSparkMax rotationMotor;
    // public VictorSPX extensionMotor;
    

    private static Climber instance = null;

    public static Climber getInstance() {
        if (instance == null) {
            instance = new Climber();
        }
        return instance;
    }

    public Climber() {
        rotationMotor = new CANSparkMax(11, MotorType.kBrushless);
        // extensionMotor = new VictorSPX(6);    // Still needs to set the motor ID
    }

    public void climberPeriodic() {
        /*
        if (OI.getInstance().armController.getPOV() == 0) {
            // Extend the climber
            extensionMotor.set(ControlMode.PercentOutput, demand);    // Needs to see if positive demand will extend or shrink the arm
        } else if (OI.getInstance().armController.getPOV() == 180) {
            // Shrink the climber
            extensionMotor.set(ControlMode.PercentOutput, -demand);
        } else {
            // Stop the climber extension
            extensionMotor.set(ControlMode.PercentOutput, 0);
        }

        */

        if (OI.getInstance().armController.getPOV() == 90) {
            // Rotate climber toward the robot
            rotationMotor.set(0.1);    // Still needs to see which way the motor spins for positive demand
        } else if (OI.getInstance().armController.getPOV() == 270) {
            // Rotate the climber away from the robot
            rotationMotor.set(-0.1);
        } else {
            // Stop rotating
            rotationMotor.set(0);
        }

    }
}
