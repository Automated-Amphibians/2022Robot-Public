package frc.robot.subsystem;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;


public class OI {
    public double robotVelocity;
    public double robotTurnSpeed;
    
    public JoystickButton leftBumper;
    public JoystickButton rightBumper;
    
    public Joystick drivetrainController;
    public Joystick armController;

    private static OI instance = null;
    public static OI getInstance() {
        if (instance == null)
            instance = new OI();

        return instance;
    }


    public OI(){
        drivetrainController = new Joystick(0);
        armController = new Joystick(1);
        
        leftBumper = new JoystickButton(armController, 5);
        rightBumper = new JoystickButton(armController, 6);
    }

    public void updateInputs() {
        robotVelocity = drivetrainController.getRawAxis(1) * -0.8;
        robotTurnSpeed = drivetrainController.getRawAxis(4) * 0.5;
    }

}
