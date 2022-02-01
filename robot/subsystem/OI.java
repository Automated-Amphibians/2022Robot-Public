package frc.robot.subsystem;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;


public class OI {
 

    public OI(){
   
        public double robotVelocity;
        public double robotTurnSpeed;
    
        public JoystickButton leftBumper;
        public JoystickButton rightBumper;

        Joystick drivetrainController = new Joystick(0);
        Joystick armController = new Joystick(1);

        //------------------------------------------------------------------------------------------------------// 
        robotVelocity = drivetrainController.getRawAxis(1);
        robotTurnSpeed = drivetrainController.getRawAxis(2);
        //------------------------------------------------------------------------------------------------------//
        
        leftBumper = new JoystickButton(armController, 5);
        rightBumper = new JoystickButton(armController, 6);
    }


}
