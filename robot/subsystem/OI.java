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

    public double driveSpeedLimit = 0.8;
    public double turnSpeedLimit = 0.5;

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
        double drivetrainAxis = drivetrainController.getRawAxis(1) ;
        drivetrainAxis = Math.abs(drivetrainAxis) < 0.075 ? 0 : drivetrainAxis;

        double turnAxis = drivetrainController.getRawAxis(4) ;
        turnAxis = Math.abs(turnAxis) < 0.075 ? 0 : turnAxis;

        robotVelocity = drivetrainAxis * driveSpeedLimit * -1;
        robotTurnSpeed = turnAxis * turnSpeedLimit;


    }

    public void printSpeedLimits() {
        if (drivetrainController.getRawButton(1)) {
            System.out.println("Drive speed limit: " + driveSpeedLimit);
            System.out.println("Turn speed limit: " + turnSpeedLimit);
        }   
    }



}
