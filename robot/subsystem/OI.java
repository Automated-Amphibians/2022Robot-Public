package frc.robot.subsystem;

import edu.wpi.first.wpilibj.Joystick;


public class OI {
    public double robotVelocity;
    public double robotTurnSpeed;
    
    public Joystick drivetrainController;
    public Joystick armController;

    public double driveSpeedLimit = 0.65;
    public double turnSpeedLimit = 0.5;

    private boolean aWasPressed = false;
    private boolean bWasPressed = false;
    private boolean xWasPressed = false;
    private boolean yWasPressed = false;

    private static OI instance = null;
    public static OI getInstance() {
        if (instance == null) {
            instance = new OI();
        }
        return instance;
    }



    public OI(){
        drivetrainController = new Joystick(0);
        armController = new Joystick(1);
    }



    public void updateInputs() {
        double drivetrainAxis = drivetrainController.getRawAxis(1) ;
        drivetrainAxis = Math.abs(drivetrainAxis) < 0.075 ? 0 : drivetrainAxis;

        double turnAxis = drivetrainController.getRawAxis(4) ;
        turnAxis = Math.abs(turnAxis) < 0.075 ? 0 : turnAxis;

        robotVelocity = drivetrainAxis * driveSpeedLimit * -1;
        robotTurnSpeed = turnAxis * turnSpeedLimit;
    }

    public void updateSpeedLimits() {
        if (drivetrainController.getRawButton(4)) {
            if (yWasPressed == false) {
                yWasPressed = true;
                if (driveSpeedLimit < 1) {
                    driveSpeedLimit += 0.05;
                }
            }
        } else {
            yWasPressed = false;
        }


        if (drivetrainController.getRawButton(1)) {
            if (aWasPressed == false) {
                aWasPressed = true;
                if (driveSpeedLimit > 0) {
                    driveSpeedLimit -= 0.05;
                }
            }
        } else {
            aWasPressed = false;
        }


        if (drivetrainController.getRawButton(2)) {
            if (bWasPressed == false) {
                bWasPressed = true;
                if (turnSpeedLimit < 1) {
                    turnSpeedLimit += 0.05;
                }
            }
        } else {
            bWasPressed = false;
        }


        if (drivetrainController.getRawButton(3)) {
            if (xWasPressed == false) {
                xWasPressed = true;
                if (turnSpeedLimit > 0.5) {
                    turnSpeedLimit -= 0.05;
                }
            }
        } else {
            xWasPressed = false;
        }
    }
}