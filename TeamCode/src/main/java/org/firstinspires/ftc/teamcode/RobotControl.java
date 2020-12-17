package org.firstinspires.ftc.teamcode;


import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;



/**
 * This class contains various methods which are used to control the robot
 * Created by Athira on 11/9/2017.
 * Last Updated by Athira on 1/1/18
 */

public class RobotControl
{

    HardwareMap hardwareMap = null;
    public static RobotHardware robot = null;
    //Constructor
    public RobotControl(HardwareMap map, RobotHardware input_robot)
    {
        hardwareMap = map;
        robot = input_robot;
    }


    /**
     *-----------------------------------------------------------------------------------
     * HAND CONTROLS
     * This is used for the gripper on the vertical lift which we use to pick up glyphs
     * ----------------------------------------------------------------------------------
     */

    /**
     * Closes the hand by setting the position of the right and left servo to 90 degrees or 0.5;
     */
    public void closeHand1()
    {
        robot.rightHand1.setPosition(0);
        robot.leftHand1.setPosition(1);
    }

    public void openHand1()
    {
        robot.rightHand1.setPosition(0.75);
        robot.leftHand1.setPosition(0.25);
    }

    public void midHand1()
    {
        robot.rightHand1.setPosition(0.55);
        robot.leftHand1.setPosition(0.45);
    }

    public void closeHand2()
    {
        robot.rightHand2.setPosition(1);
        robot.leftHand2.setPosition(0);
    }

    public void openHand2()
    {
        robot.rightHand2.setPosition(0.25);
        robot.leftHand2.setPosition(0.75);
    }

    public void midHand2()
    {
        robot.rightHand2.setPosition(0.45);
        robot.leftHand2.setPosition(0.55);
    }

    /**
     *-----------------------------------------------------------------------------------
     * WHEEL CONTROLS
     * This is used to move the robot around.  The robot can move in all directions with
     * the use of Mecanum wheels
     * ----------------------------------------------------------------------------------
     */

    /**
     * Moves the robot forward by giving power to all four wheels
     * @param speed at which the wheel motors will turn
     */
    public void moveForward(double  speed)
    {
        robot.frontLeftWheel.setPower(speed);
        robot.frontRightWheel.setPower(speed);
        robot.backLeftWheel.setPower(speed);
        robot.backRightWheel.setPower(speed);
    }

    /**
     * Moves the robot backwards by giving negative power to all four wheels
     * @param speed at which the wheel motors will turn
     */
    public void moveBackwards(double speed)
    {
        robot.frontLeftWheel.setPower(-speed);
        robot.frontRightWheel.setPower(-speed);
        robot.backLeftWheel.setPower(-speed);
        robot.backRightWheel.setPower(-speed);
    }

    /**
     * Rotates the robot to the right by giving the left wheels power and the right
     * wheels negative power
     * @param speed at which the wheel motors will turn
     */
    public void turnRight (double speed)
    {
        robot.frontLeftWheel.setPower(speed);
        robot.backLeftWheel.setPower(speed);
        robot.frontRightWheel.setPower(-speed);
        robot.backRightWheel.setPower(-speed);
    }

    /**
     * Rotates the robot to the left by giving the right wheels power and the left
     * wheels negative power
     * @param speed at which the wheel motors will turn
     */
    public void turnLeft (double speed)
    {
        robot.frontLeftWheel.setPower(-speed);
        robot.backLeftWheel.setPower(-speed);
        robot.frontRightWheel.setPower(speed);
        robot.backRightWheel.setPower(speed);
    }

    /**
     * Moves the robot to the right laterally.  The front left wheel and back right wheel
     * are given negative power and the front right wheel and back left wheel are given power.
     * @param speed at which the wheel motors will turn
     */
    public void sideWaysRight (double speed)
    {
        robot.frontLeftWheel.setPower(-speed);
        robot.backLeftWheel.setPower(speed);
        robot.frontRightWheel.setPower(speed);
        robot.backRightWheel.setPower(-speed);
    }

    /**
     * Moves the robot to the left laterally.  The front right wheel and back left wheel
     * are given negative power and the front left wheel and back right wheel are given power.
     * @param speed at which the wheel motors will turn
     */
    public void sideWaysLeft (double speed)
    {
        robot.frontLeftWheel.setPower(speed);
        robot.backLeftWheel.setPower(-speed);
        robot.frontRightWheel.setPower(-speed);
        robot.backRightWheel.setPower(speed);
    }

    /**
     * Stops the robot by setting the power of all wheels to 0
     */
    public void stopMoving()
    {
        robot.frontLeftWheel.setPower(0);
        robot.frontRightWheel.setPower(0);
        robot.backLeftWheel.setPower(0);
        robot.backRightWheel.setPower(0);
    }




    /**
     *-----------------------------------------------------------------------------------
     * VERTICAL LIFT CONTROLS
     * This is used to move the vertical linear lift which is our glyph handler.
     * There is just one motor to control the lift which will pull/release the String to
     * lift/lower the gripper
     * ----------------------------------------------------------------------------------
     */

    /**
     * Pulls the gripper up by giving power to the lift motor.
     * @param speed at which the lift motor will turn
     */
    public void liftUp(double speed)
    {
        robot.lift.setPower(speed);
    }

    /**
     * Brings the gripper down by giving negative power to the lift motor.
     * @param speed at which the lift motor will turn
     */
    public void liftDown(double speed)
    {
        robot.lift.setPower(-speed);
    }

    /**
     * Stops the lift at its current position by setting the power to 0.
     */
    public void liftStop()
    {
        robot.lift.setPower(0);
    }




    /**
     *-----------------------------------------------------------------------------------
     * JEWEL SMACKER CONTROLS
     * This is an extension composed of two wooden beams.  There are two servos that
     * control the system.  The servo at the base of the system which connects to the rest
     * of the robot is called the "shoulder" while the servo between the two wooden beams
     * is called the "elbow"
     * ----------------------------------------------------------------------------------
     */

    /**
     * Brings the stick (smacker system) up to its starting position.
     * This is done by setting the position of the shoulder to 0 and the position of the
     * elbow to 0.
     */
    public void stickUp()
    {
        moveStickShoulder(0);
        moveStickElbow(1);
    }

    /**
     * Brings the stick (smacker system) up to its extended position.
     * This is done by setting the position of the shoulder to 1 and the position of the
     * elbow to 0.6.
     */
    public void stickDown()
    {
        moveSmacker(0.2);
        moveStickElbow(0.6);
        moveStickShoulder(1);
    }

    /**
     * Moves the stick shoulder servo to the position passed to the method
     * @param position to which the servo should turn to
     */
    public void moveStickShoulder(double position)
    {

        robot.stickShoulder.setPosition(position);
    }

    /**
     * Moves the stick elbow servo to the position passed to the method
     * @param position to which the servo should turn to
     */
    public void moveStickElbow(double position)
    {

        robot.stickElbow.setPosition(position);
    }


    /**
     * Moves the smacker servo to the position passed to the method
     * @param position to which the servo should turn to
     */
    public void moveSmacker(double position)
    {
        robot.smacker.setPosition(position);

    }

    /**
     * The following two methods are used to smack the jewel either to the right or to the left
     * by setting the position of the smacker servo to either 0 or 1
     */
    public void smackRight()
    {
        moveSmacker(1);
    }

    public void smackLeft()
    {
        moveSmacker(0);
    }





    /**
     *-----------------------------------------------------------------------------------
     * ENCODER WHEEL CONTROLS
     * This is used during the autonomous mode to ensure that the robot moves to accurate
     * distances.
     * There are many different methods that are used when running with encoder.
     * ----------------------------------------------------------------------------------
     */

    /**
     * This method changes the mode the wheels are running in to passed run mode.
     * Some such run modes include RUN_USING_ENCODER and RUN_TO_POSITION, etc.
     * @param mode which the wheels should be set to running in
     */
    public void setMode(DcMotor.RunMode mode)
    {
        robot.backLeftWheel.setMode(mode);
        robot.backRightWheel.setMode(mode);
        robot.frontLeftWheel.setMode(mode);
        robot.frontRightWheel.setMode(mode);
    }

    /**
     * The follow four methods are used to individually set the power for each wheel
     * This is needed when using the encoder
     * @param speed the power that the motor will be set to
     */
    public void setBackLeftWheelPower(double speed) {robot.backLeftWheel.setPower(speed);}
    public void setBackRightWheelPower(double speed) {robot.backRightWheel.setPower(speed);}
    public void setFrontLeftWheelPower(double speed) {robot.frontLeftWheel.setPower(speed);}
    public void setFrontRightWheelPower(double speed)
    {
        robot.frontRightWheel.setPower(speed);
    }


    /**
     * The following four methods test each wheel if they are busy in that the wheel's
     * motor is powered and active
     * @return whether or not the wheel is busy where true means it is busy and false
     * means it is not busy
     */
    public boolean isBackLeftWheelBusy()
    {
        return robot.backLeftWheel.isBusy();
    }
    public boolean isBackRightWheelBusy()
    {
        return robot.backRightWheel.isBusy();
    }
    public boolean isFrontLeftWheelBusy()
    {
        return robot.frontLeftWheel.isBusy();
    }
    public boolean isFrontRightWheelBusy()
    {
        return robot.frontRightWheel.isBusy();
    }


    /**
     * Resets the encoders of all the wheels' motors and changes them to encoder mode
     */
    public void changeToEncoderMode()
    {
        robot.backRightWheel.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.backRightWheel.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        robot.backLeftWheel.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.backLeftWheel.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        robot.frontRightWheel.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.frontRightWheel.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        robot.frontLeftWheel.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.frontLeftWheel.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    }

    /**
     * The following four methods gets the current position of the wheels' motors
     * @return an integer value representing the current position of the wheel's motor
     */
    public int getBackRightWheelCurrentPosition() {return robot.backRightWheel.getCurrentPosition();}
    public int getBackLeftWheelCurrentPosition() {return robot.backLeftWheel.getCurrentPosition();}
    public int getFrontRightWheelCurrentPosition() {return robot.frontRightWheel.getCurrentPosition();}
    public int getFrontLeftWheelCurrentPosition() {return robot.frontLeftWheel.getCurrentPosition();}


    /**
     * The following four methods sets the target position to which each wheel's motor should
     * run to.
     */
    public void setBackLeftWheelTargetPosition(int position) {robot.backLeftWheel.setTargetPosition(position);}
    public void setBackRightWheelTargetPosition(int position) {robot.backRightWheel.setTargetPosition(position);}
    public void setFrontLeftWheelPosition(int position) {robot.frontLeftWheel.setTargetPosition(position);}
    public void setFrontRightWeelPosition(int position) {robot.frontRightWheel.setTargetPosition(position);}


    /**
     *-----------------------------------------------------------------------------------
     * RELIC EXTENSION CONTROLS
     * This is used during the teleOp mode.
     * This includes methods to extend the relic arm.
     * ----------------------------------------------------------------------------------
     */

    /**
     * The next three methods are used to control the extension of the relic arm by setting
     * either a positive speed (extending the arm), a negative speed (retracting the arm) or setting
     * the power to 0
     * @param speed at which the DC motor for the extension to turn
     */
    public void extendRelicExtension (double speed)
    {
        robot.relicExtension.setPower(speed);
    }

    public void retractRelicExtension (double speed)
    {
        robot.relicExtension.setPower(-speed);
    }

    public void stopRelicExtension()
    {
        robot.relicExtension.setPower(0);
    }


    /**
     * This method sets the position of the wrist servo to a passed position
     * @param position to which te wrist servo is set to
     */
    public void moveWrist(double position)
    {
        robot.relicWrist.setPosition(position);
    }


    public void moveRelicHand(double position)
    {
        robot.relicClaw.setPosition(position);
    }

    /**
     * This function opens the claw of the relic by setting the position of the servo to 0
     */
    public void openRelicHand()
    {
        robot.relicClaw.setPosition(0);
    }

    /**
     * This function closese the claw of the relic by incrementally moving the servo's postion
     * to 1
     */
    public void closeRelicHand()
    {
        for(double i = 0 ; i <= 1 ; i = i + 0.001)
        {
            robot.relicClaw.setPosition(i);
        }
    }

    public void twistDown()
    {
        robot.twister.setPosition(0.15);
    }

    public void twistUp()
    {
        robot.twister.setPosition(0.9);
    }

    public void twist3()
    {
        robot.twister.setPosition(0.5);
    }

}



