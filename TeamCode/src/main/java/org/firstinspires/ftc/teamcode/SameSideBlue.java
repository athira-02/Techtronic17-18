package org.firstinspires.ftc.teamcode;

import android.util.Log;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.Func;
import org.firstinspires.ftc.robotcore.external.navigation.Acceleration;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.robotcore.external.navigation.Position;
import org.firstinspires.ftc.robotcore.external.navigation.Velocity;

import java.util.Locale;


/**
 * The is the autonomous program for the Same Side Blue corner of the playing field
 * Same Side Blue means that it is on the BLUE ALLIANCE and the cryptobox and jewel set
 * are located on the same side of the playing field
 * Created by Athira.
 */

@Autonomous (name="SameSideBlue", group="Qualifier")
public class SameSideBlue extends LinearOpMode {
    //Hardware
    RobotHardware robot = new RobotHardware();
    RobotControl robotControl;
    SensorControl sensorControl;

    // Gyro states used for updating telemetry
    Orientation angles;
    Acceleration gravity;



    private ElapsedTime runtime = new ElapsedTime();

    //Speed Constants
    final double DRIVE_SPEED = 0.6;
    final double ALIGN_SPEED = 0.3;
    final double TURN_SPEED = 0.3;
    static final double UP_LIFT_SPEED = 0.5;

    //Encoder Constants
    final double COUNTS_PER_MOTOR_REV = 1120;
    final double DRIVE_GEAR_REDUCTION = 1.0;
    final double WHEEL_DIAMETER_INCHES = 4.0;
    final double COUNTS_PER_INCH = (COUNTS_PER_MOTOR_REV * DRIVE_GEAR_REDUCTION) /
            (WHEEL_DIAMETER_INCHES * 3.1415);

    //Other Variables
    final String myColor = "BLUE";
    double forwardDistance;
    double distanceToLeftWall;

    String vuMark = "CENTER";
    public void runOpMode() {

        robot.init(hardwareMap);

        telemetry.addData("Status", "Ready to run");
        telemetry.update();

        waitForStart();

        //Adding the robotControls and sensorControls
        robotControl = new RobotControl(hardwareMap, robot );
        sensorControl = new SensorControl(hardwareMap, robot);

        runtime.reset();

        smacker();

        vuMarkAndSetParameters();

        //Grab the glyph and move the lift up
        robotControl.closeHand2();
        sleep(100);
        runtime.reset();
        while(runtime.seconds() < 0.5)
        {
            robotControl.liftUp(UP_LIFT_SPEED);
        }
        robotControl.liftStop();

        // Move Forward
        runtime.reset();
        robotControl.changeToEncoderMode();
        encoderDrive(DRIVE_SPEED, forwardDistance, forwardDistance, 5.0);  // S1: Forward 12 Inches with 5 Sec timeout
        sleep(500);

        // Turning Left
        composeTelemetry();
        telemetry.update();
        robot.imu.startAccelerationIntegration(new Position(), new Velocity(), 1000);
        turn90(6.0);

        sleep(500);

        sideWaysAlign();

        turn90(4.0);

        placeGlyphAndPark();
    }


    /**
     * ENCODER METHOD!
     * @param speed at which the robot will move
     * @param leftInches - the inches to move forward or backwards.  Negative if backwards. Should be the same as rightInches
     * @param rightInches- the inches to move forward or backwards. Negative if backwards.  Should be the same as leftInches
     * @param timeoutS
     */
    public void encoderDrive(double speed, double leftInches, double rightInches, double timeoutS) {
        int newBackLeftTarget;
        int newBackRightTarget;
        int newFrontLeftTarget;
        int newFrontRightTarget;

        // Ensure that the opmode is still active
        if (opModeIsActive()) {

            // Determine new target position, and pass to motor controller
            newBackLeftTarget = robotControl.getBackLeftWheelCurrentPosition() + (int)(leftInches * COUNTS_PER_INCH);
            newBackRightTarget = robotControl.getBackRightWheelCurrentPosition() + (int)(rightInches * COUNTS_PER_INCH);
            newFrontLeftTarget = robotControl.getFrontLeftWheelCurrentPosition() + (int)(leftInches * COUNTS_PER_INCH);
            newFrontRightTarget = robotControl.getFrontRightWheelCurrentPosition() + (int)(rightInches * COUNTS_PER_INCH);


            robotControl.setBackLeftWheelTargetPosition(newBackLeftTarget);
            robotControl.setBackRightWheelTargetPosition(newBackRightTarget);
            robotControl.setFrontLeftWheelPosition(newFrontLeftTarget);
            robotControl.setFrontRightWeelPosition(newFrontRightTarget);

            // Turn On RUN_TO_POSITION
            robotControl.setMode(DcMotor.RunMode.RUN_TO_POSITION);


            // reset the timeout time and start motion.
            runtime.reset();
            robotControl.setBackLeftWheelPower(Math.abs(speed));
            robotControl.setBackRightWheelPower(Math.abs(speed));
            robotControl.setFrontLeftWheelPower(Math.abs(speed));
            robotControl.setFrontRightWheelPower(Math.abs(speed));


            // keep looping while we are still active, and there is time left, and both motors are running.
            while (opModeIsActive() &&
                    (runtime.seconds() < timeoutS) &&
                    (robotControl.isBackLeftWheelBusy() && robotControl.isBackRightWheelBusy() &&
                            robotControl.isFrontLeftWheelBusy() && robotControl.isFrontRightWheelBusy())) {

                // Display it for the driver.
                telemetry.addData("Path1",  "Running to %7d :%7d : %7d :%7d",
                        newBackLeftTarget,  newBackRightTarget, newFrontLeftTarget, newFrontRightTarget);
                telemetry.addData("Path2",  "Running at %7d :%7d : %7d : %7d",
                        robotControl.getBackLeftWheelCurrentPosition(),
                        robotControl.getBackRightWheelCurrentPosition(),
                        robotControl.getFrontLeftWheelCurrentPosition(),
                        robotControl.getFrontRightWheelCurrentPosition());
                telemetry.update();
            }

            // Stop all motion;
            robotControl.stopMoving();

            // Turn off RUN_TO_POSITION
            robotControl.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        }
    }

    /**
     * Gyro Method
     * This method is used to get the current orientation from the gyro.  It needs to be called in the beginning of a turn
     * This code is from the sample gyro code
     */
    public void composeTelemetry() {

        telemetry.addAction(new Runnable()
        { @Override public void run()
        {
            angles   = robot.imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
            gravity  = robot.imu.getGravity();
        }
        });

        telemetry.addLine()
                .addData("status", new Func<String>() {
                    @Override public String value() {
                        return robot.imu.getSystemStatus().toShortString();
                    }
                })
                .addData("calib", new Func<String>() {
                    @Override public String value() {
                        return robot.imu.getCalibrationStatus().toString();
                    }
                });

        telemetry.addLine()
                .addData("heading", new Func<String>() {
                    @Override public String value() {
                        return formatAngle(angles.angleUnit, angles.firstAngle);
                    }
                })
                .addData("roll", new Func<String>() {
                    @Override public String value() {
                        return formatAngle(angles.angleUnit, angles.secondAngle);
                    }
                })
                .addData("pitch", new Func<String>() {
                    @Override public String value() {
                        return formatAngle(angles.angleUnit, angles.thirdAngle);
                    }
                });

        telemetry.addLine()
                .addData("grvty", new Func<String>() {
                    @Override public String value() {
                        return gravity.toString();
                    }
                })
                .addData("mag", new Func<String>() {
                    @Override public String value() {
                        return String.format(Locale.getDefault(), "%.3f",
                                Math.sqrt(gravity.xAccel*gravity.xAccel
                                        + gravity.yAccel*gravity.yAccel
                                        + gravity.zAccel*gravity.zAccel));
                    }
                });
    }

    /**
     * The following two methods are used in the method composeTelemetry
     * They are need to format the angle in degrees so that it is comprehensive
     */
    String formatAngle(AngleUnit angleUnit, double angle) {
        return formatDegrees(AngleUnit.DEGREES.fromUnit(angleUnit, angle));
    }

    String formatDegrees(double degrees){
        return String.format(Locale.getDefault(), "%.1f", AngleUnit.DEGREES.normalize(degrees));
    }


    /**
     * VuMark
     * This methods reads the vuMark by calling the method in sensorControl
     * It identifies the vuMark as either left, center, or right and accordingly sets the parameters for
     * the variables forwardDistance and distanceToLeftWall
     * The values and parameters to set were determined through experimentation and by using the distance sensor to
     * measure the distance to the left-wall at each of the columns of the cryptobox
     * The forwardDistance is in inches and the distanceToLeftWall is in centimeters.
     */
    public void vuMarkAndSetParameters()
    {
        //Reading vuMark
        vuMark = sensorControl.readVuMark();

        //Setting the Parameter
        if (vuMark.equalsIgnoreCase("CENTER"))
        {
            forwardDistance = 40;
            distanceToLeftWall = 118;
        }
        else if (vuMark.equalsIgnoreCase("LEFT"))
        {
            forwardDistance = 32;
            distanceToLeftWall = 103;
        }
        else
        {
            forwardDistance = 48;
            distanceToLeftWall = 156;
        }
    }


    /**
     * This method is used to turn 90 degrees to the left
     * This method oscilates left and right until the angle is within a set margin of error
     * from the desired angle
     * @param maxSeconds This is the maximum time in seconds which the robot can take to get to this angle.
     * This ensures that the robot does not go back and forth for too long and does not finish all the steps
     * within the 30 second time limit for the autonomous mode.
     */
    public void turn90(double maxSeconds)
    {
        double angle = 90;
        double marginOfError = 2;
        boolean arrivedAtAngle = false;
        double speedReduction = 15;
        double speed = TURN_SPEED;
        boolean speedReduced = false;


        while (arrivedAtAngle == false && runtime.seconds() < maxSeconds)
        {
            telemetry.addData("angles.firstAngle is :", angles.firstAngle);
            Log.d("SameSideBlue", "First Angle is: " + angles.firstAngle);

            if (angles.firstAngle < (angle + marginOfError) && angles.firstAngle > (angle - marginOfError))
            {
                arrivedAtAngle = true;
                Log.d("SameSideBlue", "Reached angle  " + angles.firstAngle);
                Log.d("SameSideBlue", "Arrived at Angle?: " + arrivedAtAngle);
            }

            if(angles.firstAngle < (angle + speedReduction) && angles.firstAngle > (angle - speedReduction) && speedReduced == false)
            {
                Log.d("SameSideBlue", "Reducing Speed");

                //reduce the speed as it gets close to the angle
                speed = speed - 0.1;

                //to ensure that it does not continue to decrement the speed
                speedReduced = true;
            }

            if(arrivedAtAngle == false)
            {
                Log.d("SameSideBlue", "Arrived at Angle?: " + arrivedAtAngle);
                if (angles.firstAngle < angle) {
                    robotControl.turnLeft(speed);
                    Log.d("SameSideBlue", "Turned Left ");
                } else if (angles.firstAngle > angle) {
                    robotControl.turnRight(speed);
                    Log.d("SameSideBlue", "Turned Right ");
                } else {
                    robotControl.stopMoving();
                }

                sleep(250);
                robotControl.stopMoving();
            }

            telemetry.update();
        }

        //Display that the turn is completed
        telemetry.addData("Turned the desired angle", angles.firstAngle);
        telemetry.update();
        Log.d("SameSideBlue", "Turned the desired angle " + angles.firstAngle);
    }


    /**
     * This method is used to align with the correct column of the cryptobox.  It utilizes the
     * parameter distanceToLeftWall which was set in the method vuMarkAndSetParameters
     * This is the target distance from the left-wall that the robot should be.
     * The robot moves left and right until it is within the set margin of error of the target
     * distance form the left wall
     */
    public void sideWaysAlign()
    {
        boolean alignedWithCryptobox = false;
        double distanceMarginOfError = 3;

        runtime.reset();
        telemetry.addData("Robot Status: ", "About to align with cyrptobox");
        telemetry.update();
        Log.d("SameSideBlue", "Robot Status: " + "About to align with cyrptobox");
        Log.d("SameSideBlue", "distanceToLeftWall is  " + distanceToLeftWall);
        Log.d("SameSideBlue", "distanceMarginOfError is  " + distanceMarginOfError);

        while (alignedWithCryptobox == false && runtime.seconds() < 4)
        {
            ElapsedTime sideWaysTime = new ElapsedTime();

            //Finding the distance: Takes the distance 10 times and finds the average to be more precise
            double distanceSum = 0;
            for (int i = 0; i < 10; i++)
            {
                double testDistance = sensorControl.getDistance(myColor);

                distanceSum = distanceSum + testDistance;
            }
            double distance = distanceSum / 10;
            telemetry.addData("cm", "%.2f cm", distance);
            Log.d("SameSideBlue", "distance is " + distance);

            /*
            If the distance is less than 100 cm, then we are sensing the balancing stone rather thatn the left wall
            The from the right edge of the balancing stone to the left wall is 90 cm.  Therfore, we add this to the
            sensed distance to get the accurate distance from the left wall.
            */

            if (distance < 100)
            {
                distance = distance + 90;
                telemetry.addData("Distance: ", distance);
                telemetry.update();
                Log.d("SameSideBlue", "distance is " + distance);

            }

            if (distance < distanceToLeftWall + distanceMarginOfError && distance > distanceToLeftWall - distanceMarginOfError)
            {
                telemetry.addData("Status: ", "Alligned!");
                telemetry.update();
                Log.d("SameSideBlue", "Status:  Alligned");
                alignedWithCryptobox = true;
            }
            else
            {
                telemetry.addData("Status: ", "Not Alligned :(");
                telemetry.update();
                Log.d("SameSideBlue", "Status:  Not Alligned :(");

                if (distance > 100)
                {
                    sideWaysTime.reset();
                    if (distance > distanceToLeftWall)
                    {
                        //The robot moves to the left for 0.1 seconds
                        while (sideWaysTime.seconds() < 0.1)
                        {
                            robotControl.sideWaysLeft(ALIGN_SPEED);
                            telemetry.addData("Status: ", "Moving left");
                            telemetry.update();
                            Log.d("SameSideBlue", "Status:  Moving left");
                        }

                    }
                    else
                    {
                        //The robot moves to the right for 0.1 seconds
                        while (sideWaysTime.seconds() < 0.1)
                        {
                            robotControl.sideWaysRight(ALIGN_SPEED);
                            telemetry.addData("Status: ", "Moving right");
                            telemetry.update();
                            Log.d("SameSideBlue", "Status:  Moving right");
                        }
                    }
                }
            }

            robotControl.stopMoving();
        }
    }


    /**
     * This method is used at the end of the autonomous mode to place the glyph in the cyrptobox
     * It does this by first moving forward, then backwards.
     * it then closes the gripper and moves forward again to push teh glyph in the cryptobox.
     * Finally, it moves backwards to ensure that the robot is not touching the glyph and it is still parked in
     * the parking zone
     */
    public void placeGlyphAndPark()
    {
        //move forward to place glyph in cryptobox
        runtime.reset();
        robotControl.changeToEncoderMode();
        encoderDrive(DRIVE_SPEED, 15, 15, 5);
        sleep(250);

        //release glyph
        robotControl.openHand2();

        runtime.reset();
        robotControl.changeToEncoderMode();
        encoderDrive(DRIVE_SPEED, -8, -8, 5);
        sleep(250);

        runtime.reset();

        robotControl.changeToEncoderMode();
        encoderDrive(DRIVE_SPEED, 12 , 12, 5);
        sleep(250);


        runtime.reset();

        robotControl.changeToEncoderMode();
        encoderDrive(DRIVE_SPEED, -8 , -8, 5);
        sleep(250);


        runtime.reset();


    }


    /**
     * This is the code for the jewel smacker
     * The robot brings the jewel extension down in stages in order to avoid having the problem of
     * the smacker crashing into the floor or the wall.  The color distance sensor at the end of the smacker
     * sensed both the color and the distance.
     * If the sensor cannot identify the color OR the distance sensed is "NaN" (tested by seeing if the distance
     * is less than 20), the smacker moves straight back up instead and does not risk smacking the wrong
     * color jewel.
     * This ensures that unlesss we are 100% sure of smacker the correct jewel, we do not smack any jewel
     */
    public void smacker()
    {
        //Bringing the smacker down
        robotControl.moveSmacker(0.4);

        robotControl.moveStickShoulder(0.4);
        sleep(750);
        robotControl.moveStickElbow(0.4);
        sleep(750);

        robotControl.moveStickShoulder(0.6);
        sleep(750);
        robotControl.moveStickElbow(0.2);

        sleep(750);

        robotControl.moveStickShoulder(1);

        sleep(750);

        robotControl.moveStickElbow(0);

        telemetry.addData("Moved Stick Down: " ,"");
        sleep(500);
        telemetry.update();

        //Getting the distance
        double distance = sensorControl.getSmackerDistance();
        telemetry.addData("Distance is: " , distance);
        telemetry.update();
        sleep(750);

        //Testing if the color sensed is red
        if(sensorControl.isRed() && distance < 20)
        {
            telemetry.addData("I see: " , "Red");
            telemetry.update();
            sleep(500);
            robotControl.smackRight();
            telemetry.addData("Smacked: " , "Right");
            sleep(750);
        }
        //Testing if the color sensed is blue
        else if(sensorControl.isBlue() && distance < 20)
        {
            telemetry.addData("I see: " , "Blue");
            telemetry.update();
            sleep(500);
            robotControl.smackLeft();
            telemetry.addData("Smacked: " , "Left");
            sleep(750);
        }
        //If not able to sense the color OR the distance is "NaN'
        else
        {
            telemetry.addData("I can't sense the color", "");
            telemetry.update();
        }

        telemetry.update();

        //Bring the smacker back up
        robotControl.stickUp();
        sleep(750);
        robotControl.moveSmacker(0.4);

        telemetry.addData("I have returned to the starting position: " , "");
        telemetry.update();
    }
}
