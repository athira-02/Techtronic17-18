package org.firstinspires.ftc.teamcode;

import android.util.Log;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
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
 * Created by Athira
 */
@Disabled
@Autonomous (name="BackUpDifferentSideRed", group="Qualifier")
public class BackUpDifferentSideRed extends LinearOpMode {
    //Hardware
    RobotHardware robot = new RobotHardware();
    RobotControl robotControl;
    SensorControl sensorControl;

    // State used for updating telemetry
    Orientation angles;
    Acceleration gravity;

    Orientation angles2;
    Acceleration gravity2;


    private ElapsedTime runtime = new ElapsedTime();
    final double DRIVE_SPEED = 0.6;
    final double ALIGN_SPEED = 0.3;
    final double TURN_SPEED = 0.3;
    static final double UP_LIFT_SPEED = 0.5;
    final double COUNTS_PER_MOTOR_REV = 1120;    // eg: TETRIX Motor Encoder
    final double DRIVE_GEAR_REDUCTION = 1.0;     // This is < 1.0 if geared UP
    final double WHEEL_DIAMETER_INCHES = 4.0;     // For figuring circumference
    final double COUNTS_PER_INCH = (COUNTS_PER_MOTOR_REV * DRIVE_GEAR_REDUCTION) /
            (WHEEL_DIAMETER_INCHES * 3.1415);

    final String myColor = "RED";

    //Other Variables
    double backwardDistance = -26;
    double sideWaysTime;


    public void runOpMode() {

        /*
         * Initialize the drive system variables.
         * The init() method of the hardware class does all the work here
         */
        robot.init(hardwareMap);

        //Ready to run
        //telemetry.log().clear();
        telemetry.addData("Status", "Ready to run");
        telemetry.update();

        //Wait for start
        waitForStart();

        //Adding the robotControls
        robotControl = new RobotControl(hardwareMap, robot );
        sensorControl = new SensorControl(hardwareMap, robot);

        runtime.reset();




        //Reading VuMark
        String vuMark = sensorControl.readVuMark();

        if (vuMark.equalsIgnoreCase("CENTER"))
        {

            sideWaysTime = 2; //this is 124 cm
        }
        else if (vuMark.equalsIgnoreCase("LEFT"))
        {

            sideWaysTime = 3; //this is 108 cm
        }
        else
        {

            sideWaysTime = 1; //this is 140 cm
        }


        //Grab the glyph
        robotControl.closeHand1();
        sleep(100);
        runtime.reset();

        while(runtime.seconds() < 1)
        {
            robotControl.liftUp(UP_LIFT_SPEED);
        }
        robotControl.liftStop();


        // NOW ACTUALLY MOVING
        // Move Forward
        runtime.reset();
        robotControl.changeToEncoderMode();
        encoderDrive(DRIVE_SPEED, backwardDistance, backwardDistance, 5.0);  // S1: Forward 12 Inches with 5 Sec timeout
        sleep(500);

        // Making a U turn

        // Set up our telemetry dashboard
        composeTelemetry();
        telemetry.update();
        // Start the logging of measured acceleration
        robot.imu.startAccelerationIntegration(new Position(), new Velocity(), 1000);

        double angle = 179;
        double marginOfError = 2;
        boolean arrivedAtAngle = false;



        // Loop and update the dashboard
        while (arrivedAtAngle == false && runtime.seconds() < 10)
        {
            telemetry.addData("angles.firstAngle is :", angles.firstAngle);
            Log.d("BackUpDifferentSideRed", "First Angle is: " + angles.firstAngle);

            if (angles.firstAngle < (angle + marginOfError) && angles.firstAngle > (angle - marginOfError))
            {
                arrivedAtAngle = true;
                Log.d("BackUpDifferentSideRed", "Reached angle  " + angles.firstAngle);
                Log.d("BackUpDifferentSideRed", "Arrived at Angle?: " + arrivedAtAngle);
            }

            if(arrivedAtAngle == false)
            {
                Log.d("BackUpDifferentSideRed", "Arrived at Angle?: " + arrivedAtAngle);

                if(angles.firstAngle >=0)
                {
                    if (angles.firstAngle < angle) {
                        robotControl.turnLeft(TURN_SPEED);
                        Log.d("BackUpDifferentSideRed", "Turned Left ");
                    } else if (angles.firstAngle > angle) {
                        robotControl.turnRight(TURN_SPEED);
                        Log.d("BackUpDifferentSideRed", "Turned Right ");
                    } else {
                        robotControl.stopMoving();
                    }
                }
                else if(angles.firstAngle < 0)
                {
                    if (angles.firstAngle > angle) {
                        robotControl.turnLeft(TURN_SPEED);
                        Log.d("BackUpDifferentSideRed", "Turned Left ");
                    } else if (angles.firstAngle < angle) {
                        robotControl.turnRight(TURN_SPEED);
                        Log.d("BackUpDifferentSideRed", "Turned Right ");
                    } else {
                        robotControl.stopMoving();
                    }
                }

                sleep(250);
                robotControl.stopMoving();
            }

            telemetry.update();
        }

        sleep(500);
        runtime.reset();

        while(runtime.seconds() < sideWaysTime)
        {
            robotControl.sideWaysLeft(ALIGN_SPEED);
        }


        //move forward to place glyph in cryptobox
        runtime.reset();
        robotControl.changeToEncoderMode();
        encoderDrive(DRIVE_SPEED, 12, 12, 5);
        sleep(250);

        //release glyph
        robotControl.openHand1();

        runtime.reset();
        robotControl.changeToEncoderMode();
        encoderDrive(DRIVE_SPEED, -8, -8, 5);
        sleep(500);

        //move forward again to push the glyph in
        robotControl.closeHand1();
        sleep(250);
        runtime.reset();
        encoderDrive(DRIVE_SPEED, 12, 12, 5);

        sleep(250);

        //move backwards
        runtime.reset();
        robotControl.changeToEncoderMode();
        encoderDrive(DRIVE_SPEED, -4, -4, 5);
        sleep(250);

    }


    /**
     * ENCODER METHOD!
     * @param speed
     * @param leftInches
     * @param rightInches
     * @param timeoutS
     */
    public void encoderDrive(double speed,
                             double leftInches, double rightInches,
                             double timeoutS) {
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
            // Note: We use (isBusy() && isBusy()) in the loop test, which means that when EITHER motor hits
            // its target position, the motion will stop.  This is "safer" in the event that the robot will
            // always end the motion as soon as possible.
            // However, if you require that BOTH motors have finished their moves before the robot continues
            // onto the next step, use (isBusy() || isBusy()) in the loop test.
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

            //  sleep(250);   // optional pause after each move
        }
    }


    /**
     * GYRO METHOD!!
     */
    public void composeTelemetry() {

        // At the beginning of each telemetry update, grab a bunch of data
        // from the IMU that we will then display in separate lines.
        telemetry.addAction(new Runnable() { @Override public void run()
        {
            // Acquiring the angles is relatively expensive; we don't want
            // to do that in each of the three items that need that info, as that's
            // three times the necessary expense.
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

    // Formatting
    String formatAngle(AngleUnit angleUnit, double angle) {
        return formatDegrees(AngleUnit.DEGREES.fromUnit(angleUnit, angle));
    }

    String formatDegrees(double degrees){
        return String.format(Locale.getDefault(), "%.1f", AngleUnit.DEGREES.normalize(degrees));
    }

}
