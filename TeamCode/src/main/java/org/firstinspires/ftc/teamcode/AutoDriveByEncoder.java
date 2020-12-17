/**
 *This is the path the robot follows during autonomous mode for the  corner
 * with Blue balancing stone and racks on same side.
 * This uses the robot controls in the class RobotControl
 * @author Nikhil and Athira and Omkar
 * */


package org.firstinspires.ftc.teamcode;

//imports
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;



@Disabled
@Autonomous
public class AutoDriveByEncoder extends LinearOpMode {

    /* Define the robot hardware */
    RobotHardware myRobot   = new RobotHardware();   // Use my RobotHardware
    private ElapsedTime     runtime = new ElapsedTime();

    //Adding the robotControls
    RobotControl robotControl;


    //Constants
    final double     COUNTS_PER_MOTOR_REV    = 1000 ;    // eg: TETRIX Motor Encoder
    final double     DRIVE_GEAR_REDUCTION    = 1.0 ;     // This is < 1.0 if geared UP
    final double     WHEEL_DIAMETER_INCHES   = 4.0 ;     // For figuring circumference
    final double     COUNTS_PER_INCH         = (COUNTS_PER_MOTOR_REV * DRIVE_GEAR_REDUCTION) /
            (WHEEL_DIAMETER_INCHES * 3.1415);
    final double     DRIVE_SPEED             = 0.6;
    final double     TURN_SPEED              = 0.5;


    //Variables for time to run in seconds
    double liftArm = 7;
    double lowerArm = 0.75;
    double moveBackwards;
    double turnToRacks;
    double moveToRacks;
    double turningToPark;
    double forwardToPark;
    double backwardAlign;
    double backout;

    public void runOpMode() {

        myRobot.init(hardwareMap);

        //Robot Control
        robotControl = new RobotControl(hardwareMap, myRobot );

        //Ready to run
        telemetry.addData("Status", "Ready to run");
        telemetry.update();
        //Wait for start
        waitForStart();


        ElapsedTime     runtime = new ElapsedTime();


        robotControl.changeToEncoderMode();

        if(robotControl !=null)
        {
            telemetry.addData("Status", "Robot control is not null");
        }
        else
        {
            telemetry.addData("Status", "Robot control is null");
        }
        telemetry.update();

        // Send telemetry message to indicate successful Encoder reset
        telemetry.addData("Path0",  "Starting at %7d :%7d : %7d : %7d",
                robotControl.getBackLeftWheelCurrentPosition(),
                robotControl.getBackRightWheelCurrentPosition(),
                robotControl.getFrontLeftWheelCurrentPosition(),
                robotControl.getFrontRightWheelCurrentPosition());
        telemetry.update();

        waitForStart();

        encoderDrive(DRIVE_SPEED,  12,  12, 5.0);  // S1: Forward 12 Inches with 5 Sec timeout
        sleep(500);
        encoderDrive(DRIVE_SPEED, -24, -24, 4.0);  // S3: Reverse 24 Inches with 4 Sec timeout
        sleep(500);
        encoderDrive(DRIVE_SPEED, 36, 36, 5.0);

    }

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




}