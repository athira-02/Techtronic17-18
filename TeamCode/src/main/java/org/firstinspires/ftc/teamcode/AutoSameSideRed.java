/**
 *This is the path the robot follows during autonomous mode for the  corner
 * with Red balancing stone and racks on same side.
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
public class AutoSameSideRed extends LinearOpMode {
    //Constants to control speed and position
    static final double DRIVE_SPEED = 0.3;
    static final double TURN_SPEED = 0.3;
    static final double ARM_SPEED = 0.15;
    static final double CLOSE_LHAND_POSITION = 0.5;
    static final double CLOSE_RHAND_POSITION = 0.5;
    static final double OPEN_LHAND_POSITION = 0.0;
    static final double OPEN_RHAND_POSITION = 1.0;
    static final double STICK_UP_POSITION = 0.05;
    static final double STICK_DOWN_POSITION =0.9;
    static final double SMACKER_LEFT_POSITION =0.1;
    static final double SMACKER_RIGHT_POSITION =0.9;
    static final double SMACKER_RETURN_POSITION = 0.5;
    static final String MY_COLOR = "RED";

    private ElapsedTime runtime = new ElapsedTime();

    //Variables for time to run in seconds
    double liftArm = 2;
    double lowerArm = 0.75;
    double moveForward;
    double turnToRacks;
    double moveToRacks;
    double turningToPark;
    double forwardToPark;
    double backwardAlign;
    double backout;

    public void runOpMode() {

        //Ready to run
        telemetry.addData("Status", "Ready to run");
        telemetry.update();
/*
        //Wait for start
        waitForStart();
        //Adding the robotControls
        RobotControl robotControl = new RobotControl(hardwareMap);
        //Adding the Sensor Controls
        SensorControl sensorControl = new SensorControl(hardwareMap, RobotControl.robot);
        //Grip the glyph
        robotControl.closeHand(CLOSE_RHAND_POSITION, CLOSE_LHAND_POSITION);

        /*
        NIKHIL ARAYATH 11/12/17
        Added the color sensing and jewel smacking.
        Depending upon the decided color, the smacker servo moves left or right and knocks out one of the jewels.
         */
        /*
        robotControl.stickDown(STICK_DOWN_POSITION);
        sleep(1000);
        telemetry.addData("Red Value is ", sensorControl.getRedValue());
        telemetry.addData("Blue Value is ", sensorControl.getBlueValue());
        telemetry.addData("Combined Value is ", sensorControl.getArgValue());
        telemetry.update();

        if (sensorControl.isMyColor(MY_COLOR)){
            telemetry.addData("In isMyColor if block ", MY_COLOR);
            telemetry.update();
            sleep(500);
            robotControl.smackerLeft(SMACKER_LEFT_POSITION);
            sleep(500);
            robotControl.stickUp(STICK_UP_POSITION);
            robotControl.smackerReturn(SMACKER_RETURN_POSITION);
        }
        else{
            telemetry.addData("In else of isMyColor  ", "Opposite Color");
            telemetry.update();
            sleep(500);
            robotControl.smackerRight(SMACKER_RIGHT_POSITION);
            sleep(500);
            robotControl.stickUp(STICK_UP_POSITION);
            robotControl.smackerReturn(SMACKER_RETURN_POSITION);
        }

        runtime.reset();

        //move the arm up
        while (opModeIsActive() && (runtime.seconds() < liftArm))
        {
            robotControl.armUp(ARM_SPEED);
        }

        robotControl.armStop();

        //Sense the VuMark
        String vuMark = sensorControl.readVuMark();
        turnToRacks = 1.5;
        moveToRacks = 0.5;
        backwardAlign = 0.75;
        forwardToPark = 0.75;
        backout = 0.75;
        if(vuMark.equalsIgnoreCase("RIGHT"))
        {
            telemetry.addData("In right block ", "picture is right");
            telemetry.update();
            moveForward = 2.5;
        }
        else if (vuMark.equalsIgnoreCase("CENTER"))
        {
            telemetry.addData("In center block ", "picture is center");
            telemetry.update();
            moveForward = 2.75;
        }
        else if (vuMark.equalsIgnoreCase("LEFT"))
        {
            telemetry.addData("In left block ", "picture is left");
            telemetry.update();
            moveForward = 3.5;
        }
        else
        {
            telemetry.addData("In UNK block ", "picture is UNK");
            telemetry.update();
            moveForward = 2.0;
        }
        //The steps
        runtime.reset();
        while (opModeIsActive() && runtime.seconds() < moveForward)
        {
            robotControl.moveForward(DRIVE_SPEED);  //Moving Forward off the Balancing Stone
        }
        robotControl.stopMoving();
        sleep(500);
        runtime.reset();

        while (opModeIsActive() && runtime.seconds() < turnToRacks)
        {
            robotControl.turnRight(TURN_SPEED);
        }
        robotControl.stopMoving();
        sleep(500);
        runtime.reset();

        /*
        while (opModeIsActive() && runtime.seconds() < backwardAlign)
        {
            robotControl.moveBackwards(DRIVE_SPEED);
        }

        robotControl.stopMoving();
        sleep(500);
        runtime.reset();

        while (opModeIsActive() && (runtime.seconds() < lowerArm))
        {
            robotControl.armDown(ARM_SPEED);
        }
        robotControl.armStop();
        runtime.reset();
        while (opModeIsActive() && runtime.seconds() < moveToRacks)
        {
            robotControl.moveForward(DRIVE_SPEED);  //Move Forward
        }
        robotControl.stopMoving();
        runtime.reset();

        */
/*
        robotControl.openHand(OPEN_RHAND_POSITION,OPEN_LHAND_POSITION); //Open the hand to let go of the glyph
        sleep(500);
        runtime.reset();

        while (opModeIsActive() && runtime.seconds() < backout)
        {
            robotControl.moveBackwards(DRIVE_SPEED);  //Move Backwards
        }
        */
    }


}

