package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.Hardware;

import static android.R.attr.right;

/**
 * This program is our teleOp Mode
 * This uses the robot controls in the class called RobotControl
 * Created by athira on 10/22/2017.
 */


@Disabled
@TeleOp(name = "TeleOpMode", group = "Test")
public class TeleOpMode extends LinearOpMode
{
    static final double ARM_SPEED = 0.15;
    static final double CLOSE_LHAND_POSITION = 0.5;
    static final double CLOSE_RHAND_POSITION = 0.5;
    static final double OPEN_LHAND_POSITION = 0.0;
    static final double OPEN_RHAND_POSITION = 1.0;

    //Hardware
    RobotHardware myRobot = new RobotHardware();

    public void runOpMode()
    {
        myRobot.init(hardwareMap);
        /*
        //Adding the robotControls
        RobotControl robotControl = new RobotControl(hardwareMap);


        // Send telemetry message to signify robot waiting;
        telemetry.addData("Say", "Hello Driver");    //
        telemetry.update();

        // Wait for the game to start (driver presses PLAY)
        waitForStart();

        /*
        while (opModeIsActive())
        {
            /**
             * Variables
             */
        /*
            //Wheel variables
            double speed = 0.5;
            double yControl;
            double xControl;
            //Arm variables
            boolean armUp;
            boolean armDown;
            boolean armStop;
            //Hand variables
            boolean handOpen;
            boolean handClose;

            /**
             * Setting the gamepad buttons
             */
            //arm
        /*
            armUp = gamepad1.dpad_up;
            armDown = gamepad1.dpad_down;
            armStop = gamepad1.a;
            //hand
            handOpen = gamepad1.b;
            handClose = gamepad1.x;
            //wheel
            yControl = gamepad1.left_stick_y;
            xControl = gamepad1.left_stick_x;

            /**
             * Wheel Controls
             */
            //Moving forward/backwards

        /*
            if(yControl != 0)
            {
                robotControl.moveForward(speed * -yControl);
            }
            //Turning Right
            else if (xControl > 0)
            {
                robotControl.turnRight(speed * xControl);
            }
            //Turning Left
            else if (xControl < 0)
            {
                robotControl.turnLeft(speed * -xControl);
            }
            if (yControl == 0 && xControl == 0)
            {
                robotControl.stopMoving();
            }

            /**
             * Arm Control
             */
        /*
            if (armUp == true)
            {
                robotControl.armUp(ARM_SPEED);
            }
            else if (armDown == true)
            {
                robotControl.armDown(ARM_SPEED);
            }
            else if (armStop == true)
            {
                robotControl.armStop();
            }

            /*
            * Hand Control
            */
        /*
            if (handOpen == true)
            {
                robotControl.openHand(OPEN_RHAND_POSITION, OPEN_LHAND_POSITION);
            }
            if (handClose == true)
            {
                robotControl.closeHand(CLOSE_RHAND_POSITION, CLOSE_LHAND_POSITION);
            }

            //Sleep
            sleep(40);
        }
        */
    }
}
