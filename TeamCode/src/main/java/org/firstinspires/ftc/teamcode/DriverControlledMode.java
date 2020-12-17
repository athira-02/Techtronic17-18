package org.firstinspires.ftc.teamcode;

import android.util.Log;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;


import static android.R.attr.right;

/**
 * This program is our teleOp Mode
 * This uses the robot controls in the class called RobotControl
 * Created by Athira on 10/22/2017.
 */

@TeleOp (name="DriverControlledMode", group="Qualifier")
public class DriverControlledMode extends LinearOpMode
{

    static final double UP_ARM_SPEED = 0.8;
    static final double DOWN_ARM_SPEED = 0.5;
    static final double RELIC_SPEED = 0.65;
    double relicWristPos = 1;
    double relicHandPos = 1;

    double speed = 1;

    boolean flipUp = false;
    boolean hand1IsOpen = true;
    boolean hand2IsOpen = true;


    /* Define the robot hardware */
    RobotHardware robot   = new RobotHardware();   // Use my RobotHardware

    public void runOpMode()
    {
        robot.init(hardwareMap);

        //Adding the robotControls
        RobotControl robotControl = new RobotControl(hardwareMap, robot);

        // Send telemetry message to signify robot waiting;
        telemetry.addData("Say", "Hello Driver");    //
        telemetry.update();

        Log.d("DriverControlledMode", "Hello Driver");

        // Wait for the game to start (driver presses PLAY)
        waitForStart();

        while (opModeIsActive())
        {
            robotControl.stickUp();

            /**
             * Variables
             */
            //Wheel variables
            double yControlA;
            double xControlA;
            double sideWaysRightA;
            double sideWaysLeftA;
            double yControlB;
            double xControlB;
            double sideWaysRightB;
            double sideWaysLeftB;

            //Arm Variables
            boolean liftUp;
            boolean liftDown;
            boolean liftStop;
            //Hand variables
            boolean doubleHand;
            boolean doubleMid;
            boolean bottomHand;
            boolean midHand;


            //Relic Extension
            boolean relicWristUp;
            boolean relicWristDown;
            boolean relicWristStop;
            boolean relicRetract;
            boolean relicExtend;
            boolean relicStop;
            boolean relicHandOpen;
            boolean relicHandClose;
            boolean relicQuickWristUp;
            boolean relicHandStop;
            //Smacker Safety
            boolean smackerReturn;

            boolean flip;

            /**
             * Setting the gamepad buttons
             */
            //wheel
            yControlA = gamepad1.left_stick_y;
            xControlA = gamepad1.left_stick_x;
            sideWaysRightA = gamepad1.right_trigger;
            sideWaysLeftA = gamepad1.left_trigger;

            yControlB = gamepad2.left_stick_y;
            xControlB = gamepad2.left_stick_x;
            sideWaysRightB = gamepad2.right_trigger;
            sideWaysLeftB = gamepad2.left_trigger;

            //arm
            liftUp = gamepad1.dpad_up;
            liftDown = gamepad1.dpad_down;
            liftStop = gamepad1.atRest();
            //hand
            midHand = gamepad1.y;
            doubleHand = gamepad1.x;
            bottomHand = gamepad1.b;
            doubleMid = gamepad1.right_bumper;

            //relic
            relicRetract = gamepad2.dpad_left;
            relicExtend = gamepad2.dpad_right;
            relicStop = gamepad2.atRest();
            relicHandOpen = gamepad2.b;
            relicHandClose = gamepad2.x;
            relicHandStop = gamepad2.atRest();
            relicWristUp = gamepad2.dpad_up;
            relicQuickWristUp = gamepad2.a;
            relicWristDown = gamepad2.dpad_down;
            relicWristStop = gamepad2.atRest();

            //smacker safety
            smackerReturn = gamepad2.a;

            flip = gamepad1.a;

            /**
             * Wheel Controls
             */
            //Moving forward/backwards
            if(yControlA != 0 || yControlB != 0)
            {
                if(yControlB != 0)
                {
                    robotControl.moveForward(speed * -yControlB);
                }
                else
                {
                    robotControl.moveForward(speed * -yControlA);
                }
            }
            //Turn Right
            else if (xControlA > 0 || xControlB > 0)
            {
                if(xControlB > 0)
                {
                    robotControl.turnRight(speed * xControlB);
                }
                else
                {
                    robotControl.turnRight(speed * xControlA);
                }
            }
            //Turn Left
            else if (xControlA < 0 || xControlB < 0)
            {
                if(xControlB < 0)
                {
                    robotControl.turnLeft(speed * -xControlB);
                }
                else
                {
                    robotControl.turnLeft(speed * -xControlA);
                }
            }
            else if (sideWaysRightA > 0 || sideWaysRightB > 0)
            {
                if(sideWaysRightB > 0)
                {
                    robotControl.sideWaysRight(speed* sideWaysRightB);
                }
                else
                {
                    robotControl.sideWaysRight(speed * sideWaysRightA);
                }
            }
            else if (sideWaysLeftA > 0 || sideWaysLeftB > 0)
            {
                if(sideWaysLeftB > 0)
                {
                    robotControl.sideWaysLeft(speed* sideWaysLeftB);
                }
                else
                {
                    robotControl.sideWaysLeft(speed * sideWaysLeftA);
                }
            }
            if (yControlA == 0 && xControlA == 0 && yControlB == 0 && xControlB == 0)
            {
                robotControl.stopMoving();
            }

            /**
             * Arm Control
             */
            if (liftUp == true)
            {
                robotControl.liftUp(UP_ARM_SPEED);
            }
            else if (liftDown == true)
            {
                robotControl.liftDown(DOWN_ARM_SPEED);
            }
            else if (liftStop == true)
            {
                robotControl.liftStop();
            }
            /*
            * Hand Control
            */

            if (bottomHand == true)
            {
                robotControl.openHand2();
            }
            if (doubleHand == true)
            {
                robotControl.closeHand2();
            }

            if(midHand == true)
            {
                robotControl.midHand2();
            }

            /*
            if(midHand == true)
            {
                if(flipUp == true)
                {
                    robotControl.midHand1();
                    hand1IsOpen = true;
                }
                else
                {
                    robotControl.midHand2();
                    hand2IsOpen = true;
                }
            }

*/
            /*
            * Relic Extension Control
            */

            if(relicExtend == true)
            {
                robotControl.extendRelicExtension(0.8);
            }
            else if(relicRetract == true)
            {
                robotControl.retractRelicExtension(RELIC_SPEED);
            }
            else if(relicStop == true)
            {
                robotControl.stopRelicExtension();
            }
            else
            {
                robotControl.stopRelicExtension();
            }

            //Relic Wrist
            if (relicWristUp == true)
            {
                if(relicWristPos > 0)
                {
                    relicWristPos = relicWristPos - 0.001;
                    //Log.d("DriverControlledMode", "in wrist up- " + "relic wrist pos: " + relicWristPos);
                }
                robotControl.moveWrist(relicWristPos);
            }

            if (relicWristDown == true)
            {
                if(relicWristPos < 1)
                {
                    relicWristPos = relicWristPos + 0.001;
                    //Log.d("DriverControlledMode", "in wrist down- " + "relic wrist pos: " + relicWristPos);
                }
                robotControl.moveWrist(relicWristPos);
            }

            if(relicWristStop == true)
            {
                robotControl.moveWrist(relicWristPos);
            }

            if(relicQuickWristUp == true)
            {
                robotControl.moveWrist(0);

                relicWristPos = 0;
                sleep(250);
            }



            //Relic Claw
            if(relicHandOpen == true)
            {
                if(relicHandPos > 0)
                {
                    relicHandPos = relicHandPos - 0.005;
                    //Log.d("DriverControlledMode", "in hand open- " + "relic hand pos: " + relicHandPos);
                }

                robotControl.moveRelicHand(relicHandPos);            }

            if (relicHandClose == true)
            {
                if(relicHandPos < 1)
                {
                    relicHandPos = relicHandPos + 0.005;
                    //Log.d("DriverControlledMode", "in hand close- " + "relic hand pos: " + relicHandPos);
                }
                robotControl.moveRelicHand(relicHandPos);
            }

            if(relicHandStop == true)
            {
                robotControl.moveRelicHand(relicHandPos);
            }



            //Smacker Safety
            if(smackerReturn == true)
            {
                robotControl.stickUp();
            }

            /*
            if(flip == true)
            {
                robotControl.stopMoving();

                //Close both hands
                robotControl.closeHand1();
                robotControl.closeHand2();

                sleep(500);

                hand1IsOpen = false;
                hand2IsOpen = false;

                ElapsedTime runtime = new ElapsedTime();

                runtime.reset();


                while(opModeIsActive() && (runtime.seconds() < 0.5))
                {
                    //Lifting the arm up
                    robotControl.liftUp(UP_ARM_SPEED);

                }

                robotControl.liftStop();

                sleep(250);

                if(flipUp == true)
                {
                    robotControl.twistDown();
                    flipUp = false;
                }
                else
                {
                    robotControl.twistUp();
                    flipUp = true;
                }

                sleep(1000);
                runtime.reset();

                while(opModeIsActive() && runtime.seconds() < 0.5)
                {
                    robotControl.liftDown(DOWN_ARM_SPEED);
                }

                robotControl.liftStop();

                if(flipUp == true)
                {
                    robotControl.openHand1();
                    hand1IsOpen = true;
                }
                else
                {
                    robotControl.openHand2();
                    hand2IsOpen = true;
                }

                sleep(250);
            }

            if(doubleMid == true)
            {
                robotControl.midHand2();
                robotControl.midHand1();
            }
            */
        }


    }
}
