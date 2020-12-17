package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.Acceleration;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;

/**
 * Created by Athira on 12/31/2017.
 */

@Disabled
@Autonomous (name = "TestSmacker", group = "Sensor")
public class TestSmacker extends LinearOpMode
{
    //Hardware
    RobotHardware myRobot = new RobotHardware();;

    RobotControl robotControl;
    SensorControl sensorControl;
    private ElapsedTime runtime = new ElapsedTime();


    public void runOpMode()
    {
        myRobot.init(hardwareMap);

        robotControl = new RobotControl(hardwareMap, myRobot);
        sensorControl = new SensorControl(hardwareMap, myRobot);

        waitForStart();


        robotControl.stickDown();

        telemetry.addData("Moved Stick Down: " ,"");
        telemetry.update();
        sleep(1000);


        double distance = sensorControl.getSmackerDistance();
        telemetry.addData("Distance is: " , distance);
        telemetry.update();
        sleep(1000);

        if(sensorControl.isRed() && distance < 20)
        {
            telemetry.addData("I see: " , "Red");
            telemetry.update();
            sleep(500);
            robotControl.smackRight();
            telemetry.addData("Smacked: " , "Right");
            sleep(1000);
        }
        else if(sensorControl.isBlue() && distance < 20)
        {
            telemetry.addData("I see: " , "Blue");
            telemetry.update();
            sleep(500);
            robotControl.smackLeft();
            telemetry.addData("Smacked: " , "Left");
            sleep(1000);
        }
        else
        {
            telemetry.addData("I can't sense the color", "");
            telemetry.update();
            sleep(1000);
        }

        telemetry.update();

        //Bring the smacker back up
        robotControl.stickUp();

        telemetry.addData("I have returned to the starting position: " , "");
        telemetry.update();
        sleep(1000);




        /*

        if(distance < 50)
        {
            if(sensorControl.isBlue())
            {
                telemetry.addData("Sensed: " , "Blue");
                telemetry.update();
                sleep(250);
                robotControl.smackLeft();
                telemetry.addData("Smacked: " , "Left");
                telemetry.update();
                sleep(250);

                robotControl.moveSmacker(0.5);
                sleep(250);
                robotControl.moveStickShoulder(0);
                sleep(250);
                robotControl.moveStickElbow(1);

                telemetry.addData("Returned: " , "to position");
                telemetry.update();
                sleep(250);
            }
            else if (sensorControl.isRed())
            {
                telemetry.addData("Sensed: " , "Red");
                telemetry.update();
                sleep(250);
                robotControl.smackRight();
                telemetry.addData("Smacked: " , "Right");
                telemetry.update();
                sleep(250);

                robotControl.moveSmacker(0.5);
                sleep(250);
                robotControl.moveStickShoulder(0);
                sleep(250);
                robotControl.moveStickElbow(1);

                telemetry.addData("Returned: " , "to position");
                telemetry.update();
                sleep(250);
            }
            else
            {
                telemetry.addData("No Color Sensed!!!!", "");
                telemetry.update();

                robotControl.moveSmacker(0.5);
                sleep(250);
                robotControl.moveStickShoulder(0);
                sleep(250);
                robotControl.moveStickElbow(1);
            }
        }
        else
        {
            telemetry.addData("No Distance Sensed!!!!", "");
            telemetry.update();

            robotControl.moveSmacker(0.5);
            sleep(250);
            robotControl.moveStickShoulder(0);
            sleep(250);
            robotControl.moveStickElbow(1);
        }

        robotControl.moveSmacker(0.5);
        sleep(250);
        robotControl.moveStickShoulder(0);
        sleep(250);
        robotControl.moveStickElbow(1);
    */
    }


}
