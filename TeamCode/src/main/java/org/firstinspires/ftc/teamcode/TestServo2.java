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
@Autonomous (name = "TestServo2", group = "Sensor")
public class TestServo2 extends LinearOpMode
{
    //Hardware
    RobotHardware myRobot = new RobotHardware();;

    RobotControl robotControl;
    private ElapsedTime runtime = new ElapsedTime();


    public void runOpMode()
    {

        myRobot.init(hardwareMap);
        //Ready to run
        telemetry.addData("Status", "Ready to run");
        telemetry.update();

        //Wait for start
        waitForStart();

        //Adding the robotControls
        robotControl = new RobotControl(hardwareMap, myRobot);
        SensorControl sensorControl = new SensorControl(hardwareMap, myRobot);

        /*

        robotControl.robot.stickShoulder.setPosition(0);
        telemetry.addData("Stick Shoulder... Set to: " , "0");
        telemetry.update();
        sleep(500);

        robotControl.robot.stickShoulder.setPosition(0.5);
        telemetry.addData("Stick Shoulder... Set to: " , "0.5");
        telemetry.update();
        sleep(500);

        robotControl.robot.stickShoulder.setPosition(1);
        telemetry.addData("Stick Shoulder... Set to: " , "1");
        telemetry.update();
        sleep(500);

        robotControl.robot.stickShoulder.setPosition(0.5);
        telemetry.addData("Stick Shoulder... Set to: " , "0.5");
        telemetry.update();
        sleep(500);






        robotControl.robot.stickElbow.setPosition(0);
        telemetry.addData("Stick Elbow... Set to: " , "0");
        telemetry.update();
        sleep(500);

        robotControl.robot.stickElbow.setPosition(0.5);
        telemetry.addData("Stick Elbow... Set to: " , "0.5");
        telemetry.update();
        sleep(500);

        robotControl.robot.stickElbow.setPosition(1);
        telemetry.addData("Stick Elbow... Set to: " , "1");
        telemetry.update();
        sleep(500);

        robotControl.robot.stickElbow.setPosition(0.5);
        telemetry.addData("Stick Elbow... Set to: " , "0.5");
        telemetry.update();
        sleep(500);




*/


        robotControl.robot.smacker.setPosition(0);
        telemetry.addData("Smacker... Set to: " , "0");
        telemetry.update();
        sleep(5000);

        robotControl.robot.smacker.setPosition(0.2);
        telemetry.addData("Smacker... Set to: " , "0.2");
        telemetry.update();
        sleep(5000);

        robotControl.robot.smacker.setPosition(0);
        telemetry.addData("Smacker... Set to: " , "0");
        telemetry.update();
        sleep(5000);

        robotControl.robot.smacker.setPosition(-0.2);
        telemetry.addData("Smacker... Set to: " , "-0.5");
        telemetry.update();
        sleep(5000);
    }

}
