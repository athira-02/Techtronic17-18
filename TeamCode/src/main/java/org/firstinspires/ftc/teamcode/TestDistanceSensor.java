package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.hardware.bosch.JustLoggingAccelerationIntegrator;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

/**
 * Created by Athira on 12/27/2017.
 */

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.Func;
import org.firstinspires.ftc.robotcore.external.navigation.Acceleration;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.robotcore.external.navigation.Position;
import org.firstinspires.ftc.robotcore.external.navigation.Velocity;

import java.util.Locale;


/**
 * Created by Athira on 12/30/2017.
 */

@Disabled
@Autonomous (name = "TestDistanceSensor", group = "Sensor")
public class TestDistanceSensor extends LinearOpMode
{

    /* Define the robot hardware */
    RobotHardware myRobot   = new RobotHardware();   // Use my RobotHardware
    private ElapsedTime     runtime = new ElapsedTime();

    public void runOpMode()
    {
         /*
         * Initialize the drive system variables.
         * The init() method of the hardware class does all the work here

        myRobot.init(hardwareMap);

        waitForStart();

        while (opModeIsActive())
        {

            RobotControl robotControl = new RobotControl(hardwareMap, myRobot);
            SensorControl sensorControl = new SensorControl(hardwareMap, myRobot);

            String distance = sensorControl.getDistance(DistanceUnit.CM);

       //     telemetry.addData("Distance sensed (cm)",
            //        String.format(Locale.US, "%.02f", distance));


             telemetry.update();

             double distanceDouble = Double.parseDouble(distance);

             if(distanceDouble <= 50)
             {
                 telemetry.addData("I AM SENSING STUFF!!!: " , distanceDouble);
                 telemetry.update();
             }
             else
             {
                 telemetry.addData("HELP!!! HELP!! I CAN'T SEE!!!!", distanceDouble);
                 telemetry.update();
             }
        }
        */
    }



}
