package org.firstinspires.ftc.teamcode;

/**
 * This class has the methods for sensor use
 * Created by Athira on 11/9/2017.
 */

import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.ClassFactory;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.RelicRecoveryVuMark;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackable;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackables;

import java.util.Locale;


public class SensorControl {

    HardwareMap hardwareMap = null;
    RobotHardware robot = null;

    //Constructor
    public SensorControl(HardwareMap map, RobotHardware in_robot) {
        hardwareMap = map;
        this.robot = in_robot;

    }

    /**
     * ---------------------------------------------------------------------------------------------
     * VUFORIA
     * finds sets up the vuforia tracking system and reads the vuMark across from the phone
     *
     * @return a String (either "LEFT", "CENTER", "RIGHT" which is the decoded meaning of the vuMark
     * ---------------------------------------------------------------------------------------------
     */
    public String readVuMark() {
        try {
            ElapsedTime vumarktime = new ElapsedTime();
            VuforiaLocalizer vuforia;
            int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
            VuforiaLocalizer.Parameters parameters = new VuforiaLocalizer.Parameters(cameraMonitorViewId);


            //Vuforia License Key
            parameters.vuforiaLicenseKey = "AeOsidT/////AAAAGQnkRXs3VUhkt2oAT5Vqijlaruq7jh7amPZRmwf5ickp1shfQOlRoEW+Av5UcqVZ9xnqtptnoJGAXJ6Z6h+ci2Km6mSseUGCj6uO46ajtOrHfgXda5R8XRRzEjSyKkisttbXklJOt9tTEU+eeVXunX6wEGAppftfjybdFDZ5yTeoz7WVGF0yO/dvWGzmjrKn9YRZ/xoL0xsNy4TBLK03wJiasu1IxNOxkPgmcMW9XkFulDsH8/F44/v0eWotoYl3iHFmlhNwgCrnQaWwFaG6tbtkh9Wiy8w3jxeClKEksGzFre4aEL3v0JEaazmzfjISvW48QQA7APk3U6sGN2PXn4NaXw+ttHc+nJXmh59B7CB8";

            //Vuforia Parameters
            parameters.cameraDirection = VuforiaLocalizer.CameraDirection.FRONT;
            vuforia = ClassFactory.createVuforiaLocalizer(parameters);


            VuforiaTrackables relicTrackables = vuforia.loadTrackablesFromAsset("RelicVuMark");
            VuforiaTrackable relicTemplate = relicTrackables.get(0);
            relicTemplate.setName("relicVuMarkTemplate");

            //Activate the relic tracker
            relicTrackables.activate();
            vumarktime.reset();
            //   boolean sensed = false;
            while (vumarktime.seconds() < 3) {
                RelicRecoveryVuMark vuMark = RelicRecoveryVuMark.from(relicTemplate);
                if (vuMark != RelicRecoveryVuMark.UNKNOWN) {
                    //          sensed = true;
                    return vuMark.toString();
                }

            }
        } catch (Exception e) {
            return "CENTER";
        }
        return "CENTER";

    }

    /**
     * ---------------------------------------------------------------------------------------------
     * COLOR SENSOR
     * Placed at the end of the jewel smacker.  It is used to identify the colo rof the jewel to the
     * right of the smacker
     * ---------------------------------------------------------------------------------------------
     */

    /**
     * This method compares the sensed RED values and the sensed BLUE values
     * If red is greater, it returns true and if blue is greater it returns false
     *
     * @return whether or not red is sensed.
     */
    public boolean isRed() {

        if (robot.colorSensor.red() > robot.colorSensor.blue())
            return true;

        else
            return false;
    }

    /**
     * This method compares the sensed RED values and the sensed BLUE values
     * If blue is greater, it returns true and if red is greater it returns false
     *
     * @return whether or not blue is sensed.
     */
    public boolean isBlue() {

        if (robot.colorSensor.blue() > robot.colorSensor.red())
            return true;

        else
            return false;
    }


    /**
     * ---------------------------------------------------------------------------------------------
     * SMACKER DISTANCE SENSOR
     * Gets the distance sensed by the color distance sensor that is at the end of the smacker.
     * This is used to ensure that the color sensing is in fact sensing the jewel and not something
     * else
     * ---------------------------------------------------------------------------------------------
     */


    public double getSmackerDistance()
    {
        double distance = robot.distanceSensor.getDistance(DistanceUnit.CM);

        String distanceString = String.format(Locale.US, "%.02f", distance);

        double distanceDouble = Double.parseDouble(distanceString);

        return distanceDouble;
    }



    /**
     * ---------------------------------------------------------------------------------------------
     * RANGE SENSORS
     * This gets the distance for either the right or the left MR range sensors on the robot
     * This is used during the autonomous mode for aligning the robot with the correct column of the
     * cryptobox
     * ---------------------------------------------------------------------------------------------
     */

    public double getDistance(String myColor)
    {
        if (myColor.equalsIgnoreCase("BLUE"))
        {
            return robot.leftRangeSensor.getDistance(DistanceUnit.CM);
        }
        else
        {
            return robot.rightRangeSensor.getDistance(DistanceUnit.CM);
        }
    }
}

