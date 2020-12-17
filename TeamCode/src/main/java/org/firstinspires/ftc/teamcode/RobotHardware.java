package org.firstinspires.ftc.teamcode;

/**
 * Created by athira on 10/22/2017.
 */

import android.hardware.Sensor;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.hardware.bosch.JustLoggingAccelerationIntegrator;
import com.qualcomm.hardware.modernrobotics.ModernRoboticsI2cRangeSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.UltrasonicSensor;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.hardware.ColorSensor;


import static android.os.SystemClock.sleep;


public class RobotHardware
{
    //Defining the motor

    public DcMotor frontLeftWheel = null;
    public DcMotor frontRightWheel = null;
    public DcMotor backLeftWheel = null;
    public DcMotor backRightWheel = null;
    public DcMotor lift = null;
    public Servo rightHand1 = null;
    public Servo leftHand1 = null;
    public Servo rightHand2 = null;
    public Servo leftHand2 = null;
    public Servo stickShoulder = null;
    public Servo stickElbow = null;
    public Servo smacker = null;
    public Servo twister = null;
    // The IMU sensor object
    public BNO055IMU imu = null;

    //Color/Distance Sensors
    public DistanceSensor distanceSensor = null;
    public ColorSensor colorSensor = null;

    //Range Sensors
    ModernRoboticsI2cRangeSensor leftRangeSensor;
    ModernRoboticsI2cRangeSensor rightRangeSensor;

    //Relic Extension
    public DcMotor relicExtension = null;
    public DcMotor relicWireSpool = null;
    public Servo relicWrist = null;
    public Servo relicClaw = null;

    //Adding the Hardware Map
    public HardwareMap hwMap  = null;

    public void init(HardwareMap ahwMap)
    {
        hwMap = ahwMap;
        //Initialize the wheel motors
        frontLeftWheel = hwMap.get(DcMotor.class, "frontLeftWheel");
        frontRightWheel = hwMap.get(DcMotor.class, "frontRightWheel");
        backLeftWheel = hwMap.get(DcMotor.class, "backLeftWheel");
        backRightWheel = hwMap.get(DcMotor.class, "backRightWheel");
        frontRightWheel.setDirection(DcMotor.Direction.REVERSE);
        backRightWheel.setDirection(DcMotor.Direction.REVERSE);

        // Set up the parameters with which we will use our IMU. Note that integration
        // algorithm here just reports accelerations to the logcat log; it doesn't actually
        // provide positional information.
        BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();
        parameters.angleUnit           = BNO055IMU.AngleUnit.DEGREES;
        parameters.accelUnit           = BNO055IMU.AccelUnit.METERS_PERSEC_PERSEC;
        parameters.calibrationDataFile = "FTC13747.json"; // see the calibration sample opmode
        parameters.loggingEnabled      = true;
        parameters.loggingTag          = "IMU";
        parameters.accelerationIntegrationAlgorithm = new JustLoggingAccelerationIntegrator();

        // Retrieve and initialize the IMU. We expect the IMU to be attached to an I2C port
        // on a Core Device Interface Module, configured to be a sensor of type "AdaFruit IMU",
        // and named "imu".
        imu = hwMap.get(BNO055IMU.class, "imu");
        imu.initialize(parameters);


        //Initialize the arm motor
        lift = hwMap.get(DcMotor.class, "lift");
        lift.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        lift.setDirection(DcMotor.Direction.REVERSE);

        //Initialize the hand motor
        rightHand1 = hwMap.get(Servo.class, "rightHand1");
        leftHand1 = hwMap. get(Servo.class, "leftHand1");
        rightHand2 = hwMap.get(Servo.class, "rightHand2");
        leftHand2 = hwMap. get(Servo.class, "leftHand2");


        stickShoulder = hwMap.get(Servo.class, "stickShoulder");
        stickElbow = hwMap.get(Servo.class , "stickElbow");
        smacker = hwMap.get(Servo.class , "smacker");

        // get a reference to the color sensor.
        colorSensor = hwMap.get(ColorSensor.class, "color_distance_sensor");

        // get a reference to the distance sensor that shares the same name.
        distanceSensor = hwMap.get(DistanceSensor.class, "color_distance_sensor");

        //range sensor
        leftRangeSensor = hwMap.get(ModernRoboticsI2cRangeSensor.class, "left_sensor_range");
        rightRangeSensor = hwMap.get(ModernRoboticsI2cRangeSensor.class, "right_sensor_range");

        //relic extension
        relicExtension = hwMap.get(DcMotor.class, "relic_extension");
        relicWrist = hwMap.get(Servo.class , "relic_wrist");
        relicClaw = hwMap.get(Servo.class , "relic_claw");

        //twister
        twister = hwMap.get(Servo.class , "twister");



    }
}
