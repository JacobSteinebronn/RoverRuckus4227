package CVTesting;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.hardware.bosch.JustLoggingAccelerationIntegrator;
import com.qualcomm.hardware.rev.RevSPARKMini;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.Acceleration;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.robotcore.external.navigation.Position;
import org.firstinspires.ftc.robotcore.external.navigation.Velocity;

/**
 * Created by HHS-ROBOTICS on 12/14/2018.
 */

public class Robot {

    public Servo lights;

    public DcMotor motorR;
    public DcMotor motorL;
    public DcMotor pivot;
    public DcMotor reel;

    public DcMotor hang;


    public  Servo sweep;
    public  Servo marker;
    public  Servo latch;


    public DistanceSensor sideRange1;
    public DistanceSensor sideRange2;
    public DistanceSensor frontSensor;
    public DistanceSensor frontSensor2;

    public DistanceSensor backSensor;




    public BNO055IMU imu;

    public Orientation angles;
    public Acceleration gravity;

    Telemetry telemetry;



    public Robot(HardwareMap hardwareMap, Telemetry telemetry){
        this.telemetry=telemetry;

        lights = hardwareMap.get(Servo.class, "lights");

        motorR = hardwareMap.get(DcMotor.class, "rightMotor");
        motorL = hardwareMap.get(DcMotor.class, "leftMotor");
        pivot = hardwareMap.get(DcMotor.class, "pivot");
        reel = hardwareMap.get(DcMotor.class, "reel");

        hang = hardwareMap.get(DcMotor.class, "hang");

        sweep = hardwareMap.get(Servo.class, "sweep");
        marker=hardwareMap.get(Servo.class, "marker");
        latch=hardwareMap.get(Servo.class, "latch");


        sideRange1=hardwareMap.get(DistanceSensor.class, "sideRange1");
        sideRange2=hardwareMap.get(DistanceSensor.class, "sideRange2");
        frontSensor=hardwareMap.get(DistanceSensor.class, "front");
        frontSensor2=hardwareMap.get(DistanceSensor.class, "front2");

        backSensor=hardwareMap.get(DistanceSensor.class, "backSensor");


        motorL.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motorR.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        pivot.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        reel.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        motorL.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        motorR.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        pivot.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        reel.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        imu = hardwareMap.get(BNO055IMU.class, "imu");
        BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();
        parameters.angleUnit           = BNO055IMU.AngleUnit.DEGREES;
        parameters.accelUnit           = BNO055IMU.AccelUnit.METERS_PERSEC_PERSEC;
        parameters.calibrationDataFile = "BNO055IMUCalibration.json"; // see the calibration sample opmode
        parameters.loggingEnabled      = true;
        parameters.loggingTag          = "IMU";
        parameters.accelerationIntegrationAlgorithm = new JustLoggingAccelerationIntegrator();


        imu.initialize(parameters);
        imu.startAccelerationIntegration(new Position(), new Velocity(), 1000);


        //composeTelemetry();
    }

}
