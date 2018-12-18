package CVTesting;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.hardware.bosch.JustLoggingAccelerationIntegrator;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.Func;
import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.Acceleration;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.robotcore.external.navigation.Position;
import org.firstinspires.ftc.robotcore.external.navigation.Velocity;

import Helpers.Scale2;

/**
 * Created by HHS-ROBOTICS on 12/14/2018.
 */

public class Robot {


    public DcMotor motorL;
    public DcMotor motorR;
    public DcMotor pivot;
    public DcMotor assist;

    public  Servo lifter1;
    public  Servo lifter2;
    public  Servo sweep;
    public  Servo marker;


    public DistanceSensor sideRange1;
    public DistanceSensor sideRange2;
    public DistanceSensor frontSensor;
    public DistanceSensor frontSensor2;




    public BNO055IMU imu;

    public Orientation angles;
    public Acceleration gravity;

    Telemetry telemetry;



    public Robot(HardwareMap hardwareMap, Telemetry telemetry){
        this.telemetry=telemetry;
        motorL = hardwareMap.get(DcMotor.class, "leftMotor");
        motorR = hardwareMap.get(DcMotor.class, "rightMotor");
        pivot = hardwareMap.get(DcMotor.class, "pivot");
        assist = hardwareMap.get(DcMotor.class, "assist");

        sweep = hardwareMap.get(Servo.class, "sweep");

        lifter1=hardwareMap.get(Servo.class,"lifter1");
        lifter2=hardwareMap.get(Servo.class,"lifter2");

        marker=hardwareMap.get(Servo.class, "marker");


        lifter1.setDirection(Servo.Direction.REVERSE);
        lifter2.setDirection(Servo.Direction.FORWARD);

        frontSensor=hardwareMap.get(DistanceSensor.class, "front");
        frontSensor2=hardwareMap.get(DistanceSensor.class, "front2");
        sideRange1=hardwareMap.get(DistanceSensor.class, "sideRange1");
        sideRange2=hardwareMap.get(DistanceSensor.class, "sideRange2");

        motorL.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motorR.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        pivot.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        motorL.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        motorR.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        pivot.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

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
