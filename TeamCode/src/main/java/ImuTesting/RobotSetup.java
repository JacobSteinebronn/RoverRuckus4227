package ImuTesting;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.hardware.bosch.JustLoggingAccelerationIntegrator;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.HardwareDevice;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.navigation.Acceleration;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.robotcore.external.navigation.Position;
import org.firstinspires.ftc.robotcore.external.navigation.Velocity;

/**
 * Created by HHS-ROBOTICS on 10/3/2018.
 */

public class RobotSetup {

    public RobotSetup(){

    }

    public DcMotor motorL;
    public DcMotor motorR;
    public Servo lServo;
    public Servo rServo;
    public DistanceSensor sensor1;
    public DistanceSensor sensor2;


    HardwareMap hwMap;



    public void init(HardwareMap ahwMap) {
        hwMap = ahwMap;

        motorL = hwMap.get(DcMotor.class, "leftMotor");
        motorR = hwMap.get(DcMotor.class, "rightMotor");
        lServo=hwMap.get(Servo.class,"lServo");
        rServo=hwMap.get(Servo.class,"rServo");



        lServo.setDirection(Servo.Direction.REVERSE);
        rServo.setDirection(Servo.Direction.FORWARD);

        sensor1=hwMap.get(DistanceSensor.class, "s1");
        sensor2=hwMap.get(DistanceSensor.class, "s2");


        motorL.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motorR.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        motorL.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        motorR.setMode(DcMotor.RunMode.RUN_USING_ENCODER);



    }





}
