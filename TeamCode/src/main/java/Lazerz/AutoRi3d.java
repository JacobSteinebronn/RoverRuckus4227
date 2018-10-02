package Lazerz;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.Servo;

/**
 * Created by hhs-robotics on 8/1/2018.
 */
@Disabled
@Autonomous(name = "AutoRi3d", group = "Auto")
public class AutoRi3d extends LinearOpMode {

    DcMotor motorL;
    DcMotor motorR;
    Servo lServo;
    Servo rServo;
    DistanceSensor sensor;

    private void drive(double power, long time) throws InterruptedException {
        motorL.setPower(power);
        motorR.setPower(power);
        Thread.sleep(time);
        motorL.setPower(0);
        motorR.setPower(0);

    }

    private void turn(double powerL, double powerR, long time) throws InterruptedException{
        motorL.setPower(powerL);
        motorR.setPower(powerR);
        Thread.sleep(time);
        motorL.setPower(0);
        motorR.setPower(0);
    }
    @Override
    public void runOpMode() throws InterruptedException {
        motorL = hardwareMap.get(DcMotor.class, "leftMotor");
        motorR = hardwareMap.get(DcMotor.class, "rightMotor");
        lServo=hardwareMap.get(Servo.class,"lServo");
        rServo=hardwareMap.get(Servo.class,"rServo");
        lServo.setDirection(Servo.Direction.REVERSE);
        rServo.setDirection(Servo.Direction.FORWARD);

        sensor=hardwareMap.get(DistanceSensor.class,"sensor");

        motorR.setDirection(DcMotorSimple.Direction.REVERSE);

        waitForStart();

        lServo.setPosition(.1);
        rServo.setPosition(.1);
        Thread.sleep(3400);
        lServo.setPosition(0);
        rServo.setPosition(0);
        motorL.setPower(.4);
        motorR.setPower(-.2);
        Thread.sleep(900);
        motorR.setPower(.4);
        Thread.sleep(500);
        motorR.setPower(0);
        motorL.setPower(-.4);

        Thread.sleep(900);
        motorR.setPower(-.4);
        motorL.setPower(-.4);
        Thread.sleep(2000);
        motorR.setPower(0);
        motorL.setPower(0);





    }
}
