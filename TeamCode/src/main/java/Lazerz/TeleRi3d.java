package Lazerz;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

@TeleOp(name = "TeleRi3d", group = "ZZZ_Template")
public class TeleRi3d extends OpMode {
    DcMotor motorL;
    DcMotor motorR;
    Servo lServo;
    Servo rServo;
    DistanceSensor sensor;

    @Override
    public void init() {
        motorL = hardwareMap.get(DcMotor.class, "leftMotor");
        motorR = hardwareMap.get(DcMotor.class, "rightMotor");
        lServo=hardwareMap.get(Servo.class,"lServo");
        rServo=hardwareMap.get(Servo.class,"rServo");
        lServo.setDirection(Servo.Direction.REVERSE);
        rServo.setDirection(Servo.Direction.FORWARD);

        sensor=hardwareMap.get(DistanceSensor.class,"sensor");
    }

    @Override
    public void loop() {
        telemetry.addData("Distance:", sensor.getDistance(DistanceUnit.METER));

        double powerL = gamepad1.left_stick_y;
        double powerR = gamepad1.right_stick_y;



        if(gamepad1.dpad_up){
            lServo.setPosition(.1);
            rServo.setPosition(.1);
        }else if(gamepad1.dpad_down){
            lServo.setPosition(.9);
            rServo.setPosition(.9);
        }else{
            lServo.setPosition(0);
            rServo.setPosition(0);
        }

//        if(gamepad1.a) {
//            claw.setPower(.5);
//        }else if(gamepad1.b){
//            claw.setPower(.1);
//        }claw.setPower(0);

        powerL*=-1;
        powerL = Range.clip(powerL, -1, 1);
        powerR = Range.clip(powerR, -1, 1);

        motorL.setPower(powerL);
        motorR.setPower(powerR);



    }
}

