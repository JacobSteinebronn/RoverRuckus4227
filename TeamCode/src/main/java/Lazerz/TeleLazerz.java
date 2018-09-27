package Lazerz;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

@TeleOp(name = "TeleLazerz", group = "ZZZ_Template")
public class TeleLazerz extends OpMode {
    DcMotor motorL;
    DcMotor motorR;
    Servo lServo;
    Servo rServo;
    DistanceSensor sensor1;
    DistanceSensor sensor2;
    final int NUM_SENSOR_VALUES_MAX =10;
    int num_sensor_values=5;

    double autoAlignPower=.35;
    boolean autoAlign=false;

    double[] s1values=new double[NUM_SENSOR_VALUES_MAX];
    int s1i=0;

    double[] s2values=new double[NUM_SENSOR_VALUES_MAX];
    int s2i=0;
    int tolerance=5;

    @Override
    public void init() {
        motorL = hardwareMap.get(DcMotor.class, "leftMotor");
        motorR = hardwareMap.get(DcMotor.class, "rightMotor");
        lServo=hardwareMap.get(Servo.class,"lServo");
        rServo=hardwareMap.get(Servo.class,"rServo");
        lServo.setDirection(Servo.Direction.REVERSE);
        rServo.setDirection(Servo.Direction.FORWARD);

        sensor1=hardwareMap.get(DistanceSensor.class, "s1");
        sensor2=hardwareMap.get(DistanceSensor.class, "s2");
    }
    boolean aWasPressed;
    boolean leftPressed;
    boolean rightPressed;
    boolean rightBumper;
    boolean leftBumper;


    @Override
    public void loop() {
        if(gamepad1.right_bumper){
            rightBumper=true;
        }else if(rightBumper){
            rightBumper=false;
            tolerance++;
        }
        if(gamepad1.left_bumper){
            leftBumper=true;
        }else if(leftBumper){
            leftBumper=false;
            tolerance--;
        }



        if(gamepad1.a){
            aWasPressed=true;
        }else if(aWasPressed){
            aWasPressed=false;
            autoAlign=!autoAlign;
        }
        if(gamepad1.dpad_left)leftPressed=true;
        else if(leftPressed){
            leftPressed=false;
            num_sensor_values--;

        }
        if(gamepad1.dpad_right)rightPressed=true;
        else if(rightPressed){
            rightPressed=false;
            num_sensor_values++;

        }

        if(autoAlign){
            if(getAverage(s1values)-tolerance-getAverage(s2values)>0){
                motorL.setPower(-autoAlignPower);
                motorR.setPower(-autoAlignPower);
            }else if(getAverage(s2values)-tolerance-getAverage(s1values)>0){
                motorL.setPower(autoAlignPower);
                motorR.setPower(autoAlignPower);
            }else{
                motorL.setPower(0);
                motorR.setPower(0);
            }

        }else{

            double powerL = gamepad1.left_stick_y;
            double powerR = gamepad1.right_stick_y;

            powerL*=-1;
            powerL = Range.clip(powerL, -1, 1);
            powerR = Range.clip(powerR, -1, 1);

            motorL.setPower(powerL);
            motorR.setPower(powerR);
        }



        if(sensor1.getDistance(DistanceUnit.CM)<400)s1values[s1i++]=sensor1.getDistance(DistanceUnit.CM);
        if(sensor2.getDistance(DistanceUnit.CM)<400)s2values[s2i++]=sensor2.getDistance(DistanceUnit.CM);

        telemetry.addData("D1:", getAverage(s1values));
        telemetry.addData("D2:",getAverage(s2values));
        telemetry.addData("Sensitivity:", num_sensor_values);
        telemetry.addData("AutoAlign? ",autoAlign);
        telemetry.addData("Tolerance:",tolerance);



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

        s2i%= num_sensor_values;
        s1i%= num_sensor_values;

    }
    public double getAverage(double[] arr){
        double sum=0;
        for(int i=0;i<num_sensor_values;i++){sum+=arr[i];}
        return sum/num_sensor_values-sum/num_sensor_values%1;
    }
}

