package OfficialOpmodes;
import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.hardware.bosch.JustLoggingAccelerationIntegrator;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.Func;
import org.firstinspires.ftc.robotcore.external.navigation.Acceleration;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.robotcore.external.navigation.Position;
import org.firstinspires.ftc.robotcore.external.navigation.Velocity;

import java.util.Locale;

import Helpers.Scale2;

@TeleOp(name = "Teleop", group = "OfficialOpmodes")
public class TeleOpOfficial extends OpMode {

    public DcMotor motorL;
    public DcMotor motorR;
    public Servo lifter1;
    public Servo lifter2;


    public DistanceSensor sideRange1;
    public DistanceSensor sideRange2;
    public DistanceSensor frontSensor;
    public Servo marker;

    BNO055IMU imu;

    Orientation angles;
    Acceleration gravity;


    @Override
    public void init() {
//        robot.init(hardwareMap);

        motorL = hardwareMap.get(DcMotor.class, "leftMotor");
        motorR = hardwareMap.get(DcMotor.class, "rightMotor");

        lifter1=hardwareMap.get(Servo.class,"lifter1");
        lifter2=hardwareMap.get(Servo.class,"lifter2");

        marker=hardwareMap.get(Servo.class, "marker");


        lifter1.setDirection(Servo.Direction.REVERSE);
        lifter2.setDirection(Servo.Direction.FORWARD);

        frontSensor=hardwareMap.get(DistanceSensor.class, "front");
        sideRange1=hardwareMap.get(DistanceSensor.class, "sideRange1");
        sideRange2=hardwareMap.get(DistanceSensor.class, "sideRange2");

        motorL.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motorR.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        motorL.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        motorR.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
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


        composeTelemetry();


    }
    boolean aWasPressed;
    boolean leftPressed;
    boolean rightPressed;
    boolean rightBumper;
    boolean leftBumper;


    boolean a1=false;



    boolean slowMode=false;

    double powerL;
    double powerR;
    @Override
    public void loop() {
        telemetry.update();

        if(gamepad1.a){a1=true;}
        else if(a1){
            a1=false;
            slowMode=!slowMode;
        }


        powerL = gamepad1.right_stick_y;
        powerR = gamepad1.left_stick_y;

        powerL*=-1;
        powerL = Range.clip(powerL, -1, 1);
        powerR = Range.clip(powerR, -1, 1);

        powerL= Scale2.scaleInput(1, slowMode?powerL/2:powerL);
        powerR= Scale2.scaleInput(1, slowMode?powerR/2:powerR);


        double liftL=gamepad2.right_stick_y;
        double liftR=gamepad2.left_stick_y;

//        liftL/=2;
//        liftR/=2;
//        liftL+=0.5;
//        liftR+=0.5;
//        lifter1.setPosition(liftL);
//        lifter2.setPosition(liftR);




        motorL.setPower(powerL);                                     
        motorR.setPower(powerR);                                     
        if(gamepad1.a||gamepad2.a){                                  
            marker.setPosition(0);                                   
        }                                                            
        if(gamepad1.b||gamepad2.b){                                  
            marker.setPosition(1);                                   
                                                                     
        }
        if(gamepad1.dpad_up||gamepad2.dpad_up||gamepad2.right_stick_y<-.5)
            lifter1.setPosition(.1);
        else if(gamepad1.dpad_down||gamepad2.dpad_down||gamepad2.right_stick_y>.5)
            lifter1.setPosition(.9);
        else
            lifter1.setPosition(.5);

        if(gamepad1.dpad_up||gamepad2.dpad_up||gamepad2.left_stick_y<-.5)
            lifter2.setPosition(.1);
        else if(gamepad1.dpad_down||gamepad2.dpad_down||gamepad2.left_stick_y>.5)
            lifter2.setPosition(.9);
        else
            lifter2.setPosition(.5);


//        if(gamepad1.dpad_up||gamepad2.dpad_up){
//            lifter1.setPosition(.1);
//            lifter2.setPosition(.1);
//        }else if(gamepad1.dpad_down||gamepad2.dpad_down){
//            lifter1.setPosition(.9);
//            lifter2.setPosition(.9);
//        }else{
//            lifter1.setPosition(0);
//            lifter2.setPosition(0);
//        }

    }

    void composeTelemetry() {

        // At the beginning of each telemetry update, grab a bunch of data
        // from the IMU that we will then display in separate lines.
        telemetry.addAction(new Runnable() { @Override public void run()
        {
            // Acquiring the angles is relatively expensive; we don't want
            // to do that in each of the three items that need that info, as that's
            // three times the necessary expense.
            angles   = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
            gravity  = imu.getGravity();
        }
        });

        telemetry.addLine()
                .addData("Gamepad L/R:", new Func<String>() {
                    @Override public String value() {
                        return gamepad1.left_stick_y+"/"+gamepad1.right_stick_y;
                    }
                })
                .addData("Power L/R:", new Func<String>() {
                    @Override public String value() {
                        return powerL+"/"+powerR;
                    }
                });

        telemetry.addLine()
                .addData("heading", new Func<String>() {
                    @Override public String value() {
                        return formatAngle(angles.angleUnit, angles.firstAngle);
                    }
                })
                .addData("roll", new Func<String>() {
                    @Override public String value() {
                        return formatAngle(angles.angleUnit, angles.secondAngle);
                    }
                })
                .addData("pitch", new Func<String>() {
                    @Override public String value() {
                        return formatAngle(angles.angleUnit, angles.thirdAngle);
                    }
                });

        telemetry.addLine()
                .addData("scaleinput L/R", new Func<String>() {
                    @Override public String value() {
                        return Scale2.scaleInput(1, gamepad1.left_stick_y)+"/"+Scale2.scaleInput(1, gamepad1.right_stick_y);
                    }
                })
                .addData("scaleinput of .5", new Func<String>() {
                    @Override public String value() {
                        return ""+Scale2.scaleInput(1, .5);
                    }
                });
        telemetry.addLine()
                .addData("enc1", new Func<String>() {
                    @Override public String value() {
                        return motorL.getCurrentPosition()+"";
                    }
                })
                .addData("encR", new Func<String>() {
                    @Override public String value() {
                        return ""+motorR.getCurrentPosition();
                    }
                });
    }

    //----------------------------------------------------------------------------------------------
    // Formatting jasffkgjjaofigjl
    //----------------------------------------------------------------------------------------------

    String formatAngle(AngleUnit angleUnit, double angle) {
        return formatDegrees(AngleUnit.DEGREES.fromUnit(angleUnit, angle));
    }

    String formatDegrees(double degrees){
        return String.format(Locale.getDefault(), "%.1f", AngleUnit.DEGREES.normalize(degrees));
    }

}

