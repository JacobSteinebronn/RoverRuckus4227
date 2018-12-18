package OfficialOpmodes;
import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.hardware.bosch.JustLoggingAccelerationIntegrator;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
//import com.qualcomm.robotcore.hardware.DcMotorSimple;
//import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.Func;
//import org.firstinspires.ftc.robotcore.external.navigation.Acceleration;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.robotcore.external.navigation.Position;
import org.firstinspires.ftc.robotcore.external.navigation.Velocity;

import java.util.Locale;

import CVTesting.Robot;
import Helpers.CommonMath;
import Helpers.PivotScale;
import Helpers.RangePlus;
import Helpers.Scale2;

@TeleOp(name = "Teleop", group = "OfficialOpmodes")
public class TeleOpOfficial extends OpMode {

    Robot robot;


    @Override
    public void init() {
//        robot.init(hardwareMap);
        this.robot=new Robot(hardwareMap, telemetry);
        composeTelemetry();

    }


    private boolean a1=false;



    private boolean slowMode=false;

    private double powerL;
    private double powerR;
    @Override
    public void loop() {
        telemetry.update();

        if(gamepad1.a){a1=true;}
        else if(a1){
            a1=false;
            slowMode=!slowMode;
        }

        robot.assist.setPower(gamepad2.right_stick_y);


        powerR = gamepad1.right_stick_y;
        powerL = gamepad1.left_stick_y;

        powerL*=-1;
        powerL = Range.clip(powerL, -1, 1);
        powerR = Range.clip(powerR, -1, 1);

        powerL= Scale2.scaleInput(1, slowMode?powerL/2:powerL);
        powerR= Scale2.scaleInput(1, slowMode?powerR/2:powerR);

        robot.motorL.setPower(powerL);
        robot.motorR.setPower(powerR);

        if(gamepad2.left_trigger>.1){
            robot.lifter1.setPosition(.9);
            robot.lifter2.setPosition(.9);
            robot.assist.setPower(.7);
        }



        if(gamepad2.x){
            robot.sweep.setPosition(.4);
        }else if(gamepad2.y){
            robot.sweep.setPosition(.6);
        }else if(gamepad2.left_bumper){
            robot.sweep.setPosition(.3);
        }else if(gamepad2.right_bumper){
            robot.sweep.setPosition(.7);
        }else{
            robot.sweep.setPosition(.5);
        }

        if(robot.pivot.getCurrentPosition()>120) {
            robot.pivot.setPower(-3 * gamepad2.left_stick_y / CommonMath.abs(gamepad2.left_stick_y) * PivotScale.getPow(robot.pivot.getCurrentPosition()));
        }else{
            robot.pivot.setPower(RangePlus.constrain(-2 * gamepad2.left_stick_y / CommonMath.abs(gamepad2.left_stick_y) * PivotScale.getPow(robot.pivot.getCurrentPosition()), 0, 1));
        }
//        if(gamepad2.left_stick_y>.5)
//            pivot.setPower(.25);
//        else if(gamepad2.left_stick_y<-.5){
//            pivot.setPower(-.25);
//        }else{
//            pivot.setPower(0);
//        }


        if(gamepad2.a){
            robot.marker.setPosition(0);
        }                                                            
        if(gamepad2.b){
            robot.marker.setPosition(1);
                                                                     
        }
        if(gamepad1.dpad_up||gamepad2.dpad_up)
            robot.lifter1.setPosition(.1);
        else if(gamepad1.dpad_down||gamepad2.dpad_down)
            robot.lifter1.setPosition(.9);
        else
            robot.lifter1.setPosition(.5);

        if(gamepad1.dpad_left||gamepad2.dpad_left)
            robot.lifter2.setPosition(.1);
        else if(gamepad1.dpad_right||gamepad2.dpad_right)
            robot.lifter2.setPosition(.9);
        else
            robot.lifter2.setPosition(.5);


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

    private void composeTelemetry() {

        // At the beginning of each telemetry update, grab a bunch of data
        // from the IMU that we will then display in separate lines.
        telemetry.addAction(new Runnable() { @Override public void run()
        {
            // Acquiring the angles is relatively expensive; we don't want
            // to do that in each of the three items that need that info, as that's
            // three times the necessary expense.
            robot.angles   = robot.imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
            //gravity  = imu.getGravity();
        }
        });

        telemetry.addLine()
                .addData("GamePad L/R:", new Func<String>() {
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
                        return formatAngle(robot.angles.angleUnit, robot.angles.firstAngle);
                    }
                })
                .addData("roll", new Func<String>() {
                    @Override public String value() {
                        return formatAngle(robot.angles.angleUnit, robot.angles.secondAngle);
                    }
                })
                .addData("powthing", new Func<String>() {
                    @Override public String value() {
                        return ""+robot.pivot.getPower();
                    }
                });

        telemetry.addLine()
                .addData("scaleInput L/R", new Func<String>() {
                    @Override public String value() {
                        return Scale2.scaleInput(1, gamepad1.left_stick_y)+"/"+Scale2.scaleInput(1, gamepad1.right_stick_y);
                    }
                })
                .addData("swepos", new Func<String>() {
                    @Override public String value() {
                        return ""+robot.sweep.getPosition();
                    }
                });
        telemetry.addLine()
                .addData("pivot pos", new Func<String>() {
                    @Override public String value() {
                        return robot.pivot.getCurrentPosition()+"";
                    }
                })
                .addData("encR", new Func<String>() {
                    @Override public String value() {
                        return ""+robot.motorR.getCurrentPosition();
                    }
                });
    }

    //----------------------------------------------------------------------------------------------
    // Formatting jasffkgjjaofigjl
    //----------------------------------------------------------------------------------------------

    private String formatAngle(AngleUnit angleUnit, double angle) {
        return formatDegrees(AngleUnit.DEGREES.fromUnit(angleUnit, angle));
    }

    private String formatDegrees(double degrees){
        return String.format(Locale.getDefault(), "%.1f", AngleUnit.DEGREES.normalize(degrees));
    }

}

