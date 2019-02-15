package OfficialOpmodes;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
//import com.qualcomm.robotcore.hardware.DcMotorSimple;
//import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.Func;
//import org.firstinspires.ftc.robotcore.external.navigation.Acceleration;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;

import java.util.Locale;

import CVTesting.Robot;
import Helpers.CommonMath;
import Helpers.PivotScale;
import Helpers.RangePlus;
import Helpers.Scale2;

@TeleOp(name = "faster?", group = "OfficialOpmodes")
public class faster extends OpMode {

    Robot robot;


    @Override
    public void init() {
//        robot.init(hardwareMap);
        this.robot=new Robot(hardwareMap, telemetry);

        lastTimeLogged=System.currentTimeMillis();
    }


    private boolean a1=false;



    private boolean slowMode=false;

    private double powerL;
    private double powerR;

    private int minPos=0;



    long lastTimeLogged=System.currentTimeMillis();
    int diff;

    boolean armSlowMode=true;

    int minExtensionPosition=0;

    int mult=1;
    double disp;
    double armPower;
    @Override
    public void loop() {
        diff=(int)(System.currentTimeMillis()-lastTimeLogged)/1000;
        if(robot.reel.getCurrentPosition()<minExtensionPosition)minExtensionPosition=robot.reel.getCurrentPosition();
        int armDiff=robot.reel.getCurrentPosition()-minExtensionPosition;



        if(gamepad1.a){a1=true;}
        else if(a1){
            a1=false;
            slowMode=!slowMode;
        }
        if(armDiff<2000) {
            robot.reel.setPower(-gamepad2.right_stick_y);
        }else{
            robot.reel.setPower(-1);
        }


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
            robot.latch.setPosition(0);
        }
        if(gamepad2.right_trigger>.1){
            robot.latch.setPosition(.75);
        }



        if(gamepad2.x){
            robot.sweep.setPosition(.4);
        }else if(gamepad2.left_bumper){
            robot.sweep.setPosition(.1);
        }else if(gamepad2.right_bumper){
            robot.sweep.setPosition(.9);
        }else{
            robot.sweep.setPosition(.5);
        }


        disp=((double)robot.pivot.getCurrentPosition()-(double)minPos)/1200;
        //find the angle the robot is at, or more accurately a percentage between 0 degrees and 90 degrees
        if(gamepad2.dpad_right)
            armSlowMode=true;
        if(gamepad2.dpad_left)
            armSlowMode=false;

        if(armSlowMode)
            robot.pivot.setPower(-gamepad2.left_stick_y/6*(1+Math.abs(Math.cos(disp*Math.PI/2))));
        if(!armSlowMode)
            robot.pivot.setPower(-gamepad2.left_stick_y/3*(1+Math.abs(Math.cos(disp*Math.PI/2))));

        //Map the joystick value to the composite function between linear and trig, the function is shown above


//        if(gamepad2.left_stick_x>=.6)armSlowMode=true;
//        else if(gamepad2.left_stick_x<=-.6)armSlowMode=false;
//
//        mult=armSlowMode?1:2;
//        if(gamepad2.y) {
//            //OVERRIDE CODE
//            robot.pivot.setPower(-mult * gamepad2.left_stick_y / CommonMath.abs(gamepad2.left_stick_y) * PivotScale.getPow(20));
//            minPos = robot.pivot.getCurrentPosition();
//        }else{
//            //NOT OVERRIDEN, NOT BLOCKING MOVEMENT
//            if(robot.pivot.getCurrentPosition()-minPos>120) {
//                robot.pivot.setPower(-mult * gamepad2.left_stick_y / CommonMath.abs(gamepad2.left_stick_y) * PivotScale.getPow(robot.pivot.getCurrentPosition()-minPos));
//            }else{
//                //NOT OVERRRIDEN, BLOCKING MOVEMENT
//                robot.pivot.setPower(RangePlus.constrain(-mult * gamepad2.left_stick_y / CommonMath.abs(gamepad2.left_stick_y) * PivotScale.getPow(robot.pivot.getCurrentPosition()-minPos), 0, 1));
//            }
//
//        }
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

        robot.lights.setPosition(.6019958);

        if(diff>90&&diff<95)
            robot.lights.setPosition(.4819958);
        if(gamepad1.dpad_up||gamepad2.dpad_up) {
            robot.hang.setPower(-1);
            robot.pivot.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        }
        else if(gamepad1.dpad_down||gamepad2.dpad_down) {
            robot.hang.setPower(1);
            robot.pivot.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        }
        else {
            robot.hang.setPower(0);
            robot.pivot.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        }




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
                .addData("Disp/power", new Func<String>() {
                    @Override public String value() {
                        return disp+"/"+armPower;
                    }
                })
                .addData("Math.cos(Math.PI/2*disp);", new Func<String>() {
                    @Override public String value() {
                        return Math.cos(Math.PI/2*disp)+"";
                    }
                });

        telemetry.addLine()
                .addData("heading", new Func<String>() {
                    @Override public String value() {
                        return formatAngle(robot.angles.angleUnit, robot.angles.firstAngle);
                    }
                })
                .addData("Reel Position", new Func<String>() {
                    @Override public String value() {
                        return ""+robot.reel.getCurrentPosition();
                    }
                })
                .addData("powthing", new Func<String>() {
                    @Override public String value() {
                        return ""+robot.pivot.getPower();
                    }
                });

        telemetry.addLine()
                .addData("ElapsedTime", new Func<String>() {
                    @Override public String value() {
                        return ""+diff;
                    }
                })
                .addData("swepos", new Func<String>() {
                    @Override public String value() {
                        return ""+robot.sweep.getPosition();
                    }
                });
        telemetry.addLine()
                .addData("pivot pos/pow", new Func<String>() {
                    @Override public String value() {
                        return robot.pivot.getCurrentPosition()+"/"+robot.pivot.getPower();
                    }
                })
                .addData("ARM SLOW MODE:", new Func<String>() {
                    @Override public String value() {
                        return ""+armSlowMode;
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

