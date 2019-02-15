package CVTesting;
import android.util.Log;

import com.disnodeteam.dogecv.CameraViewDisplay;
import com.disnodeteam.dogecv.DogeCV;
import com.disnodeteam.dogecv.detectors.roverrukus.GoldAlignDetector;
import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.hardware.bosch.JustLoggingAccelerationIntegrator;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.Func;
import org.firstinspires.ftc.robotcore.external.navigation.Acceleration;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.robotcore.external.navigation.Position;
import org.firstinspires.ftc.robotcore.external.navigation.Velocity;

import java.util.Locale;



//TODO: Add full JavaDoc
//TODO: Add full JavaDoc
//TODO: Add full JavaDoc
//TODO: (or at least add some comments)

@Disabled
@Autonomous(name = "CvSample", group = "Auto")
public class CVautotest extends LinearOpMode {

    Robot robot;

    private GoldAlignDetector detector;






    enum HangState{
        LOWER,
        LTWIST,
        WIGGLE,
        RTWIST,
        STOP;

        public HangState getNext(){
            return this.ordinal()<HangState.values().length-1?HangState.values()[this.ordinal()+1]:HangState.LOWER;
        }

    }
    HangState hangState=HangState.LOWER;


    //method that dictates everything to detach from the lander. It is run by a sub-statemachine, and returns true only when
    //the entire sub-statemachine has completed. This is to better control the flow of the detaching without clogging the main
    //statemachine with insignificant clutter states.
    private boolean lower() throws InterruptedException{

        switch(hangState){
            case LOWER:
                // Lower at 80% power

                Thread.sleep(2400);


                Thread.sleep(1000);

                hangState= hangState.getNext();

                break;
            case LTWIST:
                if(turnDegrees(.3,15)){
                    hangState= hangState.getNext();

                }
                break;
            case WIGGLE:
                motorDrive(-.7);
                Thread.sleep(300);
                hangState= hangState.getNext();


                break;
            case RTWIST:
                //if(turnDegrees(.3,-20)) {
                if(true){
                    hangState = hangState.getNext();
                }

                break;
            case STOP:
            default:
                return true;
        }

        return false;
    }

    enum FarRotState{
        ROT1,
        DRIVEWALL,
        ROT2,
        STOP;

        public FarRotState getNext(){
            return this.ordinal()< FarRotState.values().length-1? FarRotState.values()[this.ordinal()+1]: FarRotState.ROT1;
        }

    }
    FarRotState farRotState= FarRotState.ROT1;

    private boolean farRot() throws InterruptedException{

        switch(farRotState){
            case ROT1:
                if(absoluteTurnDeg(.2, 87+(startNearCrater?90:0))){
                    farRotState=farRotState.getNext();
                }
                break;
            case DRIVEWALL:
                motorDrive(-.7);
                if(robot.frontSensor.getDistance(DistanceUnit.CM)<60){
                    farRotState=farRotState.getNext();
                }
                break;
            case ROT2:
                if(turnDegrees(.3, 30)){
                    farRotState=farRotState.getNext();
                }
                break;
            case STOP:
            default:
                return true;
        }

        return false;
    }

    enum State{
        DETACH,
        MOVEBIT,
        FACEFARSAMPLE,
        SCANSAMPLE,
        FACEWALL,
        DELAYTIME,
        GOTOWALL,
        BACKUP,
        FACEDEPOT,
        GOTOWALL2,
        PREPDEPOSIT,
        DEPOSIT,
        FACECRATER,
        DRIVECRATER,
        STOP;

        public State getNext(){
            return this.ordinal()<State.values().length-1?State.values()[this.ordinal()+1]:State.DETACH;
        }

    }
    State state=State.DETACH;

    int encoderTickInitialR=0;
    int encoderTickInitialL=0;

    double headingInitial=0;
    double pitchInitial=0;

    int initialRange=10;

    double angleTolerance=2.6;
    int goodCounter=0;
    int angleTimeTolerance=50;



    void motorDrive(double power){
        robot.motorL.setPower(-power);
        robot.motorR.setPower(power);

    }
    private boolean driveTicks(double power, int ticks){
        double lPower=power;
        double rPower=power;


        int tickChangeR=robot.motorR.getCurrentPosition()-encoderTickInitialR;
        int tickChangeL=robot.motorL.getCurrentPosition()-encoderTickInitialL;

        if(tickChangeR-tickChangeL-50>0)
            rPower/=2;
        if(tickChangeL-tickChangeR-50>0)
            lPower/=2;

        if(ticks<0){lPower*=-1;rPower*=-1;}

        robot.motorR.setPower(rPower);
        robot.motorL.setPower(-lPower);

        return (ticks>0?robot.motorR.getCurrentPosition()-encoderTickInitialR>=ticks:robot.motorR.getCurrentPosition()-encoderTickInitialR<=ticks);
    }

    private boolean turnDegrees(double power, int degrees){
        double diff=heading-headingInitial-degrees;
        //distance (in degrees) remaining in the turn

        if(Math.abs(diff)>angleTolerance){
            goodCounter=0;
            robot.motorR.setPower((diff>0?1:-1)*power);

            if(Math.abs(diff)>8)
                robot.motorL.setPower((diff>0?1:-1)*power);
            else robot.motorL.setPower(0);
        }else{
            robot.motorL.setPower(0);
            robot.motorR.setPower(0);
            goodCounter++;
        }

        if(goodCounter>=angleTimeTolerance) {
            Log.i("hhs4227",String.format("heading: %.1f, revHeading: %.1f",heading,robot.angles.firstAngle));
            Log.i("hhs4227",String.format("headingInitial %.1f goodCounter %d, angleTimeTolerance: %d",headingInitial, goodCounter, angleTimeTolerance));
            Log.i("hhs4227",String.format("diff %.1f",diff));

            return true;
        }
        else {
            return false;
        }



    }

    private boolean smartTurn(double power, int degrees){
        double diff=heading-headingInitial-degrees;
        //difference (in degrees) remaining in the turn
        double totalDiff=degrees-headingInitial;
        //total delta theta for the duration of the turn


        double pow=power*diff/totalDiff;




        return false;
    }

    private boolean absoluteTurnDeg (double power, int degrees) {
        headingInitial = 0;
        return turnDegrees(power, degrees);
    }
    double timeAtStart=System.currentTimeMillis()/1000;
    double timeAtState=timeAtStart;

    private void next() {

        state=state.getNext();
        encoderTickInitialR=robot.motorR.getCurrentPosition();
        encoderTickInitialL=robot.motorR.getCurrentPosition();
        headingInitial=heading;
        pitchInitial=robot.angles.secondAngle;
        motorDrive(0);
        goodCounter=0;
        initialRange=(int)robot.sideRange1.getDistance(DistanceUnit.CM);
        Log.i("hhs4227",String.format("heading %.1f",heading)+"State: "+state);

        timeAtStart=System.currentTimeMillis()/1000;
    }

    private void driveTime(double power, long time) throws InterruptedException {
        robot.motorL.setPower(power);
        robot.motorR.setPower(power);
        Thread.sleep(time);
        robot.motorL.setPower(0);
        robot.motorR.setPower(0);

    }


    int skatingTolerance=2;
    double skatingDiff=.35;
    private void skate(double power){
        int d1=(int)robot.sideRange1.getDistance(DistanceUnit.CM);
        int d2=(int)robot.sideRange2.getDistance(DistanceUnit.CM);

        //backward is r- l+
        //if port 0 is larger then r becomes more neg else vice

        if(power>0){
            double r=-power;
            double l=power;
            if(d1-d2-skatingTolerance>0)
                r-=skatingDiff;
            if(d2-d1-skatingTolerance>0)
                r+=skatingDiff;

            robot.motorR.setPower(r);
            robot.motorL.setPower(l);


        }else{
            double r=-power;
            double l=power;
            if(d1-d2-skatingTolerance>0)
                r+=skatingDiff;
            if(d2-d1-skatingTolerance>0)
                r-=skatingDiff;

            robot.motorR.setPower(r);
            robot.motorL.setPower(l);
        }




    }

    private boolean skateDist(double power, int dist){
        skate(power);
        if(dist>0){
            return robot.motorR.getCurrentPosition()-encoderTickInitialR>dist-encoderTickInitialR;
        }
        return robot.motorR.getCurrentPosition()-encoderTickInitialR<dist-encoderTickInitialR;
    }




    //TODO:=============================================================================================
    boolean startHanging=true;
    boolean startNearCrater=true;
    boolean ownCrater =true;
    double timeDelay=.5;
//TODO:=============================================================================================

    @Override
    public void runOpMode() throws InterruptedException {


        // Set up detector
        detector = new GoldAlignDetector(); // Create detector
        detector.init(hardwareMap.appContext, CameraViewDisplay.getInstance()); // Initialize it with the app context and camera
        detector.useDefaults(); // Set detector to use default settings

        // Optional tuning
        detector.alignSize = 100; // How wide (in pixels) is the range in which the gold object will be aligned. (Represented by green bars in the preview)
        detector.alignPosOffset = 0; // How far from center frame to offset this alignment zone.
        detector.downscale = 0.4; // How much to downscale the input frames

        detector.areaScoringMethod = DogeCV.AreaScoringMethod.MAX_AREA; // Can also be PERFECT_AREA
        //detector.perfectAreaScorer.perfectArea = 10000; // if using PERFECT_AREA scoring
        detector.maxAreaScorer.weight = 0.005; //

        detector.ratioScorer.weight = 5; //
        detector.ratioScorer.perfectRatio = 1.0; // Ratio adjustment

        //detector.enable();

        this.robot=new Robot(hardwareMap, telemetry);


        boolean dDown=false;
        boolean dUp=false;
        boolean aDown=false;
        boolean bDown=false;
        boolean xDown=false;

        boolean isReady=false;

        composeTelemetry2();

        while(!isStarted()||!isReady){
            telemetry.update();
            if(gamepad1.dpad_down){dDown=true;}
            if(gamepad1.dpad_up){dUp=true;}
            if(gamepad1.a){aDown=true;}
            if(gamepad1.b){bDown=true;}
            if(gamepad1.x){xDown=true;}

            if(!gamepad1.dpad_down&&dDown){dDown=false;timeDelay-=.5;timeDelay=Math.abs(timeDelay);}
            if(!gamepad1.dpad_up&&dUp){dUp=false;timeDelay+=.5;}
            if(!gamepad1.a&&aDown){aDown=false;startHanging=!startHanging;}
            if(!gamepad1.b&&bDown){bDown=false;startNearCrater=!startNearCrater;}
            if(!gamepad1.x&&xDown){xDown=false;ownCrater=!ownCrater;}

            if(gamepad1.y){

                isReady=true;
                telemetry.clearAll();
                readyTelemetry();
                telemetry.update();
                break;
            }


        }

        waitForStart();

        // TODO: IBARGUEN - Is there a reason we can't do the next 5 lines before the waitForStart()?
        telemetry.clearAll();
        composeTelemetry();
        robot.marker.setPosition(0);

        robot.angles   = robot.imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
        robot.gravity  = robot.imu.getGravity();
        timeAtStart=System.currentTimeMillis()/1000;

        long lastTimeLogged=System.currentTimeMillis();

        while(!isStopRequested()){
            updateHeading();
            telemetry.update();

            switch(state){
                case DETACH:

                    if(startHanging){
                        if(lower()){
                            next();
                        }

                    }else {
                        next();
                    }
                    break;
                case MOVEBIT:

                    motorDrive(-.7);
//                    if(!startHanging&&startNearCrater)
//                        Thread.sleep(500);
                    Thread.sleep(300);
                    next();
                    detector.enable();

                    break;
                case FACEFARSAMPLE:
                    if(turnDegrees(.2, 50)) {
                        next();
                        robot.motorL.setPower(.2);
                        robot.motorR.setPower(.2);
                    }
                    break;
                case SCANSAMPLE:
                    //Log.i("hhs4227",String.format("x: %f, aligned: "+detector.getAligned(), detector.getXPosition()));

                    if(detector.getAligned()||detector.getXPosition()<400&&detector.getXPosition()>10){
                        robot.motorL.setPower(-.2);
                        robot.motorR.setPower(-.2);
                        Thread.sleep(400);
                        motorDrive(-.4);
                        Thread.sleep(1900);
                        motorDrive(.4);
                        Thread.sleep(1400);
                        detector.disable();
                        next();
                    }
                    if(heading<-45){
                        next();
                        detector.disable();
                    }
                    break;
                case FACEWALL:
                    //headingInitial = 0;
                    if(startNearCrater){
                        if(absoluteTurnDeg(.2,70)){
                            Log.i("hhs4227",String.format("FACEWALL nearCrater"));
                            next();
                        }
                    }else{
                        if(absoluteTurnDeg(.2,-65)){
                            Log.i("hhs4227",String.format("FACEWALL farCrater"));
                            next();
                        }
                    }
                    break;
                case DELAYTIME:
                    Thread.sleep((int)(timeDelay*1000));
                    Log.i("hhs4227",String.format("DELAYTIME"));
                    next();
                    break;
                case GOTOWALL:
                    motorDrive(-.8);
                    if(robot.frontSensor.getDistance(DistanceUnit.CM)<25){
                        Log.i("hhs4227",String.format("GOTOWALL"));
                        next();
                    }
                    break;
                case BACKUP:
                    next();
                    break;
                case FACEDEPOT:
                    if(startNearCrater) {
                        if (turnDegrees(.2, 55)) {
                            Log.i("hhs4227",String.format("FACEDEPOT nearcrater"));
                            next();
                        }
                    }else{
                        if (turnDegrees(.2, 100)) {
                            Log.i("hhs4227",String.format("FACEDEPOT farcrater"));
                            next();
                        }
                    }
                    break;

                case GOTOWALL2:
                    skate(.8);

                    if(System.currentTimeMillis()-lastTimeLogged>500){
                        lastTimeLogged=System.currentTimeMillis();
                        Log.i("hhs4227", "Time (Seconds): "+System.currentTimeMillis()/1000+", State(S)"+timeAtState);

                    }
//                    if(System.currentTimeMillis()%500<100){
//                        Log.i("hhs4227", "Time (Seconds): "+System.currentTimeMillis()/1000+", State(S)"+timeAtState);
//                    }

                    if(System.currentTimeMillis()/1000-timeAtState>2&&robot.frontSensor.getDistance(DistanceUnit.CM)<25){
                        Log.i("hhs4227", "End!!! Time (Seconds): "+System.currentTimeMillis()/1000+", State(S)"+timeAtState);

                        next();
                    }
                    break;
                case PREPDEPOSIT:
                    if(ownCrater){
                        next();
                    }
                    else{
                        if(turnDegrees(.25, -90)){
                            next();
                        }
                    }


                    break;

                case DEPOSIT:
                    robot.marker.setPosition(.7);
                    Thread.sleep(300);
                    motorDrive(.7);
                    Thread.sleep(800);
                    next();


                    break;

                case FACECRATER:
                    if(!ownCrater) {
                        if(turnDegrees(.3, 180)){
                            next();
                        }
                    }else{
                        next();
                    }

                    break;




                case DRIVECRATER:

                    if(!ownCrater){
                        skate(.6);
                    }else {
                        skate(-.7);
                    }
                    int diff=Math.abs(robot.motorR.getCurrentPosition()-encoderTickInitialR);
                    if(robot.angles.secondAngle-pitchInitial>5||robot.angles.secondAngle-pitchInitial<-5||diff>4200){
                        next();
                    }

                    break;

                case STOP:
                default:
                    telemetry.update();
                    requestOpModeStop();

            }

        }





    }

    double heading;
    Orientation lastAngles = new Orientation();
    private double updateHeading() {

        // How much has the angle changed since the last check?
        double deltaAngle = robot.angles.firstAngle - lastAngles.firstAngle;

        // This will make sure that the deltaAngle is never more than 180
        // degrees in either direction.  What is the purpose of this?
        // TODO: This won't work if the deltaAngle is really big
        if (deltaAngle < -180)
            deltaAngle += 360;
        else if (deltaAngle > 180)
            deltaAngle -= 360;

        heading += deltaAngle;

        lastAngles = robot.angles;

        return heading;
    }

    void composeTelemetry() {
        telemetry.clearAll();

        // At the beginning of each telemetry update, grab a bunch of data
        // from the IMU that we will then display in separate lines.
        telemetry.addAction(new Runnable() { @Override public void run()
        {
            // Acquiring the angles is relatively expensive; we don't want
            // to do that in each of the three items that need that info, as that's
            // three times the necessary expense.
            robot.angles   = robot.imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
            robot.gravity  = robot.imu.getGravity();
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
                        return robot.motorL.getPower()+"/"+robot.motorR.getPower();
                    }
                });

        telemetry.addLine()
                .addData("heading", new Func<String>() {
                    @Override public String value() {
                        return heading+"";
                    }
                })
                .addData("roll", new Func<String>() {
                    @Override public String value() {
                        return formatAngle(robot.angles.angleUnit, robot.angles.secondAngle);
                    }
                })
                .addData("pitch", new Func<String>() {
                    @Override public String value() {
                        return formatAngle(robot.angles.angleUnit,robot.angles.thirdAngle);
                    }
                });

        telemetry.addLine()
                .addData("State: ", new Func<String>() {
                    @Override public String value() {
                        if(state==State.DETACH)
                            return "DETACH: "+hangState;
                        if(state==State.FACECRATER){
                            return "FACECRATER: "+farRotState;
                        }
                        return state+"";

                    }
                })
                ;
        telemetry.addLine()
                .addData("enc1", new Func<String>() {
                    @Override public String value() {
                        return robot.motorL.getCurrentPosition()+"";
                    }
                })
                .addData("encR", new Func<String>() {
                    @Override public String value() {
                        return ""+robot.motorR.getCurrentPosition();
                    }
                });
        telemetry.addLine()
                .addData("FramesWithinTolerance:", new Func<String>() {
                    @Override public String value() {
                        return goodCounter+"/"+angleTimeTolerance;
                    }
                });
        telemetry.addLine()
                .addData("X:", new Func<String>() {
                    @Override public String value() {
                        return detector.getXPosition()+"";
                    }
                })
                .addData("Align:", new Func<String>() {
            @Override public String value() {
                return detector.getAligned()+"";
            }
        });
    }

    String line1="Initializing Autonomous... Press <Y> to finish!";
    String line2="<A> Start hanging?";
    String line3="<X> End at own crater? ";
    String line4="<B> Start near crater? ";
    String line5="<DPAD> Delay time ";

    void composeTelemetry2() {
        telemetry.clearAll();
        // At the beginning of each telemetry update, grab a bunch of data
        // from the IMU that we will then display in separate lines.
        telemetry.addAction(new Runnable() { @Override public void run()
        {
            // Acquiring the angles is relatively expensive; we don't want
            // to do that in each of the three items that need that info, as that's
            // three times the necessary expense.

        }
        });

        telemetry.addLine()
                .addData(line1, new Func<String>() {
                    @Override public String value() {
                        return "";
                    }
                });
        telemetry.addLine();
        telemetry.addLine()
                .addData(line2, new Func<String>() {
                    @Override public String value() {
                        return startHanging?"Yes":"No";
                    }
                });
        telemetry.addLine()
                .addData(line3, new Func<String>() {
                    @Override public String value() {

                        return ownCrater?"Yes":"No";

                    }
                });
        telemetry.addLine()
                .addData(line4, new Func<String>() {
                    @Override public String value() {
                        return startNearCrater?"Yes":"No";
                    }
                });
        telemetry.addLine()
                .addData(line5, new Func<String>() {
                    @Override public String value() {
                        return timeDelay+"";
                    }
                });


    }
    void readyTelemetry(){
        telemetry.clearAll();
        // At the beginning of each telemetry update, grab a bunch of data
        // from the IMU that we will then display in separate lines.
        telemetry.addAction(new Runnable() { @Override public void run()
        {


        }
        });

        telemetry.addLine()
                .addData("Autonomous ready!", new Func<String>() {
                    @Override public String value() {
                        return "";
                    }
                });
    }

    String formatAngle(AngleUnit angleUnit, double angle) {
        return formatDegrees(AngleUnit.DEGREES.fromUnit(angleUnit, angle));
    }

    String formatDegrees(double degrees){
        return String.format(Locale.getDefault(), "%.1f", AngleUnit.DEGREES.normalize(degrees));
    }
}
