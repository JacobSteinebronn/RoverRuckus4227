package TestingRandomStuff;
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
@Autonomous(name = "TurnTest", group = "Auto")
public class RevisedTurning extends LinearOpMode {

    private GoldAlignDetector detector;

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
                lifter1.setPosition(0.1);
                lifter2.setPosition(0.1);

                Thread.sleep(2400);

                // Stop the first servo and let the other one continue
                // This gets the hook sideways so it is easier to twist off
                lifter2.setPosition(.5);
                Thread.sleep(1000);
                lifter1.setPosition(.5);
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
                if(frontSensor.getDistance(DistanceUnit.CM)<60){
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
        TURNONE,
        TURNTWO,
        TURNTHREE,
        STOP;

        public State getNext(){
            return this.ordinal()<State.values().length-1?State.values()[this.ordinal()+1]:State.TURNONE;
        }

    }
    State state=State.TURNONE;

    int encoderTickInitialR=0;
    int encoderTickInitialL=0;

    double headingInitial=0;
    double pitchInitial=0;

    int initialRange=10;

    double angleTolerance=2.6;
    int goodCounter=0;
    int angleTimeTolerance=50;



    void motorDrive(double power){
        motorL.setPower(-power);
        motorR.setPower(power);

    }
    private boolean driveTicks(double power, int ticks){
        double lPower=power;
        double rPower=power;


        int tickChangeR=motorR.getCurrentPosition()-encoderTickInitialR;
        int tickChangeL=motorL.getCurrentPosition()-encoderTickInitialL;

        if(tickChangeR-tickChangeL-50>0)
            rPower/=2;
        if(tickChangeL-tickChangeR-50>0)
            lPower/=2;

        if(ticks<0){lPower*=-1;rPower*=-1;}

        motorR.setPower(rPower);
        motorL.setPower(-lPower);

        return (ticks>0?motorR.getCurrentPosition()-encoderTickInitialR>=ticks:motorR.getCurrentPosition()-encoderTickInitialR<=ticks);
    }

    private boolean turnDegrees(double power, int degrees){
        double diff=heading-headingInitial-degrees;
        //distance (in degrees) remaining in the turn

        if(Math.abs(diff)>angleTolerance){
            goodCounter=0;
            motorR.setPower((diff>0?1:-1)*power);

            if(Math.abs(diff)>8)
                motorL.setPower((diff>0?1:-1)*power);
            else motorL.setPower(0);
        }else{
            motorL.setPower(0);
            motorR.setPower(0);
            goodCounter++;
        }

        if(goodCounter>=angleTimeTolerance) {
            Log.i("hhs4227",String.format("heading: %.1f, revHeading: %.1f",heading,angles.firstAngle));
            Log.i("hhs4227",String.format("headingInitial %.1f goodCounter %d, angleTimeTolerance: %d",headingInitial, goodCounter, angleTimeTolerance));
            Log.i("hhs4227",String.format("diff %.1f",diff));

            return true;
        }
        else {
            return false;
        }



    }

    private long lastTime=System.currentTimeMillis();

    private boolean smartTurn(double power, int degrees){
        double diff=-heading+headingInitial+degrees;
        //difference (in degrees) remaining in the turn
        double totalDiff=degrees;
        //total delta theta for the duration of the turn





        double pow=power*diff/totalDiff;
        pow = Range.clip(pow,-power,power);
        if(System.currentTimeMillis()-lastTime>100) {
            Log.i("hhs4227", String.format(""+state+": Diff: %f,  Total: %f, pow: %f", diff, totalDiff, pow));
            lastTime=System.currentTimeMillis();
        }

        motorL.setPower((degrees>0?-1:1)*pow);
        motorR.setPower((degrees>0?-1:1)*pow);



        if(Math.abs(diff)<angleTolerance){
            return true;
        }


        return false;
    }

    private boolean absoluteTurnDeg (double power, int degrees) {
        headingInitial = 0;
        return turnDegrees(power, degrees);
    }
    private boolean smartAbsoluteTurn (double power, int degrees) {
        headingInitial = 0;
        return smartTurn(power, degrees);
    }
    double timeAtStart=System.currentTimeMillis()/1000;
    double timeAtState=timeAtStart;

    private void next() {
        Log.i("hhs4227","Finished state" +state);
        state=state.getNext();
        encoderTickInitialR=motorR.getCurrentPosition();
        encoderTickInitialL=motorR.getCurrentPosition();
        headingInitial=heading;
        pitchInitial=angles.secondAngle;
        motorDrive(0);
        goodCounter=0;
        initialRange=(int)sideRange1.getDistance(DistanceUnit.CM);
        Log.i("hhs4227",String.format("heading %.1f",heading)+"New State: "+state);

        timeAtStart=System.currentTimeMillis()/1000;
    }

    private void driveTime(double power, long time) throws InterruptedException {
        motorL.setPower(power);
        motorR.setPower(power);
        Thread.sleep(time);
        motorL.setPower(0);
        motorR.setPower(0);

    }


    int skatingTolerance=2;
    double skatingDiff=.35;
    private void skate(double power){
        int d1=(int)sideRange1.getDistance(DistanceUnit.CM);
        int d2=(int)sideRange2.getDistance(DistanceUnit.CM);

        //backward is r- l+
        //if port 0 is larger then r becomes more neg else vice

        if(power>0){
            double r=-power;
            double l=power;
            if(d1-d2-skatingTolerance>0)
                r-=skatingDiff;
            if(d2-d1-skatingTolerance>0)
                r+=skatingDiff;

            motorR.setPower(r);
            motorL.setPower(l);


        }else{
            double r=-power;
            double l=power;
            if(d1-d2-skatingTolerance>0)
                r+=skatingDiff;
            if(d2-d1-skatingTolerance>0)
                r-=skatingDiff;

            motorR.setPower(r);
            motorL.setPower(l);
        }




    }

    private boolean skateDist(double power, int dist){
        skate(power);
        if(dist>0){
            return motorR.getCurrentPosition()-encoderTickInitialR>dist-encoderTickInitialR;
        }
        return motorR.getCurrentPosition()-encoderTickInitialR<dist-encoderTickInitialR;
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



        boolean dDown=false;
        boolean dUp=false;
        boolean aDown=false;
        boolean bDown=false;
        boolean xDown=false;

        boolean isReady=false;

        composeTelemetry2();



        waitForStart();

        // TODO: IBARGUEN - Is there a reason we can't do the next 5 lines before the waitForStart()?
        telemetry.clearAll();
        composeTelemetry();
        marker.setPosition(0);

        angles   = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
        gravity  = imu.getGravity();
        timeAtStart=System.currentTimeMillis()/1000;

        long lastTimeLogged=System.currentTimeMillis();

        while(!isStopRequested()){
            updateHeading();
            telemetry.update();

            switch(state){
                case TURNONE:
                    if(smartTurn(.35, 90)){
                        next();
                    }
                    break;
                case TURNTWO:
                    if(smartTurn(.35, -60))
                        next();
                    break;
                case TURNTHREE:
                    if(smartTurn(.35, 300)){
                        //next();
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
        double deltaAngle = angles.firstAngle - lastAngles.firstAngle;

        // This will make sure that the deltaAngle is never more than 180
        // degrees in either direction.  What is the purpose of this?
        // TODO: This won't work if the deltaAngle is really big
        if (deltaAngle < -180)
            deltaAngle += 360;
        else if (deltaAngle > 180)
            deltaAngle -= 360;

        heading += deltaAngle;

        lastAngles = angles;

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
                        return motorL.getPower()+"/"+motorR.getPower();
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
                        return formatAngle(angles.angleUnit, angles.secondAngle);
                    }
                })
                .addData("pitch", new Func<String>() {
                    @Override public String value() {
                        return formatAngle(angles.angleUnit, angles.thirdAngle);
                    }
                });

        telemetry.addLine()
                .addData("State: ", new Func<String>() {
                    @Override public String value() {

                        return state+"";

                    }
                })
                .addData("mag", new Func<String>() {
                    @Override public String value() {
                        return String.format(Locale.getDefault(), "%.3f",
                                Math.sqrt(gravity.xAccel*gravity.xAccel
                                        + gravity.yAccel*gravity.yAccel
                                        + gravity.zAccel*gravity.zAccel));
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
