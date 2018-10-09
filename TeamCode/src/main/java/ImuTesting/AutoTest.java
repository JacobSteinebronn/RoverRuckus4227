package ImuTesting;
import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.hardware.bosch.JustLoggingAccelerationIntegrator;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.Servo;

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

/**
 * Created by hhs-robotics on 8/1/2018.
 */

@Autonomous(name = "autoTest", group = "Auto")
public class AutoTest extends LinearOpMode {
    public DcMotor motorL;
    public DcMotor motorR;
    public Servo lServo;
    public Servo rServo;
    //public DistanceSensor sensor1;
    public DistanceSensor sensor2;

    BNO055IMU imu;

    Orientation angles;
    Acceleration gravity;

    enum InitState {
        HANGING,
        STARTPOS,
        DELAY,
        CRATER,
        READY;


        public InitState getNext(){
            return this.ordinal()<InitState.values().length-1?InitState.values()[this.ordinal()+1]:InitState.READY;
        }
    }
    InitState initState=InitState.HANGING;


    enum State{
        DETACH,
        MOVEBIT,
        FACEWALL,
        DELAYTIME,
        GOTOWALL,
        FACEDEPOT,
        GOTOWALL2,
        DEPOSIT,
        FACECRATER,
        DRIVECRATER,
        STOP;

        public State getNext(){
            return this.ordinal()<State.values().length-1?State.values()[this.ordinal()+1]:State.DETACH;
        }

    }
    State state=State.DETACH;

    int encoderTickInitial=0;
    double headingInitial=0;

    void motorDrive(double power){
        motorL.setPower(-power);
        motorR.setPower(power);

    }
    private boolean driveTicks(double power, int ticks){
        motorDrive((ticks>0?1:-1)*power);
        return (ticks>0?motorR.getCurrentPosition()-encoderTickInitial>=ticks:motorR.getCurrentPosition()-encoderTickInitial<=ticks);
    }
    private boolean turnDegrees(double power, int degrees){
        motorR.setPower((degrees>0?-1:1)*power);
        motorL.setPower((degrees>0?-1:1)*power);
        return (degrees>0?(double)angles.firstAngle-headingInitial>=degrees:(double)angles.firstAngle-headingInitial<=degrees);


    }
    private void next() {
        state=state.getNext();
        encoderTickInitial=motorR.getCurrentPosition();
        headingInitial=(double)angles.firstAngle;
        motorDrive(0);
    }

    private void driveTime(double power, long time) throws InterruptedException {
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

    boolean startHanging=false;
    boolean startNearCrater=true;
    boolean redCrater=true;
    int timeDelay=3;


    @Override
    public void runOpMode() throws InterruptedException {
        motorL = hardwareMap.get(DcMotor.class, "leftMotor");
        motorR = hardwareMap.get(DcMotor.class, "rightMotor");
        lServo=hardwareMap.get(Servo.class,"lServo");
        rServo=hardwareMap.get(Servo.class,"rServo");



        lServo.setDirection(Servo.Direction.REVERSE);
        rServo.setDirection(Servo.Direction.FORWARD);

        //sensor1=hardwareMap.get(DistanceSensor.class, "s1");
        sensor2=hardwareMap.get(DistanceSensor.class, "s2");


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


        boolean selection=false;
        boolean dDown=false;
        boolean dUp=false;
        boolean aDown=false;

        boolean isReady=false;
        /*
        while(!isReady){

            if(gamepad1.dpad_down){dDown=true;}
            if(gamepad1.dpad_up){dUp=true;}
            if(gamepad1.a){aDown=true;}

            switch(initState){
                case HANGING:
                    telemetry.clearAll();
                    telemetry.addData("Start Hanging?","");
                    telemetry.addData("Yes",selection?"<":" ");
                    telemetry.addData("No",!selection?"<":" ");
                    if(!gamepad1.dpad_down&&dDown){
                        selection=!selection;
                        dDown=false;
                    }
                    if(!gamepad1.dpad_up&&dUp){
                        dUp=false;
                        selection=!selection;
                    }
                    if(!gamepad1.a&&aDown){
                        aDown=false;
                        initState=initState.getNext();
                        startHanging=selection;
                    }

                    break;
                case DELAY:
                    telemetry.clearAll();
                    telemetry.addData("Set Delay(Seconds)","");
                    telemetry.addData("Delay time:",timeDelay);
                    if(!gamepad1.dpad_down&&dDown){
                        timeDelay--;
                        dDown=false;
                    }
                    if(!gamepad1.dpad_up&&dUp){
                        dUp=false;
                        timeDelay++;
                    }
                    if(!gamepad1.a&&aDown){
                        aDown=false;
                        initState=initState.getNext();
                    }

                    break;
                case STARTPOS:
                    telemetry.clearAll();
                    telemetry.addData("Start facing crater?","");
                    telemetry.addData("Yes",selection?"<":" ");
                    telemetry.addData("No",!selection?"<":" ");
                    if(!gamepad1.dpad_down&&dDown){
                        selection=!selection;
                        dDown=false;
                    }
                    if(!gamepad1.dpad_up&&dUp){
                        dUp=false;
                        selection=!selection;
                    }
                    if(!gamepad1.a&&aDown){
                        aDown=false;
                        initState=initState.getNext();
                        startNearCrater=selection;
                    }
                    break;
                case CRATER:
                    telemetry.clearAll();
                    telemetry.addData("Aim for own crater?","");
                    telemetry.addData("Yes",selection?"<":" ");
                    telemetry.addData("No",!selection?"<":" ");
                    if(!gamepad1.dpad_down&&dDown){
                        selection=!selection;
                        dDown=false;
                    }
                    if(!gamepad1.dpad_up&&dUp){
                        dUp=false;
                        selection=!selection;
                    }
                    if(!gamepad1.a&&aDown){
                        aDown=false;
                        initState=initState.getNext();
                        redCrater=selection;
                    }
                    break;
                case READY:
                default:
                    isReady=true;
                    break;


            }
        }
        */
        waitForStart();

        composeTelemetry();

        while(!isStopRequested()){
            telemetry.update();




            switch(state){
                case DETACH:
                    if(startHanging){

                    }else {
                        next();
                    }
                    break;
                case MOVEBIT:
                    if(driveTicks(.2,-1500)){
                        next();

                    }
                    break;
                case FACEWALL:
                    if(startNearCrater){
                        if(turnDegrees(.2,45)){
                            next();
                        }
                    }else{
                        if(turnDegrees(.2,-45)){

                        }
                    }
                    break;
                case DELAYTIME:
                    Thread.sleep(timeDelay*1000);
                    next();
                    break;
                case GOTOWALL:
                    motorDrive(-.2);
                    if(sensor2.getDistance(DistanceUnit.CM)<20){
                        next();
                    }
                    break;
                case FACEDEPOT:
                    if(turnDegrees(.2, 90)){
                        next();
                    }
                    break;
                case GOTOWALL2:
                    motorDrive(-.2);
                    if(sensor2.getDistance(DistanceUnit.CM)<20){
                        next();
                    }
                    break;
                case DEPOSIT:
                    if(driveTicks(.3,500)){
                        next();
                    }
                    break;
                case FACECRATER:
                    if(redCrater&&turnDegrees(.2,180)){
                        next();
                    }else{
                        if(turnDegrees(.2,90)){
                            next();
                        }
                    }
                    break;
                case DRIVECRATER:
                    if(driveTicks(.2, -10000)){
                        next();
                    }
                    break;
                case STOP:
                default:
                    requestOpModeStop();

            }

        }





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
                        return motorL.getPower()+"/"+motorR.getPower();
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
    }
    String formatAngle(AngleUnit angleUnit, double angle) {
        return formatDegrees(AngleUnit.DEGREES.fromUnit(angleUnit, angle));
    }

    String formatDegrees(double degrees){
        return String.format(Locale.getDefault(), "%.1f", AngleUnit.DEGREES.normalize(degrees));
    }
}
