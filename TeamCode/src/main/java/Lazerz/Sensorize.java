package Lazerz;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

@Disabled
@TeleOp(name = "Senzorize", group = "ZZZ_Template")
public class Sensorize extends OpMode {
    DistanceSensor sensor;

    @Override
    public void init() {

        sensor=hardwareMap.get(DistanceSensor.class,"sensor");
    }

    @Override
    public void loop() {
        telemetry.addData("Distance:", sensor.getDistance(DistanceUnit.METER));

    }
}

