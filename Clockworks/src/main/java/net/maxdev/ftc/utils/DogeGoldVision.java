package net.maxdev.ftc.utils;

import com.disnodeteam.dogecv.CameraViewDisplay;
import com.disnodeteam.dogecv.DogeCV;
import com.disnodeteam.dogecv.detectors.roverrukus.GoldAlignDetector;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;

public class DogeGoldVision {
    private WebcamName webcamName = null;
    private GoldAlignDetector detector = null;
    private Telemetry telemetry = null;

    public void init(HardwareMap hardwareMap, Telemetry oldTelemetry) {
        telemetry = oldTelemetry;

        webcamName = hardwareMap.get(WebcamName.class, "webcam");
        detector = new GoldAlignDetector(); // Create detector
        detector.VUFORIA_KEY = "ARIiu/3/////AAABmZIfnS41RUHHsnurKd3hu4VD39t5XPgtNFq+PTs97uQPNMxYQkJ4Z3Mu+5u41vSn/YLXkYqK4ig9noTKAxBURUfgdm5+Mg8R/SoIWOMFuUXIwkl4wkY+OFJFUxMH9TN4hM4exr6R4Kv5Ear8SWkDrfyQXE3t8z4qLKvwQIdlcgAN4bbUE425NbS2oGhjt2LUTyRpiaCrsbCBIWHFx8sS2NugaEyAIGA2PSJ86cp44moIhCpO8eWCqD2pMuA5bnfoEX8xYrFxHybyddHHeo4daXzksXSgAnuHQpLObDI8dd61q1i2YTrTyUYAxJKjZ0TcT+CokJu5+lWENm0AJuTcFODxZ5x/HBNOgcNIObFp4VU9";
        detector.init(hardwareMap.appContext, CameraViewDisplay.getInstance(), DogeCV.CameraMode.WEBCAM, false, webcamName); // Initialize it with the app context and camera
        detector.useDefaults(); // Set detector to use default settings

        // Optional tuning
        detector.alignSize = 100; // How wide (in pixels) is the range in which the gold object will be aligned. (Represented by green bars in the preview)
        detector.alignPosOffset = 0; // How far from center frame to offset this alignment zone.
        detector.downscale = 0.4; // How much to downscale the input frames

        detector.areaScoringMethod = DogeCV.AreaScoringMethod.MAX_AREA; // Can also be PERFECT_AREA
        //detector.perfectAreaScorer.perfectArea = 10000; // if using PERFECT_AREA scoring
        detector.maxAreaScorer.weight = 0.005; //

        detector.ratioScorer.weight = 5; //
        detector.ratioScorer.perfectRatio = 1.0;
    }

    public void enable(boolean enabled) {
        if (enabled) detector.enable();
        else detector.disable();
    }

    public double getLocation() { return detector.getYPosition(); }

    public void debug() {
        telemetry.addLine()
            .addData("Y Pos" , detector.getYPosition())
            .addData("X Pos" , detector.getXPosition());
    }
}
