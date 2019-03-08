package net.maxdev.ftc.utils;

import com.disnodeteam.dogecv.CameraViewDisplay;
import com.disnodeteam.dogecv.DogeCV;
import com.disnodeteam.dogecv.detectors.roverrukus.GoldAlignDetector;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;

public class DogeGoldVision {
    private WebcamName webcam = null;
    private GoldAlignDetector detector = null;
    private Telemetry telemetry = null;

    public void init(HardwareMap hardwareMap, Telemetry oldTelemetry) {
        telemetry = oldTelemetry;

        webcam = hardwareMap.get(WebcamName.class, "webcam");
        detector = new GoldAlignDetector();

        detector.VUFORIA_KEY = "ARIiu/3/////AAABmZIfnS41RUHHsnurKd3hu4VD39t5XPgtNFq+PTs97uQPNMxYQkJ4Z3Mu+5u41vSn/YLXkYqK4ig9noTKAxBURUfgdm5+Mg8R/SoIWOMFuUXIwkl4wkY+OFJFUxMH9TN4hM4exr6R4Kv5Ear8SWkDrfyQXE3t8z4qLKvwQIdlcgAN4bbUE425NbS2oGhjt2LUTyRpiaCrsbCBIWHFx8sS2NugaEyAIGA2PSJ86cp44moIhCpO8eWCqD2pMuA5bnfoEX8xYrFxHybyddHHeo4daXzksXSgAnuHQpLObDI8dd61q1i2YTrTyUYAxJKjZ0TcT+CokJu5+lWENm0AJuTcFODxZ5x/HBNOgcNIObFp4VU9";
        detector.init(hardwareMap.appContext, CameraViewDisplay.getInstance(), DogeCV.CameraMode.WEBCAM, false, webcam);

        detector.alignSize = 100;
        detector.alignPosOffset = 0;
        detector.downscale = 0.4;

        detector.areaScoringMethod = DogeCV.AreaScoringMethod.MAX_AREA;
        detector.maxAreaScorer.weight = 0.005; //

        detector.ratioScorer.weight = 5; //
        detector.ratioScorer.perfectRatio = 1.0;
    }

    public void enable(boolean enabled) {
        if (enabled) detector.enable();
        else detector.disable();
    }

    public void debug() {
        telemetry.addLine()
            .addData("IsAligned" , detector.getAligned())
            .addData("X Pos" , detector.getXPosition());
    }
}
