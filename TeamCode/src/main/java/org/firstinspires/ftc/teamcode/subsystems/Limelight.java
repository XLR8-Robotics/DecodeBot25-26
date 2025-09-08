package org.firstinspires.ftc.teamcode.subsystems;

import org.firstinspires.ftc.teamcode.TuningConfig;
import org.json.JSONArray;
import org.json.JSONException;
import org.json.JSONObject;

import java.io.BufferedReader;
import java.io.IOException;
import java.io.InputStream;
import java.io.InputStreamReader;
import java.net.HttpURLConnection;
import java.net.URL;
import java.nio.charset.StandardCharsets;
import java.util.HashMap;
import java.util.Map;

/**
 * Limelight 3A subsystem (HTTP JSON polling) for AprilTag detections on FTC.
 *
 * Notes:
 * - This polls the Limelight JSON endpoint and parses minimal fields needed for FTC.
 * - You may need to set your Limelight to "apriltag" pipeline on the web UI.
 * - Uses TuningConfig for IP/host and optional tag-id-to-name mapping.
 */
public class Limelight {
    private final String baseUrl; // e.g. http://10.xx.yy.11:5807

    // Last results
    private boolean hasTargets;
    private int bestTagId = -1;
    private double tx;
    private double ty;
    private double[] botpose = new double[0];

    public Limelight() {
        this.baseUrl = "http://" + TuningConfig.LIMELIGHT_HOST + ":5807";
    }

    /** Polls Limelight for the latest results. Call periodically (e.g., each loop). */
    public void update() {
        try {
            JSONObject root = getJson(baseUrl + "/json");
            if (root == null) {
                hasTargets = false;
                bestTagId = -1;
                return;
            }

            JSONObject results = root.optJSONObject("Results");
            if (results == null) {
                hasTargets = false;
                bestTagId = -1;
                return;
            }

            // Overall target valid flag\best target fields
            hasTargets = results.optBoolean("Valid", false) || results.optInt("tv", 0) == 1;
            bestTagId = results.optInt("tid", -1);
            tx = results.optDouble("tx", 0.0);
            ty = results.optDouble("ty", 0.0);

            JSONArray botposeArray = results.optJSONArray("botpose");
            if (botposeArray != null) {
                botpose = new double[botposeArray.length()];
                for (int i = 0; i < botposeArray.length(); i++) {
                    botpose[i] = botposeArray.optDouble(i, Double.NaN);
                }
            }

            // If bestTagId was not provided, try the first fiducial result
            if (bestTagId < 0) {
                JSONArray fiducials = results.optJSONArray("FiducialResults");
                if (fiducials != null && fiducials.length() > 0) {
                    JSONObject f0 = fiducials.optJSONObject(0);
                    if (f0 != null) {
                        bestTagId = f0.optInt("fID", -1);
                        tx = f0.optDouble("tx", tx);
                        ty = f0.optDouble("ty", ty);
                    }
                }
            }
        } catch (Exception e) {
            hasTargets = false;
            bestTagId = -1;
        }
    }

    public boolean hasTargets() { return hasTargets; }
    public int getBestTagId() { return bestTagId; }
    public double getTx() { return tx; }
    public double getTy() { return ty; }
    public double[] getBotpose() { return botpose; }

    /** Returns a human-friendly name for the detected tag id based on TuningConfig mapping. */
    public String getBestTagName() {
        if (bestTagId < 0) return "None";
        String name = TuningConfig.APRILTAG_ID_TO_NAME.get(bestTagId);
        return name != null ? name : ("Tag " + bestTagId);
    }

    private JSONObject getJson(String url) throws IOException, JSONException {
        HttpURLConnection conn = (HttpURLConnection) new URL(url).openConnection();
        conn.setConnectTimeout(200);
        conn.setReadTimeout(200);
        conn.setUseCaches(false);
        conn.setRequestMethod("GET");
        conn.connect();
        int code = conn.getResponseCode();
        if (code != 200) return null;
        try (InputStream is = conn.getInputStream();
             InputStreamReader isr = new InputStreamReader(is, StandardCharsets.UTF_8);
             BufferedReader br = new BufferedReader(isr)) {
            StringBuilder sb = new StringBuilder();
            String line;
            while ((line = br.readLine()) != null) sb.append(line);
            return new JSONObject(sb.toString());
        } finally {
            conn.disconnect();
        }
    }
}


