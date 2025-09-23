package org.firstinspires.ftc.teamcode.utils;

public class PatternIdentifier {

    // Private constructor to prevent instantiation of this utility class
    private PatternIdentifier() {}

    // Pattern IDs
    public static final int ID_GREEN_PURPLE_PURPLE = 21;
    public static final int ID_PURPLE_GREEN_PURPLE = 22;
    public static final int ID_PURPLE_PURPLE_GREEN = 23;

    // Pattern Descriptions
    public static final String DESC_GREEN_PURPLE_PURPLE = "green purple purple";
    public static final String DESC_PURPLE_GREEN_PURPLE = "purple green purple";
    public static final String DESC_PURPLE_PURPLE_GREEN = "purple purple green";

    /**
     * Gets the description of a color pattern based on its ID.
     * @param id The ID of the pattern (e.g., 21, 22, 23).
     * @return The string description of the pattern, or "Unknown Pattern ID" if not found.
     */
    public static String getPatternDescription(int id) {
        switch (id) {
            case ID_GREEN_PURPLE_PURPLE:
                return DESC_GREEN_PURPLE_PURPLE;
            case ID_PURPLE_GREEN_PURPLE:
                return DESC_PURPLE_GREEN_PURPLE;
            case ID_PURPLE_PURPLE_GREEN:
                return DESC_PURPLE_PURPLE_GREEN;
            default:
                return "Unknown Pattern ID";
        }
    }
}
