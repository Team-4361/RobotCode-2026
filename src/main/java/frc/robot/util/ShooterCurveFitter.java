package frc.robot.util;

import java.util.List;

/**
 * Fits a quadratic curve (RPM = a·d² + b·d + c) to calibration data
 * collected by ShooterCalibrationCommand, then formats it as a ready-to-paste
 * Java method body.
 *
 * The output string is printed to SmartDashboard so you can copy it directly
 * into ShooterCalculator.getRPM().
 *
 * Least-squares quadratic fit via the normal equations (no external libs needed).
 */
public class ShooterCurveFitter {

    private ShooterCurveFitter() {}



    
    /**
     * Fits the data and returns a formatted string containing:
     *  - The coefficients a, b, c
     *  - A copy-pasteable Java method
     *  - Per-point residuals so you can see how accurate the fit is
     *
     * @param distances List of distances (metres)
     * @param rpms      Matching list of scored RPM values
     * @return Human-readable fit summary for SmartDashboard / console
     */
    public static String fitAndFormat(List<Double> distances, List<Double> rpms) {
        int n = distances.size();
        if (n < 3) {
            // Fall back to linear if fewer than 3 points
            return fitLinearAndFormat(distances, rpms);
        }

        double[] coeffs = fitQuadratic(distances, rpms);
        double a = coeffs[0], b = coeffs[1], c = coeffs[2];

        StringBuilder sb = new StringBuilder();
        sb.append(String.format("Quadratic fit: RPM = %.4f*d^2 + %.4f*d + %.4f%n", a, b, c));
        sb.append("--- Residuals ---\n");

        double maxErr = 0;
        for (int i = 0; i < n; i++) {
            double d       = distances.get(i);
            double actual  = rpms.get(i);
            double predict = a * d * d + b * d + c;
            double err     = actual - predict;
            maxErr = Math.max(maxErr, Math.abs(err));
            sb.append(String.format("  d=%.2f m  actual=%.0f  predicted=%.0f  err=%.0f%n",
                    d, actual, predict, err));
        }
        sb.append(String.format("Max error: %.0f RPM%n", maxErr));

        // Ready-to-paste Java code
        sb.append("\n--- Paste into ShooterCalculator.getRPM(double distanceMeters) ---\n");
        sb.append(String.format(
                "double rpm = %.6f * distanceMeters * distanceMeters%n" +
                "           + %.6f * distanceMeters%n" +
                "           + %.6f;%n" +
                "return RPM.of(Math.max(0, rpm));%n",
                a, b, c));

        return sb.toString();
    }

    // ── Least-squares quadratic fit ───────────────────────────────────────────

    /**
     * Solves the 3×3 normal equations for y = a*x^2 + b*x + c.
     * Returns [a, b, c].
     */
    private static double[] fitQuadratic(List<Double> xs, List<Double> ys) {
        int n = xs.size();

        // Build sums needed for the normal equations
        double s0 = n, s1 = 0, s2 = 0, s3 = 0, s4 = 0;
        double t0 = 0, t1 = 0, t2 = 0;

        for (int i = 0; i < n; i++) {
            double x = xs.get(i), y = ys.get(i);
            double x2 = x * x, x3 = x2 * x, x4 = x2 * x2;
            s1 += x;  s2 += x2;  s3 += x3;  s4 += x4;
            t0 += y;  t1 += x * y;  t2 += x2 * y;
        }

        // 3×3 system:
        // | s4  s3  s2 | | a |   | t2 |
        // | s3  s2  s1 | | b | = | t1 |
        // | s2  s1  s0 | | c |   | t0 |
        double[][] A = {
            {s4, s3, s2},
            {s3, s2, s1},
            {s2, s1, s0}
        };
        double[] B = {t2, t1, t0};

        return solve3x3(A, B);
    }

    // ── Linear fallback (< 3 points) ──────────────────────────────────────────

    private static String fitLinearAndFormat(List<Double> distances, List<Double> rpms) {
        int n = distances.size();
        double sumX = 0, sumY = 0, sumXY = 0, sumX2 = 0;
        for (int i = 0; i < n; i++) {
            sumX  += distances.get(i);
            sumY  += rpms.get(i);
            sumXY += distances.get(i) * rpms.get(i);
            sumX2 += distances.get(i) * distances.get(i);
        }
        double denom = n * sumX2 - sumX * sumX;
        double m     = (n * sumXY - sumX * sumY) / denom;
        double b     = (sumY - m * sumX) / n;

        return String.format(
                "Linear fit (< 3 points): RPM = %.4f * d + %.4f%n%n" +
                "--- Paste into ShooterCalculator ----%n" +
                "return RPM.of(Math.max(0, %.6f * distanceMeters + %.6f));%n",
                m, b, m, b);
    }

    // ── Gaussian elimination for 3×3 system ──────────────────────────────────

    private static double[] solve3x3(double[][] A, double[] B) {
        int n = 3;
        // Augmented matrix
        double[][] M = new double[n][n + 1];
        for (int i = 0; i < n; i++) {
            for (int j = 0; j < n; j++) M[i][j] = A[i][j];
            M[i][n] = B[i];
        }



        // Forward elimination
        for (int col = 0; col < n; col++) {
            // Pivot
            int maxRow = col;
            for (int row = col + 1; row < n; row++)
                if (Math.abs(M[row][col]) > Math.abs(M[maxRow][col])) maxRow = row;
            double[] tmp = M[col]; M[col] = M[maxRow]; M[maxRow] = tmp;

            for (int row = col + 1; row < n; row++) {
                double factor = M[row][col] / M[col][col];
                for (int j = col; j <= n; j++)
                    M[row][j] -= factor * M[col][j];
            }
        }

        // Back substitution
        double[] result = new double[n];
        for (int i = n - 1; i >= 0; i--) {
            result[i] = M[i][n];
            for (int j = i + 1; j < n; j++)
                result[i] -= M[i][j] * result[j];
            result[i] /= M[i][i];
        }
        return result;
    }
}

