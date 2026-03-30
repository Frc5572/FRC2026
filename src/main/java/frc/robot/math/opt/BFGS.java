package frc.robot.math.opt;

import org.ejml.data.DMatrixRMaj;
import org.ejml.dense.row.CommonOps_DDRM;
import org.ejml.dense.row.NormOps_DDRM;
import edu.wpi.first.math.MathUtil;

public class BFGS {

    private int itrNum;
    private int funcNum;
    private double fMin;
    private double[] xOpt;
    private double[] gradNorm;

    public void optimize(DiffFunc func, double[] x0, double tol, int maxItr) {
        // https://github.com/ceze/tspl/blob/e67bcb8667b2de6692d6afcb39986b772637dd65/include/bfgs-impl.h#L54
        int k = 0;
        int cnt = 0;
        int N = x0.length;

        double ys;
        double yHy;
        double alpha;
        DMatrixRMaj d = new DMatrixRMaj(new double[N]);
        DMatrixRMaj s = new DMatrixRMaj(new double[N]);
        DMatrixRMaj y = new DMatrixRMaj(new double[N]);
        DMatrixRMaj v = new DMatrixRMaj(new double[N]);
        DMatrixRMaj Hy = new DMatrixRMaj(new double[N]);
        DMatrixRMaj gPrev = new DMatrixRMaj(new double[N]);
        DMatrixRMaj H = CommonOps_DDRM.identity(N);

        DMatrixRMaj x = new DMatrixRMaj(new double[N]);
        for (int i = 0; i < x0.length; i++) {
            x.set(i, x0[i]);
        }
        double fx = func.evaluate(x.getData());
        this.funcNum++;
        double[] gnorm = new double[maxItr];
        DMatrixRMaj g = new DMatrixRMaj(func.gradient(x.getData()));
        gnorm[k++] = NormOps_DDRM.fastNormF(g);

        while ((gnorm[k - 1] > tol) && (k < maxItr)) {
            CommonOps_DDRM.mult(-1.0, H, g, d);
            alpha = this.getStep(func, x, d, 10);
            if (!this.success) {
                if (NormOps_DDRM.fastNormF(
                    CommonOps_DDRM.subtract(H, CommonOps_DDRM.identity(N), null)) < 1e-6) {
                    break;
                } else {
                    H = CommonOps_DDRM.identity(N);
                    cnt++;
                    if (cnt == maxItr) {
                        break;
                    }
                }
            } else {
                // update
                CommonOps_DDRM.scale(alpha, d, s);
                CommonOps_DDRM.addEquals(x, s);
                fx = func.evaluate(x.getData());
                this.funcNum++;
                gPrev = g;
                g = new DMatrixRMaj(func.gradient(x.getData()));
                CommonOps_DDRM.subtract(g, gPrev, y);

                // TODO:
                // https://github.com/ceze/tspl/blob/e67bcb8667b2de6692d6afcb39986b772637dd65/include/bfgs-impl.h#L110
            }
        }
    }

    private boolean success = false;

    private double getStep(DiffFunc func, DMatrixRMaj xk, DMatrixRMaj dk, int maxItr) {
        double mu = 0.001;
        double kUp = 0.5;
        double kLow = 0.1;
        double alpha = 1.0;
        double alphaMin;
        double alphaMax;

        double fk = func.evaluate(xk.getData());
        double fNew = fk + 0.1;

        DMatrixRMaj xNew = xk.createLike();
        DMatrixRMaj gk = new DMatrixRMaj(func.gradient(xk.getData()));

        double gd = CommonOps_DDRM.dot(gk, dk);
        for (int i = 0; i < maxItr; i++) {
            CommonOps_DDRM.add(alpha, dk, xk, xNew);
            fNew = func.evaluate(xNew.getData());
            funcNum++;

            if (fNew < fk + mu * alpha * gd) {
                success = true;
                return alpha;
            } else {
                alphaMin = kLow * alpha;
                alphaMax = kUp * alpha;

                alpha = -0.5 * alpha * alpha * gd / (fNew - fk - alpha * gd);

                alpha = MathUtil.clamp(alpha, alphaMin, alphaMax);
            }
        }

        if (fNew > fk) {
            success = false;
            return 0.0;
        } else {
            success = true;
            return alpha;
        }
    }

    public void optimize(DiffFunc func, double[] x0, double tol) {
        optimize(func, x0, tol, 100);
    }

    public void optimize(DiffFunc func, double[] x0) {
        optimize(func, x0, 1e-6);
    }

    public int getItrNum() {
        return itrNum;
    }

    public double getFuncMin() {
        return fMin;
    }

    public double[] getOptValue() {
        return xOpt;
    }

    public double[] getGradNorm() {
        return gradNorm;
    }

}
