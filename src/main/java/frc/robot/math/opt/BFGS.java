package frc.robot.math.opt;

import org.ejml.data.DMatrixRMaj;
import org.ejml.dense.row.CommonOps_DDRM;
import org.ejml.dense.row.NormOps_DDRM;
import edu.wpi.first.math.MathUtil;

/** Broyden-Fletcher-Goldfarb-Shanno optimizer */
public class BFGS {

    private int itrNum;
    private int funcNum;
    private double fMin;
    private double[] xOpt;
    private double[] gradNorm;

    /** Find the minimum of a function */
    public void optimize(DiffFunc func, double[] x0, double tol, int maxItr) {
        // https://github.com/ceze/tspl/blob/e67bcb8667b2de6692d6afcb39986b772637dd65/include/bfgs-impl.h#L54
        int k = 0;
        int cnt = 0;
        int _N = x0.length;

        double ys;
        double yHy;
        double alpha;
        DMatrixRMaj d = new DMatrixRMaj(new double[_N]);
        DMatrixRMaj s = new DMatrixRMaj(new double[_N]);
        DMatrixRMaj s_Ys = new DMatrixRMaj(new double[_N]);
        DMatrixRMaj sOuter = CommonOps_DDRM.identity(_N);
        DMatrixRMaj y = new DMatrixRMaj(new double[_N]);
        DMatrixRMaj v = new DMatrixRMaj(new double[_N]);
        DMatrixRMaj vOuter = CommonOps_DDRM.identity(_N);
        DMatrixRMaj _Hy = new DMatrixRMaj(new double[_N]);
        DMatrixRMaj _Hy_yHy = new DMatrixRMaj(new double[_N]);
        DMatrixRMaj _HyOuter = CommonOps_DDRM.identity(_N);
        DMatrixRMaj gPrev = new DMatrixRMaj(new double[_N]);
        DMatrixRMaj _H = CommonOps_DDRM.identity(_N);
        DMatrixRMaj temp1 = CommonOps_DDRM.identity(_N);
        DMatrixRMaj temp2 = CommonOps_DDRM.identity(_N);

        DMatrixRMaj x = new DMatrixRMaj(new double[_N]);
        for (int i = 0; i < x0.length; i++) {
            x.set(i, x0[i]);
        }
        double fx = func.evaluate(x.getData());
        this.funcNum++;
        double[] gnorm = new double[maxItr];
        DMatrixRMaj g = new DMatrixRMaj(func.gradient(x.getData()));
        gnorm[k++] = NormOps_DDRM.fastNormF(g);

        while ((gnorm[k - 1] > tol) && (k < maxItr)) {
            CommonOps_DDRM.mult(-1.0, _H, g, d);
            alpha = this.getStep(func, x, d, 10);
            if (!this.success) {
                if (NormOps_DDRM.fastNormF(
                    CommonOps_DDRM.subtract(_H, CommonOps_DDRM.identity(_N), null)) < 1e-6) {
                    break;
                } else {
                    _H = CommonOps_DDRM.identity(_N);
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

                CommonOps_DDRM.mult(_H, y, _Hy);
                ys = CommonOps_DDRM.dot(y, s);
                yHy = CommonOps_DDRM.dot(y, _Hy);
                if ((ys < 1e-6) || (yHy < 1e-6)) {
                    _H = CommonOps_DDRM.identity(_N);
                } else {
                    CommonOps_DDRM.scale(1.0 / ys, s, s_Ys);
                    CommonOps_DDRM.scale(1.0 / yHy, _Hy, _Hy_yHy);
                    CommonOps_DDRM.subtract(s_Ys, _Hy_yHy, v);
                    CommonOps_DDRM.scale(Math.sqrt(yHy), v);
                    CommonOps_DDRM.multOuter(s, sOuter);
                    CommonOps_DDRM.multOuter(_Hy, _HyOuter);
                    CommonOps_DDRM.multOuter(v, vOuter);
                    CommonOps_DDRM.add(1.0 / ys, sOuter, _H, temp1);
                    CommonOps_DDRM.add(-1.0 / yHy, _HyOuter, vOuter, temp2);
                    CommonOps_DDRM.add(temp1, temp2, _H);
                }
                gnorm[k++] = NormOps_DDRM.fastNormF(g);
            }
        }

        xOpt = x.getData();
        fMin = fx;
        gradNorm = new double[k];
        for (int i = 0; i < k; i++) {
            gradNorm[i] = gnorm[i];
        }
        itrNum = k;
        if (gradNorm[k - 1] > tol) {
            this.success = false;
        }
    }

    /** Find the minimum of a function */
    public void optimize(DiffFunc func, double[] x0, double tol) {
        optimize(func, x0, tol, 10000);
    }

    /** Find the minimum of a function */
    public void optimize(DiffFunc func, double[] x0) {
        optimize(func, x0, 1e-6);
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

    public int getNumFunctionCalls() {
        return funcNum;
    }

    public boolean isSuccess() {
        return success;
    }

    @Override
    public String toString() {
        StringBuilder sb = new StringBuilder();
        sb.append("BFGS{");
        sb.append("ItrNum=");
        sb.append(getItrNum());
        sb.append(", FuncMin=");
        sb.append(getFuncMin());
        sb.append(", OptValue=[");
        var opt = getOptValue();
        for (int i = 0; i < opt.length; i++) {
            if (i != 0) {
                sb.append(", ");
            }
            sb.append(opt[i]);
        }
        sb.append("], Success=");
        sb.append(success);
        sb.append("}");
        return sb.toString();
    }

}
