package frc.gen;

import java.text.DecimalFormat;
import java.text.NumberFormat;
import java.util.ArrayList;
import java.util.Arrays;
import java.util.List;
import java.util.function.ToDoubleFunction;
import org.ejml.data.DMatrixRMaj;
import org.ejml.dense.row.CommonOps_DDRM;

public class OLS<T> {

    private final T[] data;
    private final List<OLSTerm<T>> terms = new ArrayList<>();

    public OLS(T[] data) {
        this.data = data;
    }

    public OLS<T> term(String name, ToDoubleFunction<T> func) {
        this.terms.add(new OLSTerm<T>(name, func));
        return this;
    }

    private static record OLSTerm<T>(String name, ToDoubleFunction<T> input) {
    }

    public static record OLSSolution<T>(String res, double[] terms, double[] residuals,
        OLS<T> ols) {
        @Override
        public final String toString() {
            StringBuilder sb = new StringBuilder("OLSSolution{");
            sb.append("(");
            sb.append(res);
            sb.append("), residuals=");
            sb.append(Arrays.toString(residuals));
            sb.append("}");
            return sb.toString();
        }

        public double evaluate(T t) {
            double res = 0.0;
            for (int i = 0; i < terms.length; i++) {
                res += ols.terms.get(i).input.applyAsDouble(t) * terms[i];
            }
            return res;
        }
    }

    public OLSSolution<T> solve(ToDoubleFunction<T> outputFunc) {
        int numData = data.length;
        int numTerms = terms.size();
        DMatrixRMaj a = new DMatrixRMaj(numData, numTerms);
        DMatrixRMaj b = new DMatrixRMaj(numData, 1);
        for (int i = 0; i < numData; i++) {
            for (int j = 0; j < numTerms; j++) {
                a.set(i, j, terms.get(j).input.applyAsDouble(data[i]));
            }
            b.set(i, outputFunc.applyAsDouble(data[i]));
        }
        DMatrixRMaj temp1 = new DMatrixRMaj(numData, numData);
        DMatrixRMaj temp2 = new DMatrixRMaj(numData, numData);
        DMatrixRMaj temp3 = new DMatrixRMaj(numData, 1);
        DMatrixRMaj residuals = new DMatrixRMaj(numData, 1);
        DMatrixRMaj res = new DMatrixRMaj(numTerms, 1);
        CommonOps_DDRM.multTransA(a, a, temp1);
        CommonOps_DDRM.invert(temp1, temp2);
        CommonOps_DDRM.multTransB(temp2, a, temp1);
        CommonOps_DDRM.mult(temp1, b, res);
        StringBuilder asString = new StringBuilder();
        var coeffs = res.getData();
        NumberFormat formatter = new DecimalFormat("#0.000");
        asString.append(formatter.format(coeffs[0]));
        asString.append(" * ");
        asString.append(terms.get(0).name);
        for (int i = 1; i < terms.size(); i++) {
            asString.append(" + ");
            asString.append(formatter.format(coeffs[i]));
            asString.append(" * ");
            asString.append(terms.get(i).name);
        }
        CommonOps_DDRM.mult(a, res, temp3);
        CommonOps_DDRM.subtract(b, temp3, residuals);
        return new OLSSolution<>(asString.toString(), coeffs, residuals.getData(), this);
    }

}
