package frc.gen;

import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.MetersPerSecond;
import static edu.wpi.first.units.Units.Radians;
import static edu.wpi.first.units.Units.RadiansPerSecond;
import static edu.wpi.first.units.Units.Seconds;
import java.util.List;
import java.util.Locale;
import java.util.Map;
import java.util.Optional;
import java.util.TreeMap;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.units.measure.LinearVelocity;

/**
 * Generates a single-file HTML visualization for a given distance: trajectory fan on the left,
 * angle-vs-speed region on the right, and an info box with optimal shot parameters.
 */
public class ShotVisualizer {

    private static final double TRAJ_DT = 0.025; // trajectory sample interval (s)
    private static final double MAX_T = 10.0;

    /**
     * @param targetDistance horizontal distance to the hoop center
     * @param hoopHeight vertical height of hoop center above the shooter
     * @param hoopRadius radius of the hoop opening
     * @param lipHeight required clearance above hoopHeight at the near rim
     * @param radialVelocity robot radial velocity (positive = moving away from target)
     * @param backspin ball backspin
     * @param validShots output of {@link HoopSolver#findValidShots}
     * @return complete single-file HTML page as a string
     */
    public static String generate(Distance targetDistance, Distance hoopHeight, Distance hoopRadius,
        Distance lipHeight, LinearVelocity radialVelocity, AngularVelocity backspin,
        List<HoopSolver.ShotParams> validShots) {

        double distM = targetDistance.in(Meters);
        double hoopHM = hoopHeight.in(Meters);
        double hoopRM = hoopRadius.in(Meters);
        double lipHM = lipHeight.in(Meters);
        double rvMs = radialVelocity.in(MetersPerSecond);
        double bsRadS = backspin.in(RadiansPerSecond);

        HoopSolver solver = new HoopSolver(hoopHeight, hoopRadius, lipHeight, backspin);
        Optional<HoopSolver.OptimalResult> optOpt =
            solver.findOptimal(validShots, targetDistance, radialVelocity);
        if (optOpt.isEmpty()) {
            return "<html><body style='background:#0a0e1a;color:#fff;font-family:monospace;padding:2em'>"
                + "<h2>No valid shots found</h2></body></html>";
        }
        HoopSolver.OptimalResult result = optOpt.get();
        HoopSolver.ShotParams opt = result.shot();
        double optAngle = opt.exitAngle().in(Radians);
        double optV = opt.exitVelocity().in(MetersPerSecond);

        // Ellipse semi-axes from the optimal result
        double spdMinus = result.speedSemiAxis().in(MetersPerSecond);
        double spdPlus = spdMinus;
        double angMinus = Math.toDegrees(result.angleSemiAxis().in(Radians));
        double angPlus = angMinus;
        double tof = opt.tof().in(Seconds);

        TreeMap<Double, double[]> byAngle = groupByAngle(validShots);

        // trajectory JSON: one "min" and one "max" entry per angle
        StringBuilder traj = new StringBuilder("[");
        boolean first = true;
        for (Map.Entry<Double, double[]> e : byAngle.entrySet()) {
            double a = e.getKey(), vLo = e.getValue()[0], vHi = e.getValue()[1];
            if (!first)
                traj.append(",");
            double xMax = distM + hoopRM + 0.5;
            traj.append("{\"t\":\"min\",\"pts\":").append(trajJson(vLo, a, rvMs, bsRadS, xMax))
                .append("}");
            traj.append(",{\"t\":\"max\",\"pts\":").append(trajJson(vHi, a, rvMs, bsRadS, xMax))
                .append("}");
            first = false;
        }
        traj.append("]");

        // valid-region JSON: one entry per angle, lo/hi speeds
        StringBuilder vr = new StringBuilder("[");
        first = true;
        for (Map.Entry<Double, double[]> e : byAngle.entrySet()) {
            if (!first)
                vr.append(",");
            vr.append(String.format(Locale.US, "{\"a\":%.3f,\"lo\":%.4f,\"hi\":%.4f}",
                Math.toDegrees(e.getKey()), e.getValue()[0], e.getValue()[1]));
            first = false;
        }
        vr.append("]");

        return html(distM, hoopHM, hoopRM, lipHM, traj.toString(),
            trajJson(optV, optAngle, rvMs, bsRadS, distM + hoopRM + 0.5), vr.toString(),
            Math.toDegrees(optAngle), optV, tof, spdMinus, spdPlus, angMinus, angPlus);
    }

    private static TreeMap<Double, double[]> groupByAngle(List<HoopSolver.ShotParams> shots) {
        TreeMap<Double, double[]> map = new TreeMap<>();
        for (HoopSolver.ShotParams sp : shots) {
            double a = sp.exitAngle().in(Radians);
            double v = sp.exitVelocity().in(MetersPerSecond);
            double[] r = map.computeIfAbsent(a, k -> new double[] {v, v});
            r[0] = Math.min(r[0], v);
            r[1] = Math.max(r[1], v);
        }
        return map;
    }

    private static String trajJson(double vMs, double aRad, double rvMs, double bsRadS,
        double maxX) {
        SimulatedShot s = new SimulatedShot(MetersPerSecond.of(vMs), Radians.of(aRad),
            MetersPerSecond.of(rvMs), RadiansPerSecond.of(bsRadS));
        StringBuilder sb = new StringBuilder("[");
        sb.append(String.format(Locale.US, "[%.3f,%.3f]", s.getX(), s.getY()));
        for (double t = TRAJ_DT; t < MAX_T; t += TRAJ_DT) {
            s.step(TRAJ_DT);
            double x = s.getX(), y = s.getY();
            sb.append(String.format(Locale.US, ",[%.3f,%.3f]", x, y));
            if (x > maxX || y < -1.5)
                break;
        }
        return sb.append("]").toString();
    }

    private static String f(double v) {
        return Double.isNaN(v) ? "NaN" : String.format(Locale.US, "%.6g", v);
    }

    private static String html(double distM, double hoopH, double hoopR, double lipH, String trajs,
        String optTraj, String vr, double optDeg, double optV, double tof, double spdM, double spdP,
        double angM, double angP) {

        String data = String.format(Locale.US,
            "const HOOP={dist:%s,height:%s,radius:%s,lip:%s};\n" + "const TRAJS=%s;\n"
                + "const OPT_TRAJ=%s;\n" + "const VR=%s;\n"
                + "const OPT={angle:%s,speed:%s,tof:%s,spdM:%s,spdP:%s,angM:%s,angP:%s};\n",
            f(distM), f(hoopH), f(hoopR), f(lipH), trajs, optTraj, vr, f(optDeg), f(optV), f(tof),
            f(spdM), f(spdP), f(angM), f(angP));

        return "<!DOCTYPE html><html><head><meta charset='UTF-8'><title>Shot Analysis</title>"
            + "<style>" + "*{box-sizing:border-box;margin:0;padding:0}"
            + "body{background:#0a0e1a;display:flex;height:100vh;overflow:hidden;font-family:monospace}"
            + "#left{flex:3;position:relative}"
            + "#left canvas{position:absolute;inset:0;width:100%;height:100%}"
            + "#right{flex:2;display:flex;flex-direction:column;border-left:1px solid #1a2540}"
            + "#rtop{flex:1;min-height:0;position:relative}"
            + "#rtop canvas{position:absolute;inset:0;width:100%;height:100%}"
            + "#rtitle{position:absolute;top:10px;right:14px;color:#aabbcc;font-size:11px;letter-spacing:.5px;z-index:1}"
            + "#info{background:#0f1928;margin:10px;border-radius:8px;padding:12px 16px;font-size:12px;line-height:2}"
            + ".ol{color:#f5c542}.sl{color:#f5a040}.al{color:#7b8fff}" + "</style></head><body>"
            + "<div id='left'><canvas id='tc'></canvas></div>" + "<div id='right'>"
            + "<div id='rtop'><div id='rtitle'>Valid Shot Region (Angle vs Speed)</div><canvas id='rc'></canvas></div>"
            + "<div id='info'></div>" + "</div>" + "<script>\n" + data + JS
            + "</script></body></html>";
    }

    // Stored separately so Java string escaping doesn't interfere with the rendering logic.
    private static final String JS =
        """
            function xform(x0,x1,y0,y1,pl,pr,pt,pb,cw,ch){
              const dw=cw-pl-pr,dh=ch-pt-pb;
              return{wx:x=>pl+(x-x0)/(x1-x0)*dw,wy:y=>pt+dh-(y-y0)/(y1-y0)*dh,
                     x0,x1,y0,y1,dw,dh,pl,pt,pb};
            }
            function grid(ctx,xf,dx,dy){
              ctx.save();ctx.strokeStyle='rgba(255,255,255,0.07)';ctx.lineWidth=1;
              for(let x=Math.ceil(xf.x0/dx)*dx;x<=xf.x1+1e-9;x+=dx){
                const px=xf.wx(x);ctx.beginPath();ctx.moveTo(px,xf.pt);ctx.lineTo(px,xf.pt+xf.dh);ctx.stroke();}
              for(let y=Math.ceil(xf.y0/dy)*dy;y<=xf.y1+1e-9;y+=dy){
                const py=xf.wy(y);ctx.beginPath();ctx.moveTo(xf.pl,py);ctx.lineTo(xf.pl+xf.dw,py);ctx.stroke();}
              ctx.restore();
            }
            function labels(ctx,xf,dx,dy,xl,yl){
              ctx.save();ctx.fillStyle='rgba(255,255,255,0.5)';ctx.font='11px monospace';
              ctx.textAlign='center';
              for(let x=Math.ceil(xf.x0/dx)*dx;x<=xf.x1+1e-9;x+=dx)
                ctx.fillText(x.toFixed(x>=10?0:1),xf.wx(x),xf.pt+xf.dh+15);
              ctx.textAlign='right';
              for(let y=Math.ceil(xf.y0/dy)*dy;y<=xf.y1+1e-9;y+=dy)
                ctx.fillText(y.toFixed(y>=10?0:1),xf.pl-5,xf.wy(y)+4);
              ctx.fillStyle='rgba(255,255,255,0.4)';ctx.textAlign='center';ctx.font='12px monospace';
              ctx.fillText(xl,xf.pl+xf.dw/2,xf.pt+xf.dh+32);
              ctx.translate(14,xf.pt+xf.dh/2);ctx.rotate(-Math.PI/2);
              ctx.fillText(yl,0,0);ctx.restore();
            }
            function polyline(ctx,pts,xf){
              if(!pts.length)return;
              ctx.beginPath();ctx.moveTo(xf.wx(pts[0][0]),xf.wy(pts[0][1]));
              for(let i=1;i<pts.length;i++)ctx.lineTo(xf.wx(pts[i][0]),xf.wy(pts[i][1]));
              ctx.stroke();
            }

            function drawTraj(){
              const c=document.getElementById('tc');
              c.width=c.clientWidth;c.height=c.clientHeight;
              const ctx=c.getContext('2d');
              ctx.fillStyle='#0a0e1a';ctx.fillRect(0,0,c.width,c.height);

              let xMax=HOOP.dist+HOOP.radius+0.5,yMax=0,yMin=0;
              for(const t of TRAJS)for(const[x,y]of t.pts){yMax=Math.max(yMax,y);yMin=Math.min(yMin,y);}
              yMax=Math.max(yMax,HOOP.height+HOOP.radius+0.5);yMin=Math.min(yMin,-0.3);
              const xf=xform(0,xMax,yMin,yMax,58,16,20,46,c.width,c.height);
              grid(ctx,xf,1,1);

              // Min (red) and max (green) trajectory fan
              for(const t of TRAJS){
                ctx.strokeStyle=t.t==='min'?'#ff4444':'#44ff88';
                ctx.lineWidth=0.9;ctx.globalAlpha=0.45;
                polyline(ctx,t.pts,xf);
              }
              ctx.globalAlpha=1;

              // Dashed reference line at hoop height
              ctx.save();ctx.setLineDash([6,5]);ctx.strokeStyle='rgba(255,255,255,0.15)';ctx.lineWidth=1;
              ctx.beginPath();ctx.moveTo(xf.wx(0),xf.wy(HOOP.height));ctx.lineTo(xf.wx(xMax),xf.wy(HOOP.height));ctx.stroke();
              ctx.restore();

              // Vertical reference line at hoop distance
              ctx.strokeStyle='rgba(200,50,50,0.5)';ctx.lineWidth=1;
              ctx.beginPath();ctx.moveTo(xf.wx(HOOP.dist),xf.wy(yMin));ctx.lineTo(xf.wx(HOOP.dist),xf.wy(yMax));ctx.stroke();

              // Optimal trajectory
              ctx.strokeStyle='#4488ff';ctx.lineWidth=2;
              polyline(ctx,OPT_TRAJ,xf);

              // horizontal hoop opening; ball must descend through here
              ctx.strokeStyle='#ffffff';ctx.lineWidth=3;
              ctx.beginPath();
              ctx.moveTo(xf.wx(HOOP.dist-HOOP.radius),xf.wy(HOOP.height));
              ctx.lineTo(xf.wx(HOOP.dist+HOOP.radius),xf.wy(HOOP.height));
              ctx.stroke();

              labels(ctx,xf,1,1,'Distance (m)','Height (m)');
            }

            function drawRegion(){
              const c=document.getElementById('rc');
              c.width=c.clientWidth;c.height=c.clientHeight;
              const ctx=c.getContext('2d');
              ctx.fillStyle='#0a0e1a';ctx.fillRect(0,0,c.width,c.height);

              let aMin=Infinity,aMax=-Infinity,vMin=Infinity,vMax=-Infinity;
              for(const{a,lo,hi}of VR){aMin=Math.min(aMin,a);aMax=Math.max(aMax,a);vMin=Math.min(vMin,lo);vMax=Math.max(vMax,hi);}
              const ap=(aMax-aMin)*0.12,vp=(vMax-vMin)*0.18;
              const xf=xform(aMin-ap,aMax+ap,vMin-vp,vMax+vp,52,12,28,42,c.width,c.height);
              grid(ctx,xf,5,0.5);

              // upper boundary (green): max speed per angle
              ctx.strokeStyle='#44ff88';ctx.lineWidth=2;ctx.globalAlpha=0.9;
              ctx.beginPath();VR.forEach(({a,hi},i)=>i?ctx.lineTo(xf.wx(a),xf.wy(hi)):ctx.moveTo(xf.wx(a),xf.wy(hi)));ctx.stroke();
              ctx.fillStyle='#44ff88';
              for(const{a,hi}of VR){ctx.beginPath();ctx.arc(xf.wx(a),xf.wy(hi),2.5,0,Math.PI*2);ctx.fill();}

              // lower boundary (red): min speed per angle
              ctx.strokeStyle='#ff4444';ctx.lineWidth=2;
              ctx.beginPath();VR.forEach(({a,lo},i)=>i?ctx.lineTo(xf.wx(a),xf.wy(lo)):ctx.moveTo(xf.wx(a),xf.wy(lo)));ctx.stroke();
              ctx.fillStyle='#ff4444';
              for(const{a,lo}of VR){ctx.beginPath();ctx.arc(xf.wx(a),xf.wy(lo),2.5,0,Math.PI*2);ctx.fill();}

              ctx.globalAlpha=1;

              // Optimal point + error ellipse
              const ex=xf.wx(OPT.angle),ey=xf.wy(OPT.speed);
              const eRx=Math.max(3,(OPT.angM+OPT.angP)/2/(xf.x1-xf.x0)*xf.dw);
              const eRy=Math.max(3,(OPT.spdM+OPT.spdP)/2/(xf.y1-xf.y0)*xf.dh);
              ctx.strokeStyle='rgba(255,255,255,0.85)';ctx.lineWidth=1.5;
              ctx.beginPath();ctx.ellipse(ex,ey,eRx,eRy,0,0,Math.PI*2);ctx.stroke();
              ctx.fillStyle='#fff';ctx.beginPath();ctx.arc(ex,ey,3,0,Math.PI*2);ctx.fill();

              labels(ctx,xf,5,0.5,'Launch Angle (°)','Launch Speed (m/s)');
            }

            function fillInfo(){
              const f2=n=>n.toFixed(2),f1=n=>n.toFixed(1);
              document.getElementById('info').innerHTML=
                `<div class="ol">Optimal: ${f1(OPT.angle)}° @ ${f2(OPT.speed)} m/s</div>`+
                `<div class="sl">Speed: −${f2(OPT.spdM)}+${f2(OPT.spdP)} m/s</div>`+
                `<div class="al">Angle: −${f1(OPT.angM)}°+${f1(OPT.angP)}°  Optimal TOF: ${f2(OPT.tof)} s</div>`;
            }

            function render(){drawTraj();drawRegion();fillInfo();}
            window.addEventListener('load',render);
            window.addEventListener('resize',render);
            """;
}
