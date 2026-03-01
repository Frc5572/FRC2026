package frc.robot.viz;

import java.awt.image.BufferedImage;
import java.io.File;
import java.io.IOException;
import java.util.function.ToDoubleFunction;
import javax.imageio.ImageIO;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.Pair;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.Vector;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.interpolation.InterpolatingTreeMap;
import edu.wpi.first.math.numbers.N3;

public class DrawColorMap {

    private static InterpolatingTreeMap<Double, Vector<N3>> viridis =
        new InterpolatingTreeMap<Double, Vector<N3>>((a, b, q) -> (q - a) / (b - a),
            (a, b, t) -> a.times(1.0 - t).plus(b.times(t)));

    static {
        viridis.put(0.0, VecBuilder.fill(0.267004, 0.004874, 0.329415));
        viridis.put(0.00392156862745098, VecBuilder.fill(0.26851, 0.009605, 0.335427));
        viridis.put(0.00784313725490196, VecBuilder.fill(0.269944, 0.014625, 0.341379));
        viridis.put(0.011764705882352941, VecBuilder.fill(0.271305, 0.019942, 0.347269));
        viridis.put(0.01568627450980392, VecBuilder.fill(0.272594, 0.025563, 0.353093));
        viridis.put(0.0196078431372549, VecBuilder.fill(0.273809, 0.031497, 0.358853));
        viridis.put(0.023529411764705882, VecBuilder.fill(0.274952, 0.037752, 0.364543));
        viridis.put(0.027450980392156862, VecBuilder.fill(0.276022, 0.044167, 0.370164));
        viridis.put(0.03137254901960784, VecBuilder.fill(0.277018, 0.050344, 0.375715));
        viridis.put(0.03529411764705882, VecBuilder.fill(0.277941, 0.056324, 0.381191));
        viridis.put(0.0392156862745098, VecBuilder.fill(0.278791, 0.062145, 0.386592));
        viridis.put(0.043137254901960784, VecBuilder.fill(0.279566, 0.067836, 0.391917));
        viridis.put(0.047058823529411764, VecBuilder.fill(0.280267, 0.073417, 0.397163));
        viridis.put(0.050980392156862744, VecBuilder.fill(0.280894, 0.078907, 0.402329));
        viridis.put(0.054901960784313725, VecBuilder.fill(0.281446, 0.08432, 0.407414));
        viridis.put(0.058823529411764705, VecBuilder.fill(0.281924, 0.089666, 0.412415));
        viridis.put(0.06274509803921569, VecBuilder.fill(0.282327, 0.094955, 0.417331));
        viridis.put(0.06666666666666667, VecBuilder.fill(0.282656, 0.100196, 0.42216));
        viridis.put(0.07058823529411765, VecBuilder.fill(0.28291, 0.105393, 0.426902));
        viridis.put(0.07450980392156863, VecBuilder.fill(0.283091, 0.110553, 0.431554));
        viridis.put(0.0784313725490196, VecBuilder.fill(0.283197, 0.11568, 0.436115));
        viridis.put(0.08235294117647059, VecBuilder.fill(0.283229, 0.120777, 0.440584));
        viridis.put(0.08627450980392157, VecBuilder.fill(0.283187, 0.125848, 0.44496));
        viridis.put(0.09019607843137255, VecBuilder.fill(0.283072, 0.130895, 0.449241));
        viridis.put(0.09411764705882353, VecBuilder.fill(0.282884, 0.13592, 0.453427));
        viridis.put(0.09803921568627451, VecBuilder.fill(0.282623, 0.140926, 0.457517));
        viridis.put(0.10196078431372549, VecBuilder.fill(0.28229, 0.145912, 0.46151));
        viridis.put(0.10588235294117647, VecBuilder.fill(0.281887, 0.150881, 0.465405));
        viridis.put(0.10980392156862745, VecBuilder.fill(0.281412, 0.155834, 0.469201));
        viridis.put(0.11372549019607843, VecBuilder.fill(0.280868, 0.160771, 0.472899));
        viridis.put(0.11764705882352941, VecBuilder.fill(0.280255, 0.165693, 0.476498));
        viridis.put(0.12156862745098039, VecBuilder.fill(0.279574, 0.170599, 0.479997));
        viridis.put(0.12549019607843137, VecBuilder.fill(0.278826, 0.17549, 0.483397));
        viridis.put(0.12941176470588237, VecBuilder.fill(0.278012, 0.180367, 0.486697));
        viridis.put(0.13333333333333333, VecBuilder.fill(0.277134, 0.185228, 0.489898));
        viridis.put(0.13725490196078433, VecBuilder.fill(0.276194, 0.190074, 0.493001));
        viridis.put(0.1411764705882353, VecBuilder.fill(0.275191, 0.194905, 0.496005));
        viridis.put(0.1450980392156863, VecBuilder.fill(0.274128, 0.199721, 0.498911));
        viridis.put(0.14901960784313725, VecBuilder.fill(0.273006, 0.20452, 0.501721));
        viridis.put(0.15294117647058825, VecBuilder.fill(0.271828, 0.209303, 0.504434));
        viridis.put(0.1568627450980392, VecBuilder.fill(0.270595, 0.214069, 0.507052));
        viridis.put(0.1607843137254902, VecBuilder.fill(0.269308, 0.218818, 0.509577));
        viridis.put(0.16470588235294117, VecBuilder.fill(0.267968, 0.223549, 0.512008));
        viridis.put(0.16862745098039217, VecBuilder.fill(0.26658, 0.228262, 0.514349));
        viridis.put(0.17254901960784313, VecBuilder.fill(0.265145, 0.232956, 0.516599));
        viridis.put(0.17647058823529413, VecBuilder.fill(0.263663, 0.237631, 0.518762));
        viridis.put(0.1803921568627451, VecBuilder.fill(0.262138, 0.242286, 0.520837));
        viridis.put(0.1843137254901961, VecBuilder.fill(0.260571, 0.246922, 0.522828));
        viridis.put(0.18823529411764706, VecBuilder.fill(0.258965, 0.251537, 0.524736));
        viridis.put(0.19215686274509805, VecBuilder.fill(0.257322, 0.25613, 0.526563));
        viridis.put(0.19607843137254902, VecBuilder.fill(0.255645, 0.260703, 0.528312));
        viridis.put(0.2, VecBuilder.fill(0.253935, 0.265254, 0.529983));
        viridis.put(0.20392156862745098, VecBuilder.fill(0.252194, 0.269783, 0.531579));
        viridis.put(0.20784313725490197, VecBuilder.fill(0.250425, 0.27429, 0.533103));
        viridis.put(0.21176470588235294, VecBuilder.fill(0.248629, 0.278775, 0.534556));
        viridis.put(0.21568627450980393, VecBuilder.fill(0.246811, 0.283237, 0.535941));
        viridis.put(0.2196078431372549, VecBuilder.fill(0.244972, 0.287675, 0.53726));
        viridis.put(0.2235294117647059, VecBuilder.fill(0.243113, 0.292092, 0.538516));
        viridis.put(0.22745098039215686, VecBuilder.fill(0.241237, 0.296485, 0.539709));
        viridis.put(0.23137254901960785, VecBuilder.fill(0.239346, 0.300855, 0.540844));
        viridis.put(0.23529411764705882, VecBuilder.fill(0.237441, 0.305202, 0.541921));
        viridis.put(0.23921568627450981, VecBuilder.fill(0.235526, 0.309527, 0.542944));
        viridis.put(0.24313725490196078, VecBuilder.fill(0.233603, 0.313828, 0.543914));
        viridis.put(0.24705882352941178, VecBuilder.fill(0.231674, 0.318106, 0.544834));
        viridis.put(0.25098039215686274, VecBuilder.fill(0.229739, 0.322361, 0.545706));
        viridis.put(0.2549019607843137, VecBuilder.fill(0.227802, 0.326594, 0.546532));
        viridis.put(0.25882352941176473, VecBuilder.fill(0.225863, 0.330805, 0.547314));
        viridis.put(0.2627450980392157, VecBuilder.fill(0.223925, 0.334994, 0.548053));
        viridis.put(0.26666666666666666, VecBuilder.fill(0.221989, 0.339161, 0.548752));
        viridis.put(0.27058823529411763, VecBuilder.fill(0.220057, 0.343307, 0.549413));
        viridis.put(0.27450980392156865, VecBuilder.fill(0.21813, 0.347432, 0.550038));
        viridis.put(0.2784313725490196, VecBuilder.fill(0.21621, 0.351535, 0.550627));
        viridis.put(0.2823529411764706, VecBuilder.fill(0.214298, 0.355619, 0.551184));
        viridis.put(0.28627450980392155, VecBuilder.fill(0.212395, 0.359683, 0.55171));
        viridis.put(0.2901960784313726, VecBuilder.fill(0.210503, 0.363727, 0.552206));
        viridis.put(0.29411764705882354, VecBuilder.fill(0.208623, 0.367752, 0.552675));
        viridis.put(0.2980392156862745, VecBuilder.fill(0.206756, 0.371758, 0.553117));
        viridis.put(0.30196078431372547, VecBuilder.fill(0.204903, 0.375746, 0.553533));
        viridis.put(0.3058823529411765, VecBuilder.fill(0.203063, 0.379716, 0.553925));
        viridis.put(0.30980392156862746, VecBuilder.fill(0.201239, 0.38367, 0.554294));
        viridis.put(0.3137254901960784, VecBuilder.fill(0.19943, 0.387607, 0.554642));
        viridis.put(0.3176470588235294, VecBuilder.fill(0.197636, 0.391528, 0.554969));
        viridis.put(0.3215686274509804, VecBuilder.fill(0.19586, 0.395433, 0.555276));
        viridis.put(0.3254901960784314, VecBuilder.fill(0.1941, 0.399323, 0.555565));
        viridis.put(0.32941176470588235, VecBuilder.fill(0.192357, 0.403199, 0.555836));
        viridis.put(0.3333333333333333, VecBuilder.fill(0.190631, 0.407061, 0.556089));
        viridis.put(0.33725490196078434, VecBuilder.fill(0.188923, 0.41091, 0.556326));
        viridis.put(0.3411764705882353, VecBuilder.fill(0.187231, 0.414746, 0.556547));
        viridis.put(0.34509803921568627, VecBuilder.fill(0.185556, 0.41857, 0.556753));
        viridis.put(0.34901960784313724, VecBuilder.fill(0.183898, 0.422383, 0.556944));
        viridis.put(0.35294117647058826, VecBuilder.fill(0.182256, 0.426184, 0.55712));
        viridis.put(0.3568627450980392, VecBuilder.fill(0.180629, 0.429975, 0.557282));
        viridis.put(0.3607843137254902, VecBuilder.fill(0.179019, 0.433756, 0.55743));
        viridis.put(0.36470588235294116, VecBuilder.fill(0.177423, 0.437527, 0.557565));
        viridis.put(0.3686274509803922, VecBuilder.fill(0.175841, 0.44129, 0.557685));
        viridis.put(0.37254901960784315, VecBuilder.fill(0.174274, 0.445044, 0.557792));
        viridis.put(0.3764705882352941, VecBuilder.fill(0.172719, 0.448791, 0.557885));
        viridis.put(0.3803921568627451, VecBuilder.fill(0.171176, 0.45253, 0.557965));
        viridis.put(0.3843137254901961, VecBuilder.fill(0.169646, 0.456262, 0.55803));
        viridis.put(0.38823529411764707, VecBuilder.fill(0.168126, 0.459988, 0.558082));
        viridis.put(0.39215686274509803, VecBuilder.fill(0.166617, 0.463708, 0.558119));
        viridis.put(0.396078431372549, VecBuilder.fill(0.165117, 0.467423, 0.558141));
        viridis.put(0.4, VecBuilder.fill(0.163625, 0.471133, 0.558148));
        viridis.put(0.403921568627451, VecBuilder.fill(0.162142, 0.474838, 0.55814));
        viridis.put(0.40784313725490196, VecBuilder.fill(0.160665, 0.47854, 0.558115));
        viridis.put(0.4117647058823529, VecBuilder.fill(0.159194, 0.482237, 0.558073));
        viridis.put(0.41568627450980394, VecBuilder.fill(0.157729, 0.485932, 0.558013));
        viridis.put(0.4196078431372549, VecBuilder.fill(0.15627, 0.489624, 0.557936));
        viridis.put(0.4235294117647059, VecBuilder.fill(0.154815, 0.493313, 0.55784));
        viridis.put(0.42745098039215684, VecBuilder.fill(0.153364, 0.497, 0.557724));
        viridis.put(0.43137254901960786, VecBuilder.fill(0.151918, 0.500685, 0.557587));
        viridis.put(0.43529411764705883, VecBuilder.fill(0.150476, 0.504369, 0.55743));
        viridis.put(0.4392156862745098, VecBuilder.fill(0.149039, 0.508051, 0.55725));
        viridis.put(0.44313725490196076, VecBuilder.fill(0.147607, 0.511733, 0.557049));
        viridis.put(0.4470588235294118, VecBuilder.fill(0.14618, 0.515413, 0.556823));
        viridis.put(0.45098039215686275, VecBuilder.fill(0.144759, 0.519093, 0.556572));
        viridis.put(0.4549019607843137, VecBuilder.fill(0.143343, 0.522773, 0.556295));
        viridis.put(0.4588235294117647, VecBuilder.fill(0.141935, 0.526453, 0.555991));
        viridis.put(0.4627450980392157, VecBuilder.fill(0.140536, 0.530132, 0.555659));
        viridis.put(0.4666666666666667, VecBuilder.fill(0.139147, 0.533812, 0.555298));
        viridis.put(0.47058823529411764, VecBuilder.fill(0.13777, 0.537492, 0.554906));
        viridis.put(0.4745098039215686, VecBuilder.fill(0.136408, 0.541173, 0.554483));
        viridis.put(0.47843137254901963, VecBuilder.fill(0.135066, 0.544853, 0.554029));
        viridis.put(0.4823529411764706, VecBuilder.fill(0.133743, 0.548535, 0.553541));
        viridis.put(0.48627450980392156, VecBuilder.fill(0.132444, 0.552216, 0.553018));
        viridis.put(0.49019607843137253, VecBuilder.fill(0.131172, 0.555899, 0.552459));
        viridis.put(0.49411764705882355, VecBuilder.fill(0.129933, 0.559582, 0.551864));
        viridis.put(0.4980392156862745, VecBuilder.fill(0.128729, 0.563265, 0.551229));
        viridis.put(0.5019607843137255, VecBuilder.fill(0.127568, 0.566949, 0.550556));
        viridis.put(0.5058823529411764, VecBuilder.fill(0.126453, 0.570633, 0.549841));
        viridis.put(0.5098039215686274, VecBuilder.fill(0.125394, 0.574318, 0.549086));
        viridis.put(0.5137254901960784, VecBuilder.fill(0.124395, 0.578002, 0.548287));
        viridis.put(0.5176470588235295, VecBuilder.fill(0.123463, 0.581687, 0.547445));
        viridis.put(0.5215686274509804, VecBuilder.fill(0.122606, 0.585371, 0.546557));
        viridis.put(0.5254901960784314, VecBuilder.fill(0.121831, 0.589055, 0.545623));
        viridis.put(0.5294117647058824, VecBuilder.fill(0.121148, 0.592739, 0.544641));
        viridis.put(0.5333333333333333, VecBuilder.fill(0.120565, 0.596422, 0.543611));
        viridis.put(0.5372549019607843, VecBuilder.fill(0.120092, 0.600104, 0.54253));
        viridis.put(0.5411764705882353, VecBuilder.fill(0.119738, 0.603785, 0.5414));
        viridis.put(0.5450980392156862, VecBuilder.fill(0.119512, 0.607464, 0.540218));
        viridis.put(0.5490196078431373, VecBuilder.fill(0.119423, 0.611141, 0.538982));
        viridis.put(0.5529411764705883, VecBuilder.fill(0.119483, 0.614817, 0.537692));
        viridis.put(0.5568627450980392, VecBuilder.fill(0.119699, 0.61849, 0.536347));
        viridis.put(0.5607843137254902, VecBuilder.fill(0.120081, 0.622161, 0.534946));
        viridis.put(0.5647058823529412, VecBuilder.fill(0.120638, 0.625828, 0.533488));
        viridis.put(0.5686274509803921, VecBuilder.fill(0.12138, 0.629492, 0.531973));
        viridis.put(0.5725490196078431, VecBuilder.fill(0.122312, 0.633153, 0.530398));
        viridis.put(0.5764705882352941, VecBuilder.fill(0.123444, 0.636809, 0.528763));
        viridis.put(0.5803921568627451, VecBuilder.fill(0.12478, 0.640461, 0.527068));
        viridis.put(0.5843137254901961, VecBuilder.fill(0.126326, 0.644107, 0.525311));
        viridis.put(0.5882352941176471, VecBuilder.fill(0.128087, 0.647749, 0.523491));
        viridis.put(0.592156862745098, VecBuilder.fill(0.130067, 0.651384, 0.521608));
        viridis.put(0.596078431372549, VecBuilder.fill(0.132268, 0.655014, 0.519661));
        viridis.put(0.6, VecBuilder.fill(0.134692, 0.658636, 0.517649));
        viridis.put(0.6039215686274509, VecBuilder.fill(0.137339, 0.662252, 0.515571));
        viridis.put(0.6078431372549019, VecBuilder.fill(0.14021, 0.665859, 0.513427));
        viridis.put(0.611764705882353, VecBuilder.fill(0.143303, 0.669459, 0.511215));
        viridis.put(0.615686274509804, VecBuilder.fill(0.146616, 0.67305, 0.508936));
        viridis.put(0.6196078431372549, VecBuilder.fill(0.150148, 0.676631, 0.506589));
        viridis.put(0.6235294117647059, VecBuilder.fill(0.153894, 0.680203, 0.504172));
        viridis.put(0.6274509803921569, VecBuilder.fill(0.157851, 0.683765, 0.501686));
        viridis.put(0.6313725490196078, VecBuilder.fill(0.162016, 0.687316, 0.499129));
        viridis.put(0.6352941176470588, VecBuilder.fill(0.166383, 0.690856, 0.496502));
        viridis.put(0.6392156862745098, VecBuilder.fill(0.170948, 0.694384, 0.493803));
        viridis.put(0.6431372549019608, VecBuilder.fill(0.175707, 0.6979, 0.491033));
        viridis.put(0.6470588235294118, VecBuilder.fill(0.180653, 0.701402, 0.488189));
        viridis.put(0.6509803921568628, VecBuilder.fill(0.185783, 0.704891, 0.485273));
        viridis.put(0.6549019607843137, VecBuilder.fill(0.19109, 0.708366, 0.482284));
        viridis.put(0.6588235294117647, VecBuilder.fill(0.196571, 0.711827, 0.479221));
        viridis.put(0.6627450980392157, VecBuilder.fill(0.202219, 0.715272, 0.476084));
        viridis.put(0.6666666666666666, VecBuilder.fill(0.20803, 0.718701, 0.472873));
        viridis.put(0.6705882352941176, VecBuilder.fill(0.214, 0.722114, 0.469588));
        viridis.put(0.6745098039215687, VecBuilder.fill(0.220124, 0.725509, 0.466226));
        viridis.put(0.6784313725490196, VecBuilder.fill(0.226397, 0.728888, 0.462789));
        viridis.put(0.6823529411764706, VecBuilder.fill(0.232815, 0.732247, 0.459277));
        viridis.put(0.6862745098039216, VecBuilder.fill(0.239374, 0.735588, 0.455688));
        viridis.put(0.6901960784313725, VecBuilder.fill(0.24607, 0.73891, 0.452024));
        viridis.put(0.6941176470588235, VecBuilder.fill(0.252899, 0.742211, 0.448284));
        viridis.put(0.6980392156862745, VecBuilder.fill(0.259857, 0.745492, 0.444467));
        viridis.put(0.7019607843137254, VecBuilder.fill(0.266941, 0.748751, 0.440573));
        viridis.put(0.7058823529411765, VecBuilder.fill(0.274149, 0.751988, 0.436601));
        viridis.put(0.7098039215686275, VecBuilder.fill(0.281477, 0.755203, 0.432552));
        viridis.put(0.7137254901960784, VecBuilder.fill(0.288921, 0.758394, 0.428426));
        viridis.put(0.7176470588235294, VecBuilder.fill(0.296479, 0.761561, 0.424223));
        viridis.put(0.7215686274509804, VecBuilder.fill(0.304148, 0.764704, 0.419943));
        viridis.put(0.7254901960784313, VecBuilder.fill(0.311925, 0.767822, 0.415586));
        viridis.put(0.7294117647058823, VecBuilder.fill(0.319809, 0.770914, 0.411152));
        viridis.put(0.7333333333333333, VecBuilder.fill(0.327796, 0.77398, 0.40664));
        viridis.put(0.7372549019607844, VecBuilder.fill(0.335885, 0.777018, 0.402049));
        viridis.put(0.7411764705882353, VecBuilder.fill(0.344074, 0.780029, 0.397381));
        viridis.put(0.7450980392156863, VecBuilder.fill(0.35236, 0.783011, 0.392636));
        viridis.put(0.7490196078431373, VecBuilder.fill(0.360741, 0.785964, 0.387814));
        viridis.put(0.7529411764705882, VecBuilder.fill(0.369214, 0.788888, 0.382914));
        viridis.put(0.7568627450980392, VecBuilder.fill(0.377779, 0.791781, 0.377939));
        viridis.put(0.7607843137254902, VecBuilder.fill(0.386433, 0.794644, 0.372886));
        viridis.put(0.7647058823529411, VecBuilder.fill(0.395174, 0.797475, 0.367757));
        viridis.put(0.7686274509803922, VecBuilder.fill(0.404001, 0.800275, 0.362552));
        viridis.put(0.7725490196078432, VecBuilder.fill(0.412913, 0.803041, 0.357269));
        viridis.put(0.7764705882352941, VecBuilder.fill(0.421908, 0.805774, 0.35191));
        viridis.put(0.7803921568627451, VecBuilder.fill(0.430983, 0.808473, 0.346476));
        viridis.put(0.7843137254901961, VecBuilder.fill(0.440137, 0.811138, 0.340967));
        viridis.put(0.788235294117647, VecBuilder.fill(0.449368, 0.813768, 0.335384));
        viridis.put(0.792156862745098, VecBuilder.fill(0.458674, 0.816363, 0.329727));
        viridis.put(0.796078431372549, VecBuilder.fill(0.468053, 0.818921, 0.323998));
        viridis.put(0.8, VecBuilder.fill(0.477504, 0.821444, 0.318195));
        viridis.put(0.803921568627451, VecBuilder.fill(0.487026, 0.823929, 0.312321));
        viridis.put(0.807843137254902, VecBuilder.fill(0.496615, 0.826376, 0.306377));
        viridis.put(0.8117647058823529, VecBuilder.fill(0.506271, 0.828786, 0.300362));
        viridis.put(0.8156862745098039, VecBuilder.fill(0.515992, 0.831158, 0.294279));
        viridis.put(0.8196078431372549, VecBuilder.fill(0.525776, 0.833491, 0.288127));
        viridis.put(0.8235294117647058, VecBuilder.fill(0.535621, 0.835785, 0.281908));
        viridis.put(0.8274509803921568, VecBuilder.fill(0.545524, 0.838039, 0.275626));
        viridis.put(0.8313725490196079, VecBuilder.fill(0.555484, 0.840254, 0.269281));
        viridis.put(0.8352941176470589, VecBuilder.fill(0.565498, 0.84243, 0.262877));
        viridis.put(0.8392156862745098, VecBuilder.fill(0.575563, 0.844566, 0.256415));
        viridis.put(0.8431372549019608, VecBuilder.fill(0.585678, 0.846661, 0.249897));
        viridis.put(0.8470588235294118, VecBuilder.fill(0.595839, 0.848717, 0.243329));
        viridis.put(0.8509803921568627, VecBuilder.fill(0.606045, 0.850733, 0.236712));
        viridis.put(0.8549019607843137, VecBuilder.fill(0.616293, 0.852709, 0.230052));
        viridis.put(0.8588235294117647, VecBuilder.fill(0.626579, 0.854645, 0.223353));
        viridis.put(0.8627450980392157, VecBuilder.fill(0.636902, 0.856542, 0.21662));
        viridis.put(0.8666666666666667, VecBuilder.fill(0.647257, 0.8584, 0.209861));
        viridis.put(0.8705882352941177, VecBuilder.fill(0.657642, 0.860219, 0.203082));
        viridis.put(0.8745098039215686, VecBuilder.fill(0.668054, 0.861999, 0.196293));
        viridis.put(0.8784313725490196, VecBuilder.fill(0.678489, 0.863742, 0.189503));
        viridis.put(0.8823529411764706, VecBuilder.fill(0.688944, 0.865448, 0.182725));
        viridis.put(0.8862745098039215, VecBuilder.fill(0.699415, 0.867117, 0.175971));
        viridis.put(0.8901960784313725, VecBuilder.fill(0.709898, 0.868751, 0.169257));
        viridis.put(0.8941176470588236, VecBuilder.fill(0.720391, 0.87035, 0.162603));
        viridis.put(0.8980392156862745, VecBuilder.fill(0.730889, 0.871916, 0.156029));
        viridis.put(0.9019607843137255, VecBuilder.fill(0.741388, 0.873449, 0.149561));
        viridis.put(0.9058823529411765, VecBuilder.fill(0.751884, 0.874951, 0.143228));
        viridis.put(0.9098039215686274, VecBuilder.fill(0.762373, 0.876424, 0.137064));
        viridis.put(0.9137254901960784, VecBuilder.fill(0.772852, 0.877868, 0.131109));
        viridis.put(0.9176470588235294, VecBuilder.fill(0.783315, 0.879285, 0.125405));
        viridis.put(0.9215686274509803, VecBuilder.fill(0.79376, 0.880678, 0.120005));
        viridis.put(0.9254901960784314, VecBuilder.fill(0.804182, 0.882046, 0.114965));
        viridis.put(0.9294117647058824, VecBuilder.fill(0.814576, 0.883393, 0.110347));
        viridis.put(0.9333333333333333, VecBuilder.fill(0.82494, 0.88472, 0.106217));
        viridis.put(0.9372549019607843, VecBuilder.fill(0.83527, 0.886029, 0.102646));
        viridis.put(0.9411764705882353, VecBuilder.fill(0.845561, 0.887322, 0.099702));
        viridis.put(0.9450980392156862, VecBuilder.fill(0.85581, 0.888601, 0.097452));
        viridis.put(0.9490196078431372, VecBuilder.fill(0.866013, 0.889868, 0.095953));
        viridis.put(0.9529411764705882, VecBuilder.fill(0.876168, 0.891125, 0.09525));
        viridis.put(0.9568627450980393, VecBuilder.fill(0.886271, 0.892374, 0.095374));
        viridis.put(0.9607843137254902, VecBuilder.fill(0.89632, 0.893616, 0.096335));
        viridis.put(0.9647058823529412, VecBuilder.fill(0.906311, 0.894855, 0.098125));
        viridis.put(0.9686274509803922, VecBuilder.fill(0.916242, 0.896091, 0.100717));
        viridis.put(0.9725490196078431, VecBuilder.fill(0.926106, 0.89733, 0.104071));
        viridis.put(0.9764705882352941, VecBuilder.fill(0.935904, 0.89857, 0.108131));
        viridis.put(0.9803921568627451, VecBuilder.fill(0.945636, 0.899815, 0.112838));
        viridis.put(0.984313725490196, VecBuilder.fill(0.9553, 0.901065, 0.118128));
        viridis.put(0.9882352941176471, VecBuilder.fill(0.964894, 0.902323, 0.123941));
        viridis.put(0.9921568627450981, VecBuilder.fill(0.974417, 0.90359, 0.130215));
        viridis.put(0.996078431372549, VecBuilder.fill(0.983868, 0.904867, 0.136897));
        viridis.put(1.0, VecBuilder.fill(0.993248, 0.906157, 0.143936));
    }

    public static void saveKey(String filename) throws IOException {
        int height = 1080;
        int width = 20;
        BufferedImage image = new BufferedImage(width, height, BufferedImage.TYPE_INT_ARGB);
        for (int iy = 0; iy < height; iy++) {
            var color = viridis.get(1.0 - (double) iy / (double) (height - 1));
            int r = (int) MathUtil.clamp(Math.round(color.get(0) * 255.0), 0, 255);
            int g = (int) MathUtil.clamp(Math.round(color.get(1) * 255.0), 0, 255);
            int b = (int) MathUtil.clamp(Math.round(color.get(2) * 255.0), 0, 255);
            int argb = (255 << 24) | (r << 16) | (g << 8) | b;
            for (int ix = 0; ix < width; ix++) {
                image.setRGB(ix, iy, argb);
            }
        }

        ImageIO.write(image, "png", new File(filename));
    }

    public static <T> Pair<Double, Double> draw(String filename,
        ToDoubleFunction<Translation2d> func, double xMin, double xMax, double yMin, double yMax)
        throws IOException {
        int width = 1080;
        int height = 1080;
        double[][] values = new double[width][];
        double minValue = Double.MAX_VALUE;
        double maxValue = Double.MIN_VALUE;
        for (int ix = 0; ix < width; ix++) {
            values[ix] = new double[height];
            double u = (double) ix / (double) (width - 1);
            double x = xMin + (xMax - xMin) * u;
            for (int iy = 0; iy < height; iy++) {
                double v = 1.0 - (double) iy / (double) (height - 1);
                double y = yMin + (yMax - yMin) * v;
                values[ix][iy] = func.applyAsDouble(new Translation2d(x, y));
                minValue = Math.min(minValue, values[ix][iy]);
                maxValue = Math.max(maxValue, values[ix][iy]);
            }
        }

        BufferedImage image = new BufferedImage(width, height, BufferedImage.TYPE_INT_ARGB);
        for (int ix = 0; ix < width; ix++) {
            for (int iy = 0; iy < height; iy++) {
                double v = (values[ix][iy] - minValue) / (maxValue - minValue);
                var color = viridis.get(v);
                int r = (int) MathUtil.clamp(Math.round(color.get(0) * 255.0), 0, 255);
                int g = (int) MathUtil.clamp(Math.round(color.get(1) * 255.0), 0, 255);
                int b = (int) MathUtil.clamp(Math.round(color.get(2) * 255.0), 0, 255);
                int argb = (255 << 24) | (r << 16) | (g << 8) | b;
                image.setRGB(ix, iy, argb);
            }
        }

        ImageIO.write(image, "png", new File(filename));

        return Pair.of(minValue, maxValue);
    }

}
