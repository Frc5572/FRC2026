package frc.robot.sim;

import com.github.stephengold.joltjni.BroadPhaseLayerInterfaceTable;
import com.github.stephengold.joltjni.JobSystem;
import com.github.stephengold.joltjni.JobSystemThreadPool;
import com.github.stephengold.joltjni.Jolt;
import com.github.stephengold.joltjni.JoltPhysicsObject;
import com.github.stephengold.joltjni.ObjectLayerPairFilterTable;
import com.github.stephengold.joltjni.ObjectVsBroadPhaseLayerFilterTable;
import com.github.stephengold.joltjni.PhysicsSystem;
import com.github.stephengold.joltjni.TempAllocator;
import com.github.stephengold.joltjni.TempAllocatorMalloc;
import edu.wpi.first.wpilibj.TimedRobot;
import electrostatic4j.snaploader.LibraryInfo;
import electrostatic4j.snaploader.LoadingCriterion;
import electrostatic4j.snaploader.NativeBinaryLoader;
import electrostatic4j.snaploader.filesystem.DirectoryPath;
import electrostatic4j.snaploader.platform.NativeDynamicLibrary;
import electrostatic4j.snaploader.platform.util.PlatformPredicate;

public class Arena2026 {

    private final TempAllocator tempAllocator;
    private final JobSystem jobSystem;
    private final PhysicsSystem physicsSystem;

    private static final int numBpLayers = 1;

    private static final int numObjLayers = 2;
    private static final int objLayerMoving = 0;
    private static final int objLayerNonMoving = 1;

    private static final int maxBodies = 5_000;
    private static final int numBodyMutexes = 0; // 0 means "use the default number"
    private static final int maxBodyPairs = 65_536;
    private static final int maxContacts = 20_480;

    private Arena2026() {
        LibraryInfo info = new LibraryInfo(null, "joltjni", DirectoryPath.USER_DIR);
        NativeBinaryLoader loader = new NativeBinaryLoader(info);
        NativeDynamicLibrary[] libraries = {
            new NativeDynamicLibrary("linux/x86-64/com/github/stephengold",
                PlatformPredicate.LINUX_X86_64),
            new NativeDynamicLibrary("osx/aarch64/com/github/stephengold",
                PlatformPredicate.MACOS_ARM_64),
            new NativeDynamicLibrary("osx/x86-64/com/github/stephengold",
                PlatformPredicate.MACOS_X86_64),
            new NativeDynamicLibrary("windows/x86-64/com/github/stephengold",
                PlatformPredicate.WIN_X86_64)};
        loader.registerNativeLibraries(libraries).initPlatformLibrary();
        try {
            loader.loadLibrary(LoadingCriterion.CLEAN_EXTRACTION);
        } catch (Exception e) {
            e.printStackTrace();
            System.exit(1);
        }

        JoltPhysicsObject.startCleaner(); // to free Jolt-Physics objects automatically
        Jolt.registerDefaultAllocator(); // tell Jolt Physics to use malloc/free

        Jolt.installDefaultAssertCallback();
        Jolt.installDefaultTraceCallback();

        boolean success = Jolt.newFactory();
        assert success;
        Jolt.registerTypes();

        this.tempAllocator = new TempAllocatorMalloc();

        int numWorkerThreads = Runtime.getRuntime().availableProcessors();
        this.jobSystem = new JobSystemThreadPool(Jolt.cMaxPhysicsJobs, Jolt.cMaxPhysicsBarriers,
            numWorkerThreads);

        ObjectLayerPairFilterTable ovoFilter = new ObjectLayerPairFilterTable(numObjLayers);
        // Enable collisions between 2 moving bodies:
        ovoFilter.enableCollision(objLayerMoving, objLayerMoving);
        // Enable collisions between a moving body and a non-moving one:
        ovoFilter.enableCollision(objLayerMoving, objLayerNonMoving);
        // Disable collisions between 2 non-moving bodies:
        ovoFilter.disableCollision(objLayerNonMoving, objLayerNonMoving);

        // Map both object layers to broadphase layer 0:
        BroadPhaseLayerInterfaceTable layerMap =
            new BroadPhaseLayerInterfaceTable(numObjLayers, numBpLayers);
        layerMap.mapObjectToBroadPhaseLayer(objLayerMoving, 0);
        layerMap.mapObjectToBroadPhaseLayer(objLayerNonMoving, 0);
        /*
         * Pre-compute the rules for colliding object layers with broadphase layers:
         */
        ObjectVsBroadPhaseLayerFilterTable ovbFilter =
            new ObjectVsBroadPhaseLayerFilterTable(layerMap, numBpLayers, ovoFilter, numObjLayers);

        physicsSystem = new PhysicsSystem();
        physicsSystem.init(maxBodies, numBodyMutexes, maxBodyPairs, maxContacts, layerMap,
            ovbFilter, ovoFilter);
    }

    private static Arena2026 instance;

    public static Arena2026 getInstance() {
        if (instance == null) {
            instance = new Arena2026();
        }

        return instance;
    }

    private void simulationSubTick(int subTickNum, double dt) {

    }

    public void simulationPeriodic() {
        final int numSubTicks = 5;
        double dt = TimedRobot.kDefaultPeriod / numSubTicks;
        for (int i = 0; i < 5; i++) {
            simulationSubTick(i, dt);
            physicsSystem.update((float) dt, 1, tempAllocator, jobSystem);
        }
    }

}
