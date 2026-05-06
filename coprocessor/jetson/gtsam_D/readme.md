# How to Build and Run Your C++ Project

## Step-by-Step Build Instructions

### 1. Create Build Directory

```bash
cd /path/to/your/project
mkdir build
cd build
```

### 2. Configure the Build (CMake)

```bash
cmake -DCMAKE_BUILD_TYPE=Release ..
```

**Options:**
- `-DCMAKE_BUILD_TYPE=Release` — Optimized build (faster)
- `-DCMAKE_BUILD_TYPE=Debug` — With debug symbols (slower but debuggable)

### 3. Build the Project

```bash
cmake --build . -j$(nproc)
```

Or on Windows:
```cmd
cmake --build . -j %NUMBER_OF_PROCESSORS%
```

### 4. Run the Executable

**Linux/macOS:**
```bash
./bin/program
```

**Windows (from build directory):**
```cmd
.\bin\program.exe
```

Or directly:
```cmd
.\bin\Release\program.exe
```

---

