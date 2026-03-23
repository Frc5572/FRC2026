import os
import subprocess
import time

project_dir = os.getcwd()
java_sources_root = os.path.join(project_dir, "src/main/java")
class_files_root = os.path.join(project_dir, "build/classes/java/main")
sources_prefix = "[parsing started SimpleFileObject[{}/".format(java_sources_root)
generated_prefix = "[parsing started DirectoryFileObject[{}/build/generated/sources/annotationProcessor/java/main:".format(project_dir)
done_prefix = "[wrote {}/".format(class_files_root)
check_prefix = "[checking "

print("Starting Gradle")

gradle = subprocess.Popen(["./gradlew", "compileJava"], stdout=subprocess.PIPE, stderr=subprocess.STDOUT)

starts = {}
checks = {}
ends = {}

abs_start = time.perf_counter_ns()

for i, stdout_line in enumerate(iter(gradle.stdout.readline, b'')):
    t = time.perf_counter_ns()
    if len(stdout_line) == 0:
        continue
    line = stdout_line.decode('utf-8')
    # print(line)
    if line.startswith(sources_prefix):
        name = line[len(sources_prefix):].split('.java')[0]
        starts[name] = t - abs_start
        pass
    if line.startswith(generated_prefix):
        name = line[len(generated_prefix):].split('.java')[0]
        starts[name] = t - abs_start
        pass
    elif line.startswith(done_prefix):
        name = line[len(done_prefix):].split('.class')[0].split('$')[0]
        ends[name] = t - abs_start
        pass
    elif line.startswith(check_prefix):
        name = line[len(check_prefix):].split(']')[0].replace('.', '/')
        checks[name] = t - abs_start
        pass

gradle.stdout.close()
return_code = gradle.wait()

with open('compile_times.csv', 'w') as f:
    f.write("Class,Start,Check,End\n")
    for key,start in starts.items():
        end = ends[key]
        check = checks[key]
        f.write("{},{},{},{}\n".format(key, start, check, end))
