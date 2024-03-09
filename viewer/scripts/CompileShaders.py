import os
import subprocess
import shutil
import sys
from pathlib import Path


def compile_shader(filename, input_path, output_path, shader_type):
    return subprocess.run(['glslc',
                           os.path.join(input_path, filename + '.' + shader_type),
                           '-o', os.path.join(output_path, filename + '.' + shader_type + '.spv')],
                          check=True)


# Set up directories.
input_dir = sys.argv[1]
output_dir = sys.argv[2]
allowed_file_ext = ['vert', 'frag']

# Removes compiled shaders.
if os.path.isdir(output_dir):
    shutil.rmtree(output_dir)

# Create a directory for the compiled shaders.
Path(output_dir).mkdir(parents=True, exist_ok=True)

directory = os.fsencode(input_dir)

for file in os.listdir(directory):
    full_filename = os.fsdecode(file)
    filename, file_extension = os.path.splitext(full_filename)

    if file_extension[1:] in allowed_file_ext:
        try:
            result = compile_shader(filename, input_dir, output_dir, file_extension[1:])

            print("Compiled shader {0}.".format(full_filename))
        except subprocess.CalledProcessError as e:
            print(f'Command {e.cmd} failed with error {e.returncode}')
