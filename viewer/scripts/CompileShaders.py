import os
import subprocess
import shutil
import sys
from pathlib import Path

def compile_shader(filename, input_path, output_path, shader_type):
    stage_flags = {
        'vert': 'vertex',
        'frag': 'fragment',
        'comp': 'compute',
        'comp.glsl': 'compute',
        'rgen': 'rgen',
        'rmiss': 'rmiss', 
        'rchit': 'rchit',
    }
    
    if shader_type not in stage_flags:
        return False
    
    input_file = os.path.join(input_path, filename + '.' + shader_type)
    output_file = os.path.join(output_path, filename + '.' + shader_type + '.spv')
    
    # For ray tracing shaders
    if shader_type.startswith('r'):
        # With your NEW SDK, you need to explicitly target SPIR-V 1.5
        cmd = [
            'glslc',
            input_file,
            '-o', output_file,
            f'-fshader-stage={stage_flags[shader_type]}',
            '--target-spv=spv1.5',           # ← CRITICAL: Target SPIR-V 1.5
            '--target-env=vulkan1.3',        # ← Target Vulkan 1.3
            '-std=460'                       # ← GLSL 460
        ]
        
        try:
            result = subprocess.run(cmd, capture_output=True, text=True)
            if result.returncode == 0:
                print(f"Compiled {filename}.{shader_type}")
                return True
            else:
                print(f"Failed {filename}.{shader_type}: {result.stderr}")
                return False
        except Exception as e:
            print(f"Error compiling {filename}.{shader_type}: {e}")
            return False
    else:
        # Standard shaders
        cmd = ['glslc', input_file, '-o', output_file]
        try:
            subprocess.run(cmd, check=True, capture_output=True, text=True)
            print(f"Compiled {filename}.{shader_type}")
            return True
        except subprocess.CalledProcessError as e:
            print(f"Failed {filename}.{shader_type}: {e.stderr}")
            return False

# Main
if __name__ == "__main__":
    if len(sys.argv) != 3:
        print("Usage: python compile_shaders.py <input_dir> <output_dir>")
        sys.exit(1)
    
    input_dir = sys.argv[1]
    output_dir = sys.argv[2]
    
    if os.path.isdir(output_dir):
        shutil.rmtree(output_dir)
    Path(output_dir).mkdir(parents=True, exist_ok=True)
    
    success = 0
    total = 0
    
    for file in os.listdir(input_dir):
        filename, ext = os.path.splitext(file)
        if ext[1:] in ['vert', 'frag', 'comp', 'rgen', 'rmiss', 'rchit']:
            total += 1
            if compile_shader(filename, input_dir, output_dir, ext[1:]):
                success += 1
    
    print(f"\nSummary: {success}/{total} shaders compiled")