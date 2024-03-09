import os
import meshio

def convertToOff(meshfile):
    try:
        mesh = meshio.read(meshfile)
        basename = os.path.splitext(os.path.basename(meshfile))[0]
        meshio.write(basename + '.off', mesh)
    except:
        print("Conversion failed for " + meshfile)

def convertMeshesToOff():
    for filename in os.listdir('.'):
        if not filename.endswith('.obj') or not 'cage' in filename:
            continue
        if 'deformed' in filename:
            continue
        convertToOff(filename)

def main():
    os.chdir('../models')
    convertMeshesToOff()

if __name__ == "__main__":
    main()
