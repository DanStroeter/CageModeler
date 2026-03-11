import os
import subprocess
import json

coordinateFlags = ['--MVC', '--harmonic', '--green', '--MEC', '--BBW', '--LBC', '--QGC', '--MLC', '--somigliana']

# In matching order
models = ['cactus.obj']#['chessBishop.obj']
cages = ['cactus_cages_quads.obj']#['bishop_cages_triangulated.off']
cagesDeformed = ['cactus_cages_quads_deformed.obj']#['bishop_cages_triangulated_deformed.obj']
embeddings = ['cactus_cages_triangulated_embedding.msh']#['bishop_cages_triangulated_embedding.msh']
outFiles = ['cactus_deformed.obj']#['chessBishop_deformed.obj']

coordinateFlags_influence = ['--MVC', '--harmonic', '--MEC', '--BBW', '--LBC', '--MLC']

models_influence = ['sphere_fine.obj']
cages_influence = ['sphere_cages_triangulated.obj']
cagesDeformed_influence = ['sphere_cages_triangulated_deformed_single.obj']
embeddings_influence = ['sphere_cages_triangulated_very_fine.msh']
outFiles_influence = ['sphere_fine_deformed.obj']

def eval_runtime(meshFile, cageFile, cageDeformedFile, embeddingFile, outFile, coordsFlag):
    command = ['cageDeformation3D', '-m', meshFile, '-c', cageFile, '--cd', cageDeformedFile, '-e', embeddingFile, '-o', outFile, coordsFlag, '-v', '0', '-t', '--interpolate-weights']
    pipe = subprocess.Popen(command, stdout=subprocess.PIPE)
    log = pipe.communicate()[0].decode("utf-8")
    if (pipe.returncode != 0):
        print('cageDeformation3D failed for ' + str(command))
        print(log)

    return float(log.split("\n")[-2])

def eval_influence(meshFile, cageFile, cageDeformedFile, embeddingFile, outFile, coordsFlag):
    command = ['cageDeformation3D', '-m', meshFile, '-c', cageFile, '--cd', cageDeformedFile, '-e', embeddingFile, '-o', outFile, coordsFlag, '--interpolate-weights', '--influence', '--iter', '10000']
    pipe = subprocess.Popen(command, stdout=subprocess.PIPE)
    log = pipe.communicate()[0].decode("utf-8")
    if (pipe.returncode != 0):
        print('cageDeformation3D failed for ' + str(command))
        print(log)
    return log 

def eval_runtimes_meshes():
    runtimes = []
    for i in range(len(models)):
        meshFile = models[i]
        cageFile = cages[i]
        cageDeformedFile = cagesDeformed[i]
        embeddingFile = embeddings[i]
        outFile = outFiles[i]
        model_runtimes = {
            'mesh': meshFile,
            'runtimes': {}
        }
        for coordsFlag in coordinateFlags:
            print('Evaluate ' + meshFile + ' (...) ' + coordsFlag)
           runtime = eval_runtime(meshFile, cageFile, cageDeformedFile, embeddingFile, outFile, coordsFlag)
            model_runtimes['runtimes'][coordsFlag] = runtime
        runtimes.append(model_runtimes)

    with open('runtimes.json', 'w') as runtimes_file:
        json.dump(runtimes, runtimes_file, indent=2)

def eval_influence_meshes():
   influence_results = []
    for i in range(len(models_influence)):
        meshFile = models_influence[i]
        cageFile = cages_influence[i]
        cageDeformedFile = cagesDeformed_influence[i]
        embeddingFile = embeddings_influence[i]
        outFile = outFiles_influence[i]
         model_influence = {
            'mesh': meshFile,
            'influence': {}
        }
        for coordsFlag in coordinateFlags_influence:
            print('Evaluate ' + meshFile + ' (...) ' + coordsFlag)
            model_influence['influence'][coordsFlag] = eval_influence(meshFile, cageFile, cageDeformedFile, embeddingFile, outFile, coordsFlag)
        influence_results.append(model_influence)

    with open('influence_evaluation.json', 'w') as influence_file:
        json.dump(influence_results, influence_file, indent=2)

def main():
    os.chdir('../models')
    eval_influence_meshes()


if __name__ == "__main__":
    main()
