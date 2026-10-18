import os
import subprocess
import csv

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
    f = open('runtimes.csv', 'w')
    writer = csv.writer(f)
    for i in range(len(models)):
        meshFile = models[i]
        cageFile = cages[i]
        cageDeformedFile = cagesDeformed[i]
        embeddingFile = embeddings[i]
        outFile = outFiles[i]
        runtimes_row = [meshFile]
        for coordsFlag in coordinateFlags:
            print('Evaluate ' + meshFile + ' (...) ' + coordsFlag)
            runtimes_row.append(eval_runtime(meshFile, cageFile, cageDeformedFile, embeddingFile, outFile, coordsFlag))
        writer.writerow(runtimes_row)

def eval_influence_meshes():
    f = open('influence_evaluation.log', 'w')
    log = ''
    for i in range(len(models_influence)):
        meshFile = models_influence[i]
        cageFile = cages_influence[i]
        cageDeformedFile = cagesDeformed_influence[i]
        embeddingFile = embeddings_influence[i]
        outFile = outFiles_influence[i]
        for coordsFlag in coordinateFlags_influence:
            print('Evaluate ' + meshFile + ' (...) ' + coordsFlag)
            log += eval_influence(meshFile, cageFile, cageDeformedFile, embeddingFile, outFile, coordsFlag)
    f.write(log)
    f.close()

def main():
    os.chdir('../models')
    eval_influence_meshes()


if __name__ == "__main__":
    main()
