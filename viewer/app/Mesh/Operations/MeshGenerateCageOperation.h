#pragma once

#include<Mesh/Operations/MeshOperation.h>
#include<filesystem>

struct GenerateCageFromMeshOperationParams{

GenerateCageFromMeshOperationParams(std::filesystem::path meshFilepath,
                               std::filesystem::path cageFilepath, 
                               const int scale,
                               const int smoothIterations,
                               bool smoothParamUpdated,
                               std::vector<bool>& closingResult
                               ):

                               _meshfilepath(std::move(meshFilepath)),
                               _cagefilepath(std::move(cageFilepath)),
                               _scale(scale),
                               _smoothIterations(smoothIterations),
                               _smoothParamUpdated(smoothParamUpdated),
                               _closingResult(closingResult)
                               {}

std::filesystem::path _meshfilepath;
std::filesystem::path _cagefilepath;
int _scale;
int _smoothIterations;
bool _smoothParamUpdated;
std::vector<bool>& _closingResult;


};


class GenerateCageFromMeshOperation final:public MeshOperationTemplated<GenerateCageFromMeshOperationParams,void>
{
    public:
    using MeshOperationTemplated::MeshOperationTemplated;
    [[nodiscard]] std::string GetDescription() const override
	{
		return "Generating cage";
	}

	void Execute();

};