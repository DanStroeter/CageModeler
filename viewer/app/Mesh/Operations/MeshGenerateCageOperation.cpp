#include<Mesh/Operations/MeshGenerateCageOperation.h>

#include <iostream>
#include <utility>
#include <array>
#include <fstream>
#include <algorithm>
#include <map>
#include <unordered_map>
#include <unordered_set>

#include <omp.h>
#include <ctime>
#define CGAL_EIGEN3_ENABLED
//#define DETERMINE_VOXEL_SIZE

#include <CGAL/Exact_predicates_inexact_constructions_kernel.h>
#include <CGAL/Exact_predicates_exact_constructions_kernel.h>
#include <CGAL/Polyhedron_3.h>
#include <CGAL/Polygon_mesh_processing/IO/polygon_mesh_io.h>
#include <CGAL/boost/graph/convert_nef_polyhedron_to_polygon_mesh.h>
#include <CGAL/Optimal_bounding_box/oriented_bounding_box.h>
#include <CGAL/Polygon_mesh_processing/intersection.h>
#include <CGAL/Polygon_mesh_processing/stitch_borders.h>
#include <CGAL/Polygon_mesh_processing/polygon_soup_to_polygon_mesh.h>
#include <CGAL/Polygon_mesh_processing/triangulate_faces.h>
#include <CGAL/Polygon_mesh_processing/polygon_mesh_to_polygon_soup.h>
#include <CGAL/Polygon_mesh_processing/repair_degeneracies.h>
#include <CGAL/Polygon_mesh_processing/compute_normal.h>
#include <CGAL/Polygon_mesh_processing/corefinement.h>
#include <CGAL/Side_of_triangle_mesh.h>
#include <CGAL/Polyhedron_incremental_builder_3.h>

#include <CGAL/Nef_polyhedron_3.h>
#include <CGAL/Surface_mesh.h>
#include <CGAL/AABB_tree.h>
#include <CGAL/Surface_mesh_simplification/edge_collapse.h>
#include <CGAL/Surface_mesh_simplification/Policies/Edge_collapse/Edge_length_cost.h>
#include <CGAL/Surface_mesh_simplification/Policies/Edge_collapse/Edge_length_stop_predicate.h>
#include <CGAL/Surface_mesh_simplification/Policies/Edge_collapse/Midpoint_placement.h>
#include <CGAL/Surface_mesh_simplification/Policies/Edge_collapse/Polyhedral_envelope_filter.h>
#include <CGAL/Surface_mesh_simplification/Policies/Edge_collapse/Bounded_normal_change_filter.h>
#include <CGAL/Surface_mesh_simplification/Policies/Edge_collapse/GarlandHeckbert_policies.h>
#include <CGAL/Surface_mesh_simplification/Policies/Edge_collapse/Bounded_normal_change_placement.h>
#include <CGAL/Surface_mesh_simplification/Policies/Edge_collapse/Constrained_placement.h>
#include <CGAL/Surface_mesh_simplification/Policies/Edge_collapse/Edge_count_stop_predicate.h>
#include <boost/optional/optional_io.hpp>

namespace PMP = CGAL::Polygon_mesh_processing;
namespace SMS = CGAL::Surface_mesh_simplification;

typedef CGAL::Exact_predicates_exact_constructions_kernel			Exact_Kernel;
typedef CGAL::Exact_predicates_inexact_constructions_kernel			Inexact_Kernel;
typedef CGAL::Polyhedron_3<Inexact_Kernel>							Inexact_Polyhedron;
typedef CGAL::Polyhedron_3<Exact_Kernel>							Exact_Polyhedron;
typedef CGAL::Nef_polyhedron_3<Exact_Kernel>						Nef_polyhedron;
typedef Exact_Kernel::Point_3										ExactPoint;
typedef Inexact_Kernel::Point_3										InexactPoint;
typedef Inexact_Kernel::Triangle_3									Triangle;
typedef Exact_Kernel::Vector_3										ExactVector;
typedef CGAL::Surface_mesh<InexactPoint>							Mesh;
typedef CGAL::Surface_mesh<ExactPoint>								ExactMesh;

typedef CGAL::AABB_face_graph_triangle_primitive<Exact_Polyhedron>	Primitive;
typedef CGAL::AABB_traits<Exact_Kernel, Primitive>					Traits;
typedef CGAL::AABB_tree<Traits>										Tree;
typedef CGAL::Side_of_triangle_mesh<Exact_Polyhedron, Exact_Kernel>	Point_inside;
typedef SMS::Polyhedral_envelope_filter<Inexact_Kernel,
	SMS::Bounded_normal_change_filter<>>							Filter;

typedef boost::graph_traits<Inexact_Polyhedron>::edge_descriptor      edge_descriptor;
typedef boost::graph_traits<Inexact_Polyhedron>                             GraphTraits;
typedef typename GraphTraits::halfedge_descriptor                           halfedge_descriptor;
typedef typename GraphTraits::vertex_descriptor                             vertex_descriptor;

typedef CGAL::Simple_cartesian<double>                                  Kernel;
typedef CGAL::Polyhedron_3<Kernel>                                      Surface_mesh;
typedef Kernel::Point_3                                                 Surface_Point;
typedef SMS::GarlandHeckbert_plane_policies<Surface_mesh,Kernel>         QEM_Policies;
typedef Surface_mesh::Halfedge_handle                             HalfedgeHandle;
typedef Surface_mesh::Edge_iterator                                EdgeIterator;
typedef CGAL::Side_of_triangle_mesh<Surface_mesh, Kernel>          Point_inside_simp;
typedef boost::graph_traits<Surface_mesh>::edge_descriptor        sedge_descriptor;
typedef boost::graph_traits<Surface_mesh>::halfedge_descriptor    shalfedge_descriptor;


using Grid3D=std::vector<std::vector<std::vector<bool>>>; //represent the voxelized grid
using MipmapTree=std::vector<std::map<std::array<int,3>,bool>>;  //used to save the hierarchy


Grid3D initializeGrid(int x, int y, int z){  //initialize the grid with given dimensions in x,y,z drtections

    return Grid3D(x,std::vector<std::vector<bool>>(y,std::vector<bool>(z,false)));
}


void convert_voxel_idx_to_coords(unsigned int idx, std::array<unsigned int, 3> numVoxels, unsigned int& x_idx, unsigned int& y_idx, unsigned int& z_idx)
{
	x_idx = idx / (numVoxels[1] * numVoxels[2]);
	auto const w = idx % (numVoxels[1] * numVoxels[2]);
	y_idx = w / numVoxels[2];
	z_idx = w % numVoxels[2];
}

std::array<ExactPoint, 8> calc_voxel_points(unsigned int idx, std::array<unsigned int, 3> numVoxels, ExactPoint min_point,
	const std::array<ExactVector, 3>& voxel_strides, bool* new_scanline = nullptr)
{
	unsigned int x_idx, y_idx, z_idx;

	convert_voxel_idx_to_coords(idx, numVoxels, x_idx, y_idx, z_idx);

	if (new_scanline)
	{
		*new_scanline = z_idx == 0;
	}

	return {
		min_point + x_idx * voxel_strides[0] + y_idx * voxel_strides[1] + z_idx * voxel_strides[2],
		min_point + (x_idx + 1u) * voxel_strides[0] + y_idx * voxel_strides[1] + z_idx * voxel_strides[2],
		min_point + x_idx * voxel_strides[0] + (y_idx + 1u) * voxel_strides[1] + z_idx * voxel_strides[2],
		min_point + (x_idx + 1u) * voxel_strides[0] + (y_idx + 1u) * voxel_strides[1] + z_idx * voxel_strides[2],
		min_point + x_idx * voxel_strides[0] + y_idx * voxel_strides[1] + (z_idx + 1u) * voxel_strides[2],
		min_point + (x_idx + 1u) * voxel_strides[0] + y_idx * voxel_strides[1] + (z_idx + 1u) * voxel_strides[2],
		min_point + x_idx * voxel_strides[0] + (y_idx + 1u) * voxel_strides[1] + (z_idx + 1u) * voxel_strides[2],
		min_point + (x_idx + 1u) * voxel_strides[0] + (y_idx + 1u) * voxel_strides[1] + (z_idx + 1u) * voxel_strides[2]
	};
}


void calc_voxel_from_idx_tets(unsigned int idx, std::array<unsigned int, 3> numVoxels, ExactPoint min_point,
	const std::array<ExactVector, 3>& voxel_strides, Exact_Polyhedron& voxel, bool* new_scanline)
{
	auto const p = calc_voxel_points(idx, numVoxels, min_point, voxel_strides, new_scanline);

	voxel.make_tetrahedron(p[0], p[3], p[5], p[1]);
	voxel.make_tetrahedron(p[0], p[3], p[2], p[6]);
	voxel.make_tetrahedron(p[0], p[4], p[5], p[6]);
	voxel.make_tetrahedron(p[5], p[6], p[7], p[3]);
	assert(voxel.is_valid());
}

void calc_voxel_from_idx_hex(unsigned int idx, std::array<unsigned int, 3> numVoxels, ExactPoint min_point,
	const std::array<ExactVector, 3>& voxel_strides, Exact_Polyhedron& voxel)
{
	auto const p = calc_voxel_points(idx, numVoxels, min_point, voxel_strides);

	CGAL::make_hexahedron(p[0], p[1], p[3], p[2], p[6], p[4], p[5], p[7], voxel);
	assert(voxel.is_valid());
}

//output the grid into off file
void writeGridIntoObj(const Grid3D& grid, const std::string& filepath){
	std::ofstream file(filepath);
	if(!file.is_open()){
      std::cerr<<"Error:Unable to open the file"<<std::endl;
	  return;
	}

    std::vector<std::array<double,3>> vertices;
    std::vector<std::array<int,4>> faces;

	//traverse the grid, calculate vertices and faces
	for(int x=0;x<grid.size();++x){
		for(int y=0;y<grid[0].size();++y){
			for(int z=0;z<grid[0][0].size();++z){
				if(grid[x][y][z]){
					//calculate the position of 8 vertices of the voxel cube
					int baseIndex=vertices.size()+1;
					double dx=static_cast<double>(x);
					double dy=static_cast<double>(y);
					double dz=static_cast<double>(z);

                    //add 8 vertices of the cube
					vertices.push_back({dx,dy,dz});
					vertices.push_back({dx+1,dy,dz});
					vertices.push_back({dx+1,dy+1,dz});
					vertices.push_back({dx,dy+1,dz});
					vertices.push_back({dx,dy,dz+1});
					vertices.push_back({dx+1,dy,dz+1});
					vertices.push_back({dx+1,dy+1,dz+1});
					vertices.push_back({dx,dy+1,dz+1});

					//add 6 faces of the cube
					faces.push_back({baseIndex,baseIndex+1,baseIndex+2,baseIndex+3});
					faces.push_back({baseIndex+4,baseIndex+5,baseIndex+6,baseIndex+7});
					faces.push_back({baseIndex,baseIndex+1,baseIndex+5,baseIndex+4});
					faces.push_back({baseIndex+1,baseIndex+2,baseIndex+6,baseIndex+5});
					faces.push_back({baseIndex+2,baseIndex+3,baseIndex+7,baseIndex+6});
					faces.push_back({baseIndex+3,baseIndex,baseIndex+4,baseIndex+7});




				}
			}
		}
	}

//write into the obj file

//write the vertices
for(const auto& vertex: vertices){
	file<<"v "<<vertex[0]<<" "<<vertex[1]<<" "<<vertex[2]<<"\n";
}
    
//write the faces
for(const auto& face: faces){
	file<<"f "<<face[0]<<" "<<face[1]<<" "<<face[2]<<" "<<face[3]<<"\n";
}

file.close();
std::cout<<"3D grid has been written to file!"<<std::endl;

}

//build hierarchy from the finest level and save into the mipmapTree
MipmapTree buildHeirarchy(const Grid3D& fineGrid, int levels){  
	MipmapTree mipmaptree;
	Grid3D currentGrid=fineGrid;

	for(int level=0; level<levels; ++level){
	
	//create a map for the current level
	std::map<std::array<int,3>,bool> levelMap;

	//assign the boolean value for the current level
	for(int x=0;x<currentGrid.size(); ++x){
		for(int y=0;y<currentGrid[x].size();++y){
			for(int z=0;z<currentGrid[x][y].size(); ++z){
				if(currentGrid[x][y][z]==true){
					levelMap[{x,y,z}]=true;    //save the occupied voxel
				}
			}
		}
	}

	mipmaptree.push_back(levelMap);   //add into the mipmaptree

	//build the next coarser level
	if(level<levels-1){
		//compute the dimensions of the new level
		int newX=std::max(1,static_cast<int>(currentGrid.size()/2));
		int newY=std::max(1,static_cast<int>(currentGrid[0].size()/2));
		int newZ=std::max(1,static_cast<int>(currentGrid[0][0].size()/2));

        Grid3D nextGrid=initializeGrid(newX,newY,newZ);  //voxel grid of the new level

		//traverse every voxels in the new level
		for(int x=0;x<newX;++x){
			for(int y=0;y<newY;++y){
				for(int z=0;z<newZ;++z){
					bool occupied=false;    //initialized the boolean value of every voxel
					//calculate the offsets when use 8 finer grid to represent 1 coarser grid,the values of dx,dy,dz will iterate between 0,1
					for(int dx=0;dx<2;++dx){
						for(int dy=0;dy<2;++dy){
							for(int dz=0;dz<2;++dz){
								//calculate the 8(2*2*2) finer grids' positions which represent a coarser grid 
								int fineX=2*x+dx;
								int fineY=2*y+dy;
								int fineZ=2*z+dz;

								if(fineX<currentGrid.size() && fineY<currentGrid[0].size() && fineZ<currentGrid[0][0].size()){
									occupied=occupied || currentGrid[fineX][fineY][fineZ];    //calculate the occupation of the coarser grid, it is considered occupied as long as one of the finer grid is occupied
								}
							}
						}
					} 

					nextGrid[x][y][z]=occupied;

				}
			}
		}

		currentGrid=nextGrid;  //downscale recursively
	}
	}

	return mipmaptree;
}


//print the mipmap for debug
void printMipmap(const MipmapTree& mipmaptree){
	for(size_t level=0; level<mipmaptree.size();++level){
		std::cout<<"Level"<<level<<":\n";
		for(const auto& [coord,occupied]: mipmaptree[level]){
			if(occupied){
				std::cout<<"("<<coord[0]<<","<<coord[1]<<","<<coord[2]<<"):"<<occupied<<std::endl;
			}
		}
		std::cout<<"\n";
	}
}


//dilation
//check the intersection between the SE of a voxel and the node

bool checkIntersect(const std::array<int,3>& node, int level,const std::array<int,3>& center,int seSize){
	
	//the region of the node box
	int nodeBoxSize=1<<level;
	int nodeBoxMinX=node[0] * nodeBoxSize, nodeBoxMaxX=(node[0]+1) * nodeBoxSize;
	int nodeBoxMinY=node[1] * nodeBoxSize, nodeBoxMaxY=(node[1]+1) * nodeBoxSize;
	int nodeBoxMinZ=node[2] * nodeBoxSize, nodeBoxMaxZ=(node[2]+1) * nodeBoxSize;


	//the region of the SE
	int seMinX=center[0]-seSize, seMaxX=center[0]+seSize+1;
	int seMinY=center[1]-seSize, seMaxY=center[1]+seSize+1;
	int seMinZ=center[2]-seSize, seMaxZ=center[2]+seSize+1;

	//check the overlap
	bool xOverlap=std::max(nodeBoxMinX,seMinX)< std::min(nodeBoxMaxX,seMaxX);
	bool yOverlap=std::max(nodeBoxMinY,seMinY)< std::min(nodeBoxMaxY,seMaxY);
	bool zOverlap=std::max(nodeBoxMinZ,seMinZ)< std::min(nodeBoxMaxZ,seMaxZ);

return xOverlap && yOverlap && zOverlap;


}

bool isValid(int x, int y,int z,const std::array<int,3>& gridSizes ){
	return x>=0 && x<gridSizes[0] && y>=0 && y< gridSizes[1] && z>=0 && z<gridSizes[2];
}

bool checkIntersectE(const std::array<int,3>& node, int level,const std::array<int,3>& center,int seSize, const std::array<int,3>& gridSizes){
	bool result=false;
	//the region of the node box
	int nodeBoxSize=1<<level;
	int nodeBoxMinX=node[0] * nodeBoxSize, nodeBoxMaxX=(node[0]+1) * nodeBoxSize;
	int nodeBoxMinY=node[1] * nodeBoxSize, nodeBoxMaxY=(node[1]+1) * nodeBoxSize;
	int nodeBoxMinZ=node[2] * nodeBoxSize, nodeBoxMaxZ=(node[2]+1) * nodeBoxSize;

    std::vector<std::array<int,3>> sphere_voxels;
	//the region of the sphere SE
	for(int x=center[0]-seSize; x<center[0]+seSize+1; ++x){
		for(int y=center[1]-seSize; y<center[1]+seSize+1; ++y){
			for(int z=center[2]-seSize; z<center[2]+seSize+1; ++z){
                 if((std::pow((x-center[0]),2)+ std::pow((y-center[1]),2) +std::pow((z-center[2]),2))<=std::pow(seSize,2) && isValid(x,y,z,gridSizes) ){
                        sphere_voxels.push_back({x,y,z});
				 }
			}
		}
	}
	
	

	//check the overlap
	for(int x=nodeBoxMinX; x<nodeBoxMaxX;++x){
		for(int y=nodeBoxMinY;y<nodeBoxMaxY; ++y){
			for(int z=nodeBoxMinZ;z<nodeBoxMaxZ; ++z){
               std::array<int,3> node_voxel={x,y,z};
			   if(std::find(sphere_voxels.begin(),sphere_voxels.end(),node_voxel)!=sphere_voxels.end()){
				result=true;
				break;
			   }
			}
		}
	}
//	bool xOverlap=std::max(nodeBoxMinX,seMinX)< std::min(nodeBoxMaxX,seMaxX);
//	bool yOverlap=std::max(nodeBoxMinY,seMinY)< std::min(nodeBoxMaxY,seMaxY);
//	bool zOverlap=std::max(nodeBoxMinZ,seMinZ)< std::min(nodeBoxMaxZ,seMaxZ);

//return xOverlap && yOverlap && zOverlap;
return result;

}

Grid3D parallelDilationA(Grid3D& grid, MipmapTree& mipmap, int se_scale){
	Grid3D result = grid; 
    int maxLevel = mipmap.size() - 1; 
    int gridSizeX = grid.size();
    int gridSizeY = grid[0].size();
    int gridSizeZ = grid[0][0].size();
	std::vector<std::array<int,3>> originalV;

	for(int x=0;x<gridSizeX;++x){
		for(int y=0;y<gridSizeY;++y){
			for(int z=0;z<gridSizeZ;++z){
				if(result[x][y][z]==false){
					originalV.push_back({x,y,z});
				}
			}
		}
	}

   struct Pair{
	int level;
	std::array<int,3> position;
   };
 
  for(auto ov:originalV){
     // std::stack<std::pair<int, std::array<int, 3>>> stack;
	// std::cout<<"processing target voxel: ("<<ov[0]<<","<<ov[1]<<","<<ov[2]<<")"<<std::endl;
	 std::vector<Pair> stack;
      stack.push_back({maxLevel, {0,0,0}});
	  while(!stack.empty() && result[ov[0]][ov[1]][ov[2]]!=true){
		Pair top=stack.back();
         stack.pop_back();
		// std::cout<<"pop the stack top:("<<top.level<<", ("<<top.position[0]<<","<<top.position[1]<<","<<top.position[2]<<")"<<std::endl;


       

		 if(top.level==0){
   
			result[ov[0]][ov[1]][ov[2]]=true;
		//	std::cout<<"add new dilation: ("<<ov[0]<<","<<ov[1]<<","<<ov[2]<<std::endl;
			
		 }
		 else{
			for(int dx=0;dx<=1;++dx){
				for(int dy=0; dy<=1;++dy){
					for(int dz=0;dz<=1;++dz){
						 std::array<int, 3> subNode = {
                                           top.position[0]*2 + dx ,
                                            top.position[1]*2 + dy ,
                                            top.position[2] *2+ dz 
											};

						//	std::cout<<"checking the subnode :("<<subNode[0]<<","<<subNode[1]<<","<<subNode[2]<<")"<<"in level"<<top.level-1<<std::endl;
                       if(mipmap[top.level - 1].count(subNode) &&
                         mipmap[top.level - 1].at(subNode) ){
						//	std::cout<<"the subnode in the level"<<top.level-1<<" is occupied"<<std::endl;
				  
				  if( checkIntersect(subNode, top.level - 1, ov, se_scale)){
        		//	if( checkIntersectE(subNode, top.level - 1, ov, se_scale,{gridSizeX,gridSizeY,gridSizeZ})){
						//   std::cout<<"the subNode is valid"<<std::endl;
                           stack.push_back({top.level-1,subNode});
					        }
						 }
                    
					}
				}
			}
		 }
	  }
  }

return result;

}

void grid_into_voxelized_off(Grid3D& grid, const std::string& outputfile,std::array<unsigned int, 3>& numVoxels,
std::array<ExactPoint, 8>& obb_points,std::array<ExactVector, 3>& voxel_strides){

	std::vector<unsigned int> activated_voxels;
    Exact_Polyhedron voxels;

	for(int x=0;x<numVoxels[0];++x){
       for(int y=0;y<numVoxels[1];++y){
		  for(int z=0;z<numVoxels[2]; ++z){

			if (grid[x][y][z]==true){
 
                //calculate the index of the activated voxel
				int index=x* (numVoxels[1] * numVoxels[2]) + y * numVoxels[2] + z;

                activated_voxels.push_back(index);
			}

		  }
	   }
	}

   for(auto voxel_idx : activated_voxels){
        calc_voxel_from_idx_hex(voxel_idx,numVoxels,obb_points[7],voxel_strides,voxels);
   }

   CGAL::IO::write_OFF(outputfile,voxels);
	
}

//extract 6-connected contour of dilatioin
Grid3D extract_contour(Grid3D& dilation){
	Grid3D result=dilation;
	int gridSizeX=dilation.size();
	int gridSizeY=dilation[0].size();
	int gridSizeZ=dilation[0][0].size();

  	std::vector<std::array<int,3>> originalV;

    //collect the voxels
	for(int x=1;x<gridSizeX-1;++x){
		for(int y=1;y<gridSizeY-1;++y){
			for(int z=1;z<gridSizeZ-1;++z){
				if(result[x][y][z]==true){
					originalV.push_back({x,y,z});
				}
			}
		}
	}

   for(auto ov : originalV){
     
	 //check  6 neighboors of the voxel
	   if(dilation[ov[0]+1][ov[1]][ov[2]]==true &&
	      dilation[ov[0]-1][ov[1]][ov[2]]==true &&
	      dilation[ov[0]][ov[1]+1][ov[2]]==true &&
	      dilation[ov[0]][ov[1]-1][ov[2]]==true &&
	      dilation[ov[0]][ov[1]][ov[2]+1]==true &&
	      dilation[ov[0]][ov[1]][ov[2]-1]==true ){
			result[ov[0]][ov[1]][ov[2]]=false;
		  }
   }

   return result;


}

Grid3D cut_the_half(Grid3D& grid){

    Grid3D result=grid;
	int gridSizeX=grid.size();
	int gridSizeY=grid[0].size();
	int gridSizeZ=grid[0][0].size();

	for(int x=0; x<gridSizeX; ++x){
		for(int y=0; y<gridSizeY; ++y){
			for(int z=gridSizeZ/2;z<gridSizeZ; ++z){
				result[x][y][z]=false;
			}
		}
	}

return result;

}



//create sphere SE
std::vector<std::array<int,3>> generate_sphere_offsets(int radius){
std::vector<std::array<int,3>> offsets;

for(int x=-radius; x<=radius; ++x){
	for(int y=-radius; y<=radius; ++y){
		for(int z=-radius; z<=radius; ++z){
			if((x*x+y*y+z*z)<=std::pow(radius,2)){
				offsets.push_back({x,y,z});
			}
		}
	}
}
return offsets;
	
}


int getIndex(int x, int y, int z, const std::array<int, 3>& gridSizes) {
    return x + y * gridSizes[0] + z * gridSizes[0] * gridSizes[1];
}

bool check_intersect_erosionE(const std::array<int,3>& node, int level,const std::array<int,3>& voxelp,int seSize,const std::array<int,3>& gridSizes,std::vector<std::array<int,3>>& offsets){


bool result=false;
//the rigoin of dilated subnode box
//the rigion of the subnode box
int nodeBoxSize=1<<level;
int nodeBoxMinX=node[0] * nodeBoxSize, nodeBoxMaxX=(node[0]+1) * nodeBoxSize;
int nodeBoxMinY=node[1] * nodeBoxSize, nodeBoxMaxY=(node[1]+1) * nodeBoxSize;
int nodeBoxMinZ=node[2] * nodeBoxSize, nodeBoxMaxZ=(node[2]+1) * nodeBoxSize;

//the rigon of dilated subnode box
//std::vector<std::array<int,3>> dilated_voxels;
std::vector<uint8_t> dilated_voxels(gridSizes[0] * gridSizes[1] * gridSizes[2], 0);


//the sphere offset
//std::vector<std::array<int,3>> offsets= generate_sphere_offsets(seSize);

//do the dilation
#pragma omp parallel for collapse(3) 
for(int x=nodeBoxMinX; x<nodeBoxMaxX; ++x){
	for(int y=nodeBoxMinY; y<nodeBoxMaxY; ++y){
		for(int z=nodeBoxMinZ; z<nodeBoxMaxZ; ++z){
           std::array<int,3> box_voxel={x,y,z};
           for(const auto& offset: offsets){
               int nx=x+offset[0];
               int ny=y+offset[1];
               int nz=z+offset[2];
			   if(isValid(nx,ny,nz,gridSizes)){
				//dilated_voxels.push_back({nx,ny,nz});
				  dilated_voxels[getIndex(nx, ny, nz, gridSizes)] = 1;
			   }

		   }

		  
		}
	}
}


//calculate the rigion of checked voxel box
for(int x=voxelp[0];x<=voxelp[0]+1; ++x){
	for(int y=voxelp[1]; y<=voxelp[1]+1; ++y){
		for(int z=voxelp[2]; z<=voxelp[2]+1; ++z){
			std::array<int,3> pos={x,y,z};
			/*
			if(std::find(dilated_voxels.begin(),dilated_voxels.end(),pos)!=dilated_voxels.end()){
				result=true;
				break;
			}
			*/
		  int idx=getIndex(x, y, z, gridSizes);
		  if (dilated_voxels[idx]) {
                    result= true;
					break;
                }
		    
		}
	}
}

return result;

}




bool check_intersect_erosion(const std::array<int,3>& node, int level,
                              const std::array<int,3>& voxelp, int seSize,
                              const std::array<int,3>& gridSizes,
                              const std::vector<std::array<int,3>>& offsets) {

    bool result = false;
    int nodeBoxSize = 1 << level;
    int nodeBoxMinX = node[0] * nodeBoxSize, nodeBoxMaxX = (node[0] + 1) * nodeBoxSize;
    int nodeBoxMinY = node[1] * nodeBoxSize, nodeBoxMaxY = (node[1] + 1) * nodeBoxSize;
    int nodeBoxMinZ = node[2] * nodeBoxSize, nodeBoxMaxZ = (node[2] + 1) * nodeBoxSize;

    // 改为 std::vector<uint8_t> 避免 bit 操作的额外开销
    std::vector<uint8_t> dilated_voxels(gridSizes[0] * gridSizes[1] * gridSizes[2], 0);

    // 仅并行最外层循环，避免过多线程竞争
    #pragma omp parallel for
    for (int x = nodeBoxMinX; x < nodeBoxMaxX; ++x) {
        for (int y = nodeBoxMinY; y < nodeBoxMaxY; ++y) {
            for (int z = nodeBoxMinZ; z < nodeBoxMaxZ; ++z) {
                int baseIdx = getIndex(x, y, z, gridSizes);  // 预计算索引
                for (const auto& offset : offsets) {
                    int nx = x + offset[0];
                    int ny = y + offset[1];
                    int nz = z + offset[2];

                    if (isValid(nx, ny, nz, gridSizes)) {
                        int idx = getIndex(nx, ny, nz, gridSizes);
                        dilated_voxels[idx] = 1;
                    }
                }
            }
        }
    }

    // 检查目标体素区域
    for (int x = voxelp[0]; x <= voxelp[0] + 1; ++x) {
        for (int y = voxelp[1]; y <= voxelp[1] + 1; ++y) {
            for (int z = voxelp[2]; z <= voxelp[2] + 1; ++z) {
                int idx = getIndex(x, y, z, gridSizes);
                if (dilated_voxels[idx]) {
                    return true;
                }
            }
        }
    }

    return result;
}



bool check_intersect_erosion(const std::array<int,3>& node, int level,const std::array<int,3>& voxelp,int seSize,const std::array<int,3> gridSizes){

//the region of dilated subnode box 
	//the region of the node box
	int nodeBoxSize=1<<level;
	int nodeBoxMinX=node[0] * nodeBoxSize, nodeBoxMaxX=(node[0]+1) * nodeBoxSize;
	int nodeBoxMinY=node[1] * nodeBoxSize, nodeBoxMaxY=(node[1]+1) * nodeBoxSize;
	int nodeBoxMinZ=node[2] * nodeBoxSize, nodeBoxMaxZ=(node[2]+1) * nodeBoxSize;

	//the offset created by sphere SE
	int newBoxMinX=std::max(0,nodeBoxMinX-seSize);
	int newBoxMaxX=std::min(nodeBoxMaxX+seSize,gridSizes[0]);
	int newBoxMinY=std::max(0,nodeBoxMinY-seSize);
	int newBoxMaxY=std::min(nodeBoxMaxY+seSize,gridSizes[1]);
	int newBoxMinZ=std::max(0,nodeBoxMinZ-seSize);
	int newBoxMaxZ=std::min(nodeBoxMaxZ+seSize,gridSizes[2]);


   //the region of checked voxel box
   int voxelMinX=voxelp[0];
   int voxelMaxX=voxelp[0]+1;
   int voxelMinY=voxelp[1];
   int voxelMaxY=voxelp[1]+1;
   int voxelMinZ=voxelp[2];
   int voxelMaxZ=voxelp[2]+1;

   //check the overlap
   bool isOverlapX= std::max(newBoxMinX,voxelMinX)<std::min(newBoxMaxX,voxelMaxX);
   bool isOverlapY= std::max(newBoxMinY,voxelMinY)<std::min(newBoxMaxY,voxelMaxY);
   bool isOverlapZ= std::max(newBoxMinZ,voxelMinZ)<std::min(newBoxMaxZ,voxelMaxZ);

return isOverlapX && isOverlapY && isOverlapZ;

}

/*
Grid3D spaciallyErosion(Grid3D& dilation,Grid3D& grid, MipmapTree mipmap, int seScale){

Grid3D result = dilation; 
    int maxLevel = mipmap.size() - 1; 
    int gridSizeX = grid.size();
    int gridSizeY = grid[0].size();
    int gridSizeZ = grid[0][0].size();
	std::vector<std::array<int,3>> originalV;  //save all the positions that need to be checked

	for(int x=0;x<gridSizeX;++x){
		for(int y=0;y<gridSizeY;++y){
			for(int z=0;z<gridSizeZ;++z){
				if(result[x][y][z]==true && grid[x][y][z]==false){
					originalV.push_back({x,y,z});
				}
			}
		}
	}

	struct Pair{
	int level;
	std::array<int,3> position;
   };

   for(auto ov: originalV){
	//	 std::cout<<"processing target voxel: ("<<ov[0]<<","<<ov[1]<<","<<ov[2]<<")"<<std::endl;

	std::vector<Pair> stack;
      stack.push_back({maxLevel, {0,0,0}});

	  while(!stack.empty() && result[ov[0]][ov[1]][ov[2]]==true){
		Pair top=stack.back();
		stack.pop_back();

		if(top.level==0){
			result[ov[0]][ov[1]][ov[2]]=false;
		}
		else{
			for(int dx=0;dx<=1;++dx){
				for(int dy=0;dy<=1;++dy){
					for(int dz=0;dz<=1;++dz){
						std::array<int,3> subNode={
							top.position[0]*2+dx,
							top.position[1]*2+dy,
							top.position[2]*2+dz
						};
     //		std::cout<<"checking the subnode :("<<subNode[0]<<","<<subNode[1]<<","<<subNode[2]<<")"<<"in level"<<top.level-1<<std::endl;

						if(mipmap[top.level - 1].count(subNode) &&
                         mipmap[top.level - 1].at(subNode)){
	//			std::cout<<"the subnode in the level"<<top.level-1<<" is occupied"<<std::endl;

							if(check_intersect_erosionE(subNode,top.level-1,ov,seScale,{gridSizeX,gridSizeY,gridSizeZ})){
							//				   std::cout<<"the subNode is valid"<<std::endl;

								stack.push_back({top.level-1,subNode});
							}
						}
						
					}
				}
			}
		}
	  }
   }
   return result;

}
*/


//test erosion
struct Pair {
    int level;
    std::array<int, 3> position;
};

Grid3D spaciallyErosionT(Grid3D& dilation, Grid3D& grid, MipmapTree& mipmap, int seScale,std::vector<std::array<int,3>>& offsets) {
    Grid3D result = dilation;
    int maxLevel = mipmap.size() - 1;
    int gridSizeX = grid.size();
    int gridSizeY = grid[0].size();
    int gridSizeZ = grid[0][0].size();
    std::vector<std::array<int, 3>> originalV;

    // 并行化体素遍历，找出需要处理的体素
    #pragma omp parallel
    {
        std::vector<std::array<int, 3>> local_originalV;
        #pragma omp for collapse(3) nowait
        for (int x = 0; x < gridSizeX; ++x) {
            for (int y = 0; y < gridSizeY; ++y) {
                for (int z = 0; z < gridSizeZ; ++z) {
                    if (result[x][y][z] == true && grid[x][y][z] == false) {
                        local_originalV.push_back({x, y, z});
                    }
                }
            }
        }
        #pragma omp critical
        originalV.insert(originalV.end(), local_originalV.begin(), local_originalV.end());
    }

    // 并行处理 originalV 体素
    #pragma omp parallel for schedule(dynamic)
    for (size_t i = 0; i < originalV.size(); ++i) {
        auto ov = originalV[i];
        std::deque<Pair> stack;
        stack.push_back({maxLevel, {0, 0, 0}});

        while (!stack.empty() && result[ov[0]][ov[1]][ov[2]] == true) {
            Pair top = stack.back();
            stack.pop_back();

            if (top.level == 0) {
                result[ov[0]][ov[1]][ov[2]] = false;
            } else {

                for (int dx = 0; dx <= 1; ++dx) {
                    for (int dy = 0; dy <= 1; ++dy) {
                        for (int dz = 0; dz <= 1; ++dz) {
                            std::array<int, 3> subNode = {
                                top.position[0] * 2 + dx,
                                top.position[1] * 2 + dy,
                                top.position[2] * 2 + dz
                            };

                            if (mipmap[top.level - 1].count(subNode) &&
                            mipmap[top.level - 1].at(subNode) &&
                            check_intersect_erosionE(subNode, top.level - 1, ov, seScale, {gridSizeX, gridSizeY, gridSizeZ}, offsets)) {
                            stack.push_back({top.level - 1, subNode});
                        }
                        }
                    }
                }
            }
        }
    }

    return result;
}

//meshing

void convert_to_triangulated_polyhedron(const Nef_polyhedron& nef, Exact_Polyhedron& polyhedron) {
    
    nef.convert_to_polyhedron(polyhedron);

    PMP::triangulate_faces(polyhedron);

    
    if (!CGAL::is_triangle_mesh(polyhedron)) {
        std::cerr << "Error: Resulting mesh is not a triangulated surface!" << std::endl;
    }
}



void meshing(Grid3D& grid, std::array<ExactVector, 3> voxel_strides, std::array<unsigned int, 3>& numVoxels, 
                       std::array<ExactPoint, 8>& obb_points,const std::string outputfile){

	std::vector<unsigned int> activated_voxels;
     
    Nef_polyhedron nef_result;

	for(int x=0; x<numVoxels[0]; ++x){
		for(int y=0;y<numVoxels[1]; ++y){
			for(int z=0;z<numVoxels[2]; ++z){
				if(grid[x][y][z]==true){

              //calculate the index of the activated voxel
			  int index=x* (numVoxels[1] * numVoxels[2]) + y * numVoxels[2] + z;
			  activated_voxels.push_back(index);

					
				}
				


			}
		}
	}

      std::vector<Nef_polyhedron> nef_voxels;

   for(auto voxel_idx : activated_voxels){
	    Exact_Polyhedron voxel;

        calc_voxel_from_idx_hex(voxel_idx,numVoxels,obb_points[7],voxel_strides,voxel);
		 if (!CGAL::is_valid_polygon_mesh(voxel)) {
            std::cerr << "Invalid voxel detected, skipping index: " << voxel_idx << std::endl;
            continue;
        }

		Nef_polyhedron nef_voxel(voxel);
		nef_voxels.push_back(nef_voxel);
		
		/*
		if(nef_result.is_empty()){
			nef_result=nef_voxel;
		}
		else{
			nef_result=nef_result.join(nef_voxel);
		}

		*/
   }

    while (nef_voxels.size() > 1) {
        std::vector<Nef_polyhedron> next_level;
        for (size_t i = 0; i < nef_voxels.size(); i += 2) {
            if (i + 1 < nef_voxels.size()) {
                try {
                    next_level.push_back(nef_voxels[i].join(nef_voxels[i + 1]));
                } catch (const CGAL::Precondition_exception& e) {
                    std::cerr << "CGAL Precondition violation: " << e.what() << std::endl;
                }
            } else {
                next_level.push_back(nef_voxels[i]);
            }
        }
        nef_voxels.swap(next_level);
    }

    if (nef_voxels.empty()) {
        std::cerr << "All voxels failed to merge!" << std::endl;
        return;
    }

     nef_result = nef_voxels[0];

if(nef_result.is_empty()){
	std::cout<<"the nrf_result is empty!"<<std::endl;
	return;
}


   Exact_Polyhedron result;
//convert_to_triangulated_polyhedron(nef_result,result);
nef_result.convert_to_polyhedron(result);
   //output as off file
   std::ofstream output(outputfile);
   if(!output){
	std::cerr<<"Error: Cannot open output file"<<outputfile<<std::endl;
	return;
   }

   output<<result;
   output.close();

   std::cout<<"meshing is done successfully!"<<std::endl;



}






const std::array<std::array<int, 3>, 6> neighbors = {{
    {{-1, 0, 0}}, {{1, 0, 0}},  // X 方向
    {{0, -1, 0}}, {{0, 1, 0}},  // Y 方向
    {{0, 0, -1}}, {{0, 0, 1}}   // Z 方向
}};

std::array<ExactPoint, 8> compute_voxel_vertices(
    const ExactPoint& origin,
    const std::array<ExactVector, 3>& voxel_strides)
{
    return {origin,
            origin + voxel_strides[0],
            origin + voxel_strides[1],
            origin + voxel_strides[2],
            origin + voxel_strides[0] + voxel_strides[1],
            origin + voxel_strides[0] + voxel_strides[2],
            origin + voxel_strides[1] + voxel_strides[2],
            origin + voxel_strides[0] + voxel_strides[1] + voxel_strides[2]};
}

void add_voxel_faces(
    Exact_Polyhedron& mesh,
    const std::array<ExactPoint, 8>& vertices,
    const std::array<bool, 6>& exposed_faces)
{
  static const std::array<std::array<int, 4>, 6> faces = {{
        {{0, 3, 6, 2}},  // -X
        {{1, 4, 7, 5}},  // +X
        {{1, 5, 3, 0}},  // -Y
        {{2, 6, 7, 4}},  // +Y
        {{1, 0, 2, 4}},  // -Z
        {{3, 5, 7, 6}}   // +Z
    }};

    for (int i = 0; i < 6; ++i) {
        if (exposed_faces[i]) {
            mesh.make_triangle(vertices[faces[i][0]], vertices[faces[i][1]], vertices[faces[i][2]]);
            mesh.make_triangle(vertices[faces[i][2]], vertices[faces[i][3]], vertices[faces[i][0]]);
        }
    }
}

void extract_surface_from_voxels(
    const Grid3D& grid,
    const std::array<ExactVector, 3>& voxel_strides,
    const ExactPoint& origin,
    const std::string outputfile)
{
	Exact_Polyhedron output_mesh;
    size_t nx = grid.size(), ny = grid[0].size(), nz = grid[0][0].size();
    for (size_t x = 0; x < nx; ++x) {
        for (size_t y = 0; y < ny; ++y) {
            for (size_t z = 0; z < nz; ++z) {
                if (!grid[x][y][z]) continue;  // 仅处理被占据的体素

                std::array<ExactPoint, 8> voxel_vertices =
                    compute_voxel_vertices(origin + x * voxel_strides[0] + y * voxel_strides[1] + z * voxel_strides[2], voxel_strides);

                std::array<bool, 6> exposed_faces = {true, true, true, true, true, true};
                for (int i = 0; i < 6; ++i) {
                    int nx = x + neighbors[i][0];
                    int ny = y + neighbors[i][1];
                    int nz = z + neighbors[i][2];
                    if (nx >= 0 && nx < grid.size() &&
                        ny >= 0 && ny < grid[0].size() &&
                        nz >= 0 && nz < grid[0][0].size() &&
                        grid[nx][ny][nz]) {
                        exposed_faces[i] = false;
                    }
                }

                add_voxel_faces(output_mesh, voxel_vertices, exposed_faces);
            }
        }
    }

if (!CGAL::is_closed(output_mesh)) {
    std::cerr << "Warning: Mesh is not closed! Trying to close it..." << std::endl;

    PMP::stitch_borders(output_mesh);
    PMP::triangulate_faces(output_mesh);
    
    if (CGAL::is_closed(output_mesh)) {
        std::cout << "Mesh is now closed after stitching!" << std::endl;
    } else {
        std::cerr << "Error: Mesh is still not closed!" << std::endl;
    }
}

std::ofstream output(outputfile);
 if(!output){
	std::cerr<<"Error: Cannot open output file"<<outputfile<<std::endl;
	return;
   }

   output<<output_mesh;
   output.close();

   std::cout<<"meshing is done successfully!"<<std::endl;


}





struct Constrained_edge_map {
    typedef boost::readable_property_map_tag category;
    typedef bool value_type;
    typedef bool reference;
    typedef sedge_descriptor key_type;

    Constrained_edge_map(const CGAL::Unique_hash_map<key_type, bool>& constraints) 
        : mConstraints(constraints) {}

    value_type operator[](const key_type& e) const { return is_constrained(e); }
    friend inline value_type get(const Constrained_edge_map& m, const key_type& k) { return m[k]; }
    bool is_constrained(const key_type& e) const { return mConstraints.is_defined(e); }

private:
    const CGAL::Unique_hash_map<key_type, bool>& mConstraints;
};


class ErrorDrivenStopPredicate {
private:
    double max_geometric_error;

public:
    ErrorDrivenStopPredicate(double max_error) : max_geometric_error(max_error) {}

    template <typename Cost, typename Profile>
    bool operator()(const Cost& cost, const Profile&, size_t, size_t) const {
        return cost > max_geometric_error;
    }
};


void mesh_simplifyB(const std::string& inputfile, double scale,double threshold, const std::string outputfile){

	Surface_mesh mesh;
	std::ifstream input(inputfile);
	if(!input || ! (input >>mesh)){
		std::cerr<<"Failed to open input mesh"<<inputfile<<std::endl;
		return;
	}




 typedef typename QEM_Policies::Get_cost                                QEM_cost;                              
  typedef typename QEM_Policies::Get_placement                          QEM_placement;
  typedef SMS::Bounded_normal_change_placement<QEM_placement>                   Bounded_QEM_placement;


SMS::Edge_length_stop_predicate<double>  edge_length_predicate(scale);

ErrorDrivenStopPredicate error_stop(threshold);


  CGAL::Unique_hash_map<sedge_descriptor, bool> constraint_hmap(false);
    Constrained_edge_map constraints_map(constraint_hmap);

    std::map<sedge_descriptor, std::pair<Surface_Point, Surface_Point>> constrained_edges;
    std::size_t nb_sharp_edges = 0;

    for (sedge_descriptor ed : edges(mesh)) {
        shalfedge_descriptor hd = halfedge(ed, mesh);
        
        // ** 计算二面角 **
        double angle = CGAL::approximate_dihedral_angle(
            get(CGAL::vertex_point, mesh, target(opposite(hd, mesh), mesh)),
            get(CGAL::vertex_point, mesh, target(hd, mesh)),
            get(CGAL::vertex_point, mesh, target(next(hd, mesh), mesh)),
            get(CGAL::vertex_point, mesh, target(next(opposite(hd, mesh), mesh), mesh))
        );

        // ** 低于角度阈值的边设为不可删除 **
        if (CGAL::abs(angle) < 90) {
            ++nb_sharp_edges;
            constraint_hmap[ed] = true;
            constrained_edges[ed] = std::make_pair(
                get(CGAL::vertex_point, mesh, source(hd, mesh)), 
                get(CGAL::vertex_point, mesh, target(hd, mesh))
            );
        }
    }
    std::cerr << "# sharp edges = " << nb_sharp_edges << std::endl;


QEM_Policies qem_policies(mesh);




const QEM_cost& qem_cost = qem_policies.get_cost();
  const QEM_placement& qem_placement = qem_policies.get_placement();
      
  Bounded_QEM_placement placement(qem_placement);
  

std::size_t removed_edges=SMS::edge_collapse(mesh,
   [&](const auto& cost, const auto& profile, size_t initial_edge_count, size_t current_edge_count) {
            return edge_length_predicate(cost, profile, initial_edge_count, current_edge_count)
			&&
               error_stop(cost, profile, initial_edge_count, current_edge_count);
        },
           /*  CGAL::parameters::get_cost(qem_cost)
                                              .get_placement(placement)   */
			 CGAL::parameters::vertex_index_map(get(CGAL::vertex_external_index, mesh))
        .halfedge_index_map(get(CGAL::halfedge_external_index, mesh))
		.edge_is_constrained_map(constraints_map)
        .get_cost(qem_policies.get_cost())
       // .get_placement(qem_policies.get_placement())								  
        .get_placement(placement)								  
											  
											  );

std::cout<<"simplification completed with removed"<<removed_edges<<"edges"<<std::endl;

if (!PMP::is_outward_oriented(mesh)) {
    PMP::reverse_face_orientations(mesh);
    std::cout << "Fixed flipped face orientations!" << std::endl;
}

std::ofstream output(outputfile);
output<<mesh;
output.close();






}





void mesh_simplifyA(const std::string& inputfile, double scale,double threshold, const std::string outputfile){

	Surface_mesh mesh;
	std::ifstream input(inputfile);
	if(!input || ! (input >>mesh)){
		std::cerr<<"Failed to open input mesh"<<inputfile<<std::endl;
		return;
	}

 typedef typename QEM_Policies::Get_cost                                QEM_cost;                              
  typedef typename QEM_Policies::Get_placement                          QEM_placement;
  typedef SMS::Bounded_normal_change_placement<QEM_placement>                   Bounded_QEM_placement;


SMS::Edge_length_stop_predicate<double>  edge_length_predicate(scale);

ErrorDrivenStopPredicate error_stop(threshold);





QEM_Policies qem_policies(mesh);
const QEM_cost& qem_cost = qem_policies.get_cost();
  const QEM_placement& qem_placement = qem_policies.get_placement();
  Bounded_QEM_placement placement(qem_placement);

std::size_t removed_edges=SMS::edge_collapse(mesh,
  [&](const auto& cost, const auto& profile, size_t initial_edge_count, size_t current_edge_count) {
           // return edge_length_predicate(cost, profile, initial_edge_count, current_edge_count)
			//&&
            return   error_stop(cost, profile, initial_edge_count, current_edge_count);
        },
           /*  CGAL::parameters::get_cost(qem_cost)
                                              .get_placement(placement)   */
			 CGAL::parameters::vertex_index_map(get(CGAL::vertex_external_index, mesh))
        .halfedge_index_map(get(CGAL::halfedge_external_index, mesh))
        .get_cost(qem_policies.get_cost())
        .get_placement(qem_policies.get_placement())								  
											  
											  );

std::cout<<"simplification completed with removed"<<removed_edges<<"edges"<<std::endl;

std::ofstream output(outputfile);
output<<mesh;
output.close();



}





void voxelization(const std::string& inputfile,
                  Grid3D& grid,
                  const std::string& outputfile,
                  std::array<unsigned int, 3>& numVoxels ,
                  std::array<ExactPoint, 8>& obb_points,
                  std::array<ExactVector, 3>& voxel_strides){
std::cout << "Loading surface\n";
    Exact_Polyhedron poly;
    if (!PMP::IO::read_polygon_mesh(inputfile, poly) || !CGAL::is_triangle_mesh(poly))
    {
        std::cerr << "Invalid input.\n";
        return ;
    }
    Mesh surface;
    if (!CGAL::Polygon_mesh_processing::IO::read_polygon_mesh(inputfile, surface) || surface.is_empty())
    {
        std::cerr << "Invalid input file.\n";
        return ;
    }
 
   std::cout << "Compute OBB\n";
    std::array<InexactPoint, 8> obb_points_inexact;
    CGAL::oriented_bounding_box(surface, obb_points_inexact, CGAL::parameters::use_convex_hull(false));

    // std::array<ExactPoint, 8> obb_points;

    for (unsigned i = 0; i < 8u; ++i)
    {
        obb_points[i] = ExactPoint(obb_points_inexact[i].x(), obb_points_inexact[i].y(), obb_points_inexact[i].z());
    }

   //voxel number
   // std::array<unsigned int, 3> numVoxels = { 16u, 16u, 16u };

    //step length of voxels
    voxel_strides = { (obb_points[0] - obb_points[1]) / static_cast<double>(numVoxels[0]),
        (obb_points[6] - obb_points[7]) / static_cast<double>(numVoxels[1]), (obb_points[1] - obb_points[6]) / static_cast<double>(numVoxels[2]) };

  std::cout<<"OBB is done"<<std::endl;

  std::vector<unsigned int> intersecting_voxels;

    
    auto numVoxel = numVoxels[0] * numVoxels[1] * numVoxels[2];
    bool interior = false;

    std::vector<uint8_t> voxels_marking(numVoxel, 0); // either outside 0, surface 1 or interior 2
    Tree tree(faces(poly).first, faces(poly).second, poly);
	tree.accelerate_distance_queries();

    Point_inside inside_tester(tree);
  intersecting_voxels.reserve(numVoxel / 10);   //pre-allocation
    // grid=initializeGrid(numVoxels[0],numVoxels[1],numVoxels[2]);

     #pragma omp parallel for schedule(dynamic)
     for (unsigned int i = 0; i < numVoxel; ++i){
        Exact_Polyhedron voxel = Exact_Polyhedron();
        bool new_scanline;

        calc_voxel_from_idx_tets(i, numVoxels, obb_points[7], voxel_strides, voxel, &new_scanline);
      
       //check if inside
       bool inside = true;

        for (auto vert : voxel.vertex_handles())
        {
            if (inside_tester(vert->point()) != CGAL::ON_BOUNDED_SIDE)
            {
                inside = false;
                break;
            }
        }
     CGAL::Iso_cuboid_3<Exact_Kernel> voxel_bbox = CGAL::bounding_box(voxel.points().begin(), voxel.points().end());
     bool intersects = tree.any_intersected_primitive(voxel_bbox).has_value();  

      if (intersects|| inside){
            #pragma omp critical
            {
            grid[i/(numVoxels[1]*numVoxels[2])][(i/numVoxels[2])%numVoxels[1]][i%numVoxels[2]]=true;
            intersecting_voxels.push_back(i);
            voxels_marking[i]=inside ? 2 : 1;
           }
      }


     }
	 std::cout<<"parallel part is done"<<std::endl;

/*
     Exact_Polyhedron voxels;
    for (auto voxel_idx : intersecting_voxels)
    {
        if (voxels_marking[voxel_idx] == 2)
        {
            calc_voxel_from_idx_hex(voxel_idx, numVoxels, obb_points[7], voxel_strides, voxels);
			
        }
    }

	


    CGAL::IO::write_OFF(outputfile, voxels);
*/




}



bool convert_off_to_obj(const std::string& off_filename,const std::string& obj_filename){
	//create a exactmesh
	ExactMesh mesh;

struct Point3D{
	double x, y, z;
};

struct FaceforConversion{
	std::vector<size_t> vertices;
};


	//open the off file
	std::ifstream off_input(off_filename);
	if (!off_input){
		std::cerr<<"Error: can not open the off file:"<<off_filename<<std::endl;
		return false;
	}

	//check is the off file valid
	std::string header;
	off_input>>header;
	if(header!="OFF"){
		std::cerr<<"Error: file is not a valid off file!"<<std::endl;
		return false;
	}
    
	//read the off file
	size_t num_vertices=0, num_faces=0, num_edges=0;
     off_input>>num_vertices>>num_faces>>num_edges;

	 std::vector<Point3D>vertices(num_vertices);
	 for(size_t i=0 ; i<num_vertices ; ++i){
		off_input>>vertices[i].x>>vertices[i].y>>vertices[i].z;
	 }

	 std::vector<FaceforConversion>faces(num_faces);
	 for(size_t i=0 ; i<num_faces ; ++i){
		size_t num_vertices_in_face=0;
		off_input>>num_vertices_in_face;
		faces[i].vertices.resize(num_vertices_in_face);
		for(size_t j=0 ; j<num_vertices_in_face ; ++j){
			off_input>>faces[i].vertices[j];
		}
	 }

	 off_input.close();

  std::ofstream obj_output(obj_filename);
  if(!obj_output.is_open()){
	std::cerr<<"Error: could not open obj file!"<<std::endl;
    return false;
  }

	for(const auto& vertex: vertices){
		obj_output<<"v "<<vertex.x<<" "<<vertex.y<<" "<<vertex.z<<"\n";
	}

	for(const auto& face:faces){
		obj_output<<"f";
		for(const auto& vertex_index: face.vertices){
			obj_output<<" "<<(vertex_index+1);
		}
		obj_output<<"\n";
	}
	obj_output.close();

	return true;

	
}

void GenerateCageFromMeshOperation::Execute(){

const std::string filepath="/Users/liujiaqi/Desktop/opencv_Lab/CageModeler_1/models/";
const std::string filename=_params._meshfilepath;
const std::string outputfilename=_params._cagefilepath;

 //voxel number
    std::array<unsigned int, 3> numVoxels = { 32u, 32u, 32u };
 Grid3D fineGrid=initializeGrid(numVoxels[0],numVoxels[1],numVoxels[2]);
 std::array<ExactPoint, 8> obb_points;
std::array<ExactVector, 3> voxel_strides;

 const std::string off_filename=filepath+"voxelization.off";
    const std::string dilation_off_filename=filepath+"dilation.off";
	const std::string erosion_off_filename=filepath+"erosion.off";
	

    const std::string dilation_obj_sphere_filename=filepath+"dialation.obj";
    const std::string dilation_obj_cube_filename=filepath+"dialation_cube.obj";

	const std::string grid_filename=filepath+"grid.obj";
	const std::string halfcontour_obj_filename=filepath+"halfcontour.obj";
    const std::string erosion_obj_filename=filepath+"erosion.obj";
	const std::string meshing_off_filename=filepath+"meshing.off";
	 const std::string off_meshing_simplified=filepath+"meshing_simp_2.off";
	 const std::string off_cage=filepath+"outputCage_1.off";

   voxelization(filename,fineGrid,off_filename,numVoxels,obb_points,voxel_strides);
   		std::cout<<"voxelization is done!"<<std::endl;

	int gridSizeX,gridSizeY,gridSizeZ;
	const int hierarchyLevels=6;
	const int se_scale=_params._scale;
	double threshold_asymatric = 0.001;
  	double threshold_symatric = 1000;
   	double threshold =  std::sqrt(CGAL::to_double((voxel_strides[0] + voxel_strides[1] + voxel_strides[2]).squared_length()));
    std::vector<std::array<int,3>> offsets=generate_sphere_offsets(se_scale);

		//wirte the 3D grid into the obj file
     writeGridIntoObj(fineGrid,grid_filename);

	  //build the mipmaptree for dilation
	   MipmapTree mipmap_dilate = buildHeirarchy(fineGrid,hierarchyLevels);

      // std::cout<<"the mipmap for dilation is:"<<std::endl;
      // printMipmap(mipmap_dilate);

	   //do the dilation
	    Grid3D dilation=parallelDilationA(fineGrid,mipmap_dilate,se_scale);
		std::cout<<"dilation is done!"<<std::endl;
		//write the dilation (grid)into obj file
		//writeGridIntoObj(dilation,dilation_obj_sphere_filename);
	   writeGridIntoObj(dilation,dilation_obj_cube_filename);

    
		//write the dilated object into off file
	//	grid_into_voxelized_off(dilation,dilation_off_filename,numVoxels,obb_points,voxel_strides);

      
	    //extract the contour of the dilation
		Grid3D contour=extract_contour(dilation);

		//cut the contour into half to see if the extraction works
       // Grid3D half=cut_the_half(contour);
		

		//write the half contour (grid)into obj file
		//writeGridIntoObj(half,halfcontour_obj_filename);

		//build the mipmap for eroion
		MipmapTree mipmap_erose=buildHeirarchy(contour,hierarchyLevels);

		// std::cout<<"the mipmap for erosion is:"<<std::endl;
      // printMipmap(mipmap_erose);

		//do the erosion
		std::time_t now;
		 time(&now);
		std::cout<<"the time:"<<std::ctime(&now)<<" start erosion..."<<std::endl;

		Grid3D erosion=spaciallyErosionT(dilation,fineGrid,mipmap_erose,se_scale,offsets);
				std::cout<<"erosion is done!"<<std::endl;

		//write the erosion(grid) into obj file
		writeGridIntoObj(erosion,erosion_obj_filename);
        
	//	std::cout<<"generating the result off file after closing..."<<std::endl;
       //write the erosed object into off file
	//	grid_into_voxelized_off(erosion,erosion_off_filename,numVoxels,obb_points,voxel_strides);

		//do the meshing
		//create a coarse grid C from erosion
		//meshing(erosion,voxel_strides,numVoxels,obb_points,meshing_off_filename);
		std::cout<<"doing the meshing..."<<std::endl;
		extract_surface_from_voxels(erosion,voxel_strides,obb_points[7],meshing_off_filename);
        mesh_simplifyB(meshing_off_filename,threshold,threshold_asymatric,off_meshing_simplified);


		std::cout<<"the output path is:"<<outputfilename<<std::endl;
        if(convert_off_to_obj(off_meshing_simplified,outputfilename)){
			LOG_DEBUG("Done Cage Generation.");

			
		}
		else{
			LOG_ERROR("Failed to generate the cage!.");
		}
	  

}