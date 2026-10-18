#define _USE_MATH_DEFINES
#include <cmath>

#include <cagedeformations/MaximumLikelihoodCoordinates.h>

#include <iostream>
#include <vector>
#include <Eigen/Geometry>
#include <igl/vertex_triangle_adjacency.h>

// #include <iomanip>
// #include <igl/Timer.h>

// #include <nlopt.h>

// #include <fstream>

void computeIntegralUnitNormals(const Eigen::MatrixXd &v, const Eigen::MatrixXi &f, Eigen::MatrixXd &transMatrix, Eigen::MatrixXd &normalized_integral_outward_allfaces)
{
    normalized_integral_outward_allfaces.resize(3, f.cols());
    Eigen::Matrix3d T, norm_;
    Eigen::Vector3d cross_0, cross_1, cross_2, length_c, beta;
    double length_of_integral_outward_allfaces;
    for (int i = 0; i < f.cols(); ++i)
    {
        T.col(0) = v.col(f(0, i));
        T.col(1) = v.col(f(1, i));
        T.col(2) = v.col(f(2, i));

        norm_.col(0) = T.col(0).cross(T.col(1));
        norm_.col(0) = norm_.col(0).normalized();
        norm_.col(1) = T.col(1).cross(T.col(2));
        norm_.col(1) = norm_.col(1).normalized();
        norm_.col(2) = T.col(2).cross(T.col(0));
        norm_.col(2) = norm_.col(2).normalized();

        beta[0] = acos(T.col(0).dot(T.col(1)));
        beta[1] = acos(T.col(1).dot(T.col(2)));
        beta[2] = acos(T.col(2).dot(T.col(0)));

        normalized_integral_outward_allfaces.col(i) = norm_ * beta / 2.0;
        length_of_integral_outward_allfaces = normalized_integral_outward_allfaces.col(i).norm();
        normalized_integral_outward_allfaces.col(i) /= length_of_integral_outward_allfaces;

        for (int j = 0; j < 3; ++j)
        {
            transMatrix(j, i) =
                (beta[(j + 1) % 3] + beta[j] * norm_.col(j).dot(norm_.col((j + 1) % 3)) +
                 beta[(j + 2) % 3] * norm_.col((j + 2) % 3).dot(norm_.col((j + 1) % 3))) /
                (2 * T.col(j).dot(norm_.col((j + 1) % 3)));
        }
        transMatrix.col(i) /= length_of_integral_outward_allfaces;
        if (T.determinant() < 0)
        {
            normalized_integral_outward_allfaces.col(i) *= -1;
            transMatrix.col(i) *= -1;
        }
    }
}

void calculateMaximumLikelihoodCoordinates(const Eigen::MatrixXd &cage_v, const Eigen::MatrixXi &cage_f, const Eigen::MatrixXd &model_v, Eigen::MatrixXd &mlc)
{
    int nv = cage_v.cols();
    int nf = cage_f.cols();

    mlc.resize(nv, model_v.cols());

    std::vector<std::vector<int>> allAdjFaces;
    std::vector<int> adjFaces;

    std::vector< std::vector< int > > VF;
    //  std::vector< std::vector< int > > VFi; //allAdjFaces
    igl::vertex_triangle_adjacency(nv, cage_f, VF, allAdjFaces);
    // get adjacent faces of each vertex
    // for (int i = 0; i < nv; ++i)
    // {
    //     adjFaces = findAdjacentFaces(cage_f, i);
    //     allAdjFaces.push_back(adjFaces);
    // }

    for (int ii = 0; ii < model_v.cols(); ++ii)
    {
        // Eigen::VectorXd mlc = Eigen::VectorXd::Zero(nv);
        std::vector<Eigen::MatrixXd> transMatrixGroup;
        Eigen::MatrixXd transMatrix = Eigen::MatrixXd::Identity(nv, nv);

        // 1. get projection vertices on unit sphere
        Eigen::MatrixXd v;
        v.resize(3, nv);
        // timer->start();
        v = cage_v.colwise() - model_v.col(ii);
        transMatrix.diagonal()  = (1.0 / v.colwise().norm().array()).matrix();
        // timer->stop();
        // std::cout << " took " << timer->getElapsedTime() << "seconds\n";
        // Eigen::MatrixXd v1 = v;
        transMatrixGroup.push_back(transMatrix);
        v *= transMatrix;
        // std::cout << v.transpose() << std::endl;

        // 2. smooth
        Eigen::MatrixXd tempVar;
        double integral_vector_length;

        tempVar = Eigen::MatrixXd::Zero(3, nf);
        transMatrix = Eigen::MatrixXd::Zero(nv, nv);

        Eigen::MatrixXd normalized_integral_outward_allfaces;
        computeIntegralUnitNormals(v, cage_f, tempVar, normalized_integral_outward_allfaces);
        // Eigen::MatrixXd v2 = v;
        v = Eigen::MatrixXd::Zero(3, nv);

        for (int i = 0; i < nv; ++i)
        {
            for (std::vector<int>::iterator iter = allAdjFaces.at(i).begin();
                 iter != allAdjFaces.at(i).end(); ++iter)
            {
                v.col(i) += normalized_integral_outward_allfaces.col(*iter);

                transMatrix(cage_f(0, *iter), i) += tempVar(0, *iter);
                transMatrix(cage_f(1, *iter), i) += tempVar(1, *iter);
                transMatrix(cage_f(2, *iter), i) += tempVar(2, *iter);
            }
            integral_vector_length = v.col(i).norm();
            transMatrix.col(i) /= integral_vector_length;
            v.col(i) /= integral_vector_length;
        }

        transMatrixGroup.push_back(transMatrix);

        // std::cout << v2*transMatrix -v << std::endl;

        Eigen::Vector3d x(0, 0, 0);
        // double minf;
        // nlopt_opt opt;
        // opt = nlopt_create(NLOPT_LD_TNEWTON, 3);  // Choose an optimization algorithm
        // // std::cout << std::fixed << std::setprecision(5) << v << std::endl;
        // // std::fstream os("d:/a.txt");
        // // os << v;
        // // os.close();
        // nlopt_set_min_objective(opt, f, &v);  // Set the objective function
        // nlopt_set_xtol_rel(opt, 1e-10);  // Set optimization options if needed
        // if (nlopt_optimize(opt, x.data(), &minf) < 0) {
        // std::cerr << "NLopt optimization failed!" << std::endl;
        // } 
        
        // nlopt_destroy(opt);



        
        // damp Newton
        
        Eigen::Vector3d g, d, g_old(0.0,0.0,0.0);
        Eigen::Matrix3d H;
        double error = 1e-10;
        double rho = .7;
        double sigma = .3;

        

        while (true)
        {
            double step_length = 1.0;
            g = f_gradient(x, v);
            
            if (std::isnan(g.norm())) 
            {
                std::cerr<< "There is numerical instability in the process of solving optimization problems. (please check the " << ii << "-th point)" << std::endl;
                assert(false);
            }

            if (g.norm() < error || (g_old-g).norm() < error*error)
            {
                break;
            }
            g_old = g;
            H = f_Hessian(x, v);
            // d = -H.inverse() * g;
            d = -H.colPivHouseholderQr().solve(g);
            if (g.norm() > 1e-4){
                // Armijo linear search
                while (step_length > error)
                {
                    if (f(x + step_length * d, v) < f(x, v) + sigma * step_length * g.dot(d))
                    {
                        break;
                    }
                    step_length *= rho;
                }
            }
            // cout << pow(rho, mk) << endl;
            x += step_length * d;
        }
        

        for (int i = 0; i < nv; ++i)
        {
            mlc(i, ii) = 1 / (nv + x[0] * v(0, i) + x[1] * v(1, i) + x[2] * v(2, i));
        }
        // std::cout << ii << "-------------------------------------------------------------:" << (v * mlc.col(ii)).norm() << std::endl; // for test
        // std::cout << mlc.col(ii).sum()<<std::endl;
        // std::cout << mlc << std::endl;
        while (!transMatrixGroup.empty())
        {
            mlc.col(ii) = transMatrixGroup.back() * mlc.col(ii);
            transMatrixGroup.pop_back();
        }
        // std::cout << ii << "--------------------------------------------:" << (v1 * mlc.col(ii)).norm() << std::endl; // for test
        // std::cout << mlc.col(ii).sum() << std::endl;

        mlc.col(ii) /= mlc.col(ii).sum();
        // std::cout << mlc.col(ii).sum() << std::endl;
        // std::cout << ii << "--------------------------------------------:" << (v1 * mlc.col(ii)).norm() << std::endl; // for test
        // std::cout << ii << "-------------------------------------------------------------:" << ((cage_v.colwise() - model_v.col(ii))*mlc.col(ii)).norm() << std::endl; // for test
        if ((cage_v * mlc.col(ii) - model_v.col(ii)).norm()>1e-7)
            std::cerr<< "---------------- The maximum likelihood coordinates of the " << ii << "-th point may be wrong, as c*lambda-v=" << (cage_v * mlc.col(ii) - model_v.col(ii)).norm() << " > 1e-7" << std::endl;
    }
}

// std::vector<int> findAdjacentFaces(const Eigen::MatrixXi &faces, int indexOfVertex)
// {
//     std::vector<int> adjacentFaces;
//     for (int i = 0; i < faces.cols(); ++i)
//     {
//         for (int j = 0; j < 3; ++j)
//         {
//             if (faces(j, i) == indexOfVertex)
//             {
//                 adjacentFaces.push_back(i);
//                 break;
//             }
//         }
//     }
//     return adjacentFaces;
// }

Eigen::VectorXd f_gradient(Eigen::VectorXd x, Eigen::MatrixXd v)
{
    // int n = v.cols();
    // Eigen::Vector3d g(0, 0, 0);
    // for (int i = 0; i < n; ++i)
    // {
    //     g -= v.col(i) / (n + x.dot(v.col(i)));
    // }
    return -v * (1 / (((v.transpose()*x).array() + 1.0*v.cols()))).matrix();
}

Eigen::MatrixXd f_Hessian(Eigen::VectorXd x, Eigen::MatrixXd v)
{
    //  std::unique_ptr<igl::Timer> timer;
    // timer = std::make_unique<igl::Timer>();
    // int n = v.cols();

    // timer->start();
    // Eigen::Matrix3d H = Eigen::Matrix3d::Zero();

    // for (int i = 0; i < 3; ++i)
    // {
    //     for (int j = i; j < 3; ++j)
    //     {
    //         for (int k = 0; k < n; ++k)
    //         {
    //             H(i, j) += v(i, k) * v(j, k) / ((n + x.dot(v.col(k))) * (n + x.dot(v.col(k))));
    //         }
    //         H(j, i) = H(i, j);
    //     }
    // }
    // timer->stop();
    // std::cout << " took " << timer->getElapsedTime() << "seconds\n";

    // timer->start();
    Eigen::Matrix3d H = Eigen::Matrix3d::Zero();
    Eigen::MatrixXd denominator  =  (1/((v.transpose()*x).array() + 1.0*v.cols()).array().square());
    for (int i = 0; i < 3; ++i)
    {
        for (int j = i; j < 3; ++j)
        {
            H(i, j) = ((  v.row(i).array() * v.row(j).array() ).matrix() * denominator  ).sum();
            H(j, i) = H(i, j);
        }
    }
    // timer->stop();
    // std::cout << " took " << timer->getElapsedTime() << "seconds\n";
    return H;
}

double f(Eigen::VectorXd x, Eigen::MatrixXd v)
{
    int n = v.cols();
    double z = 0;

    for (int i = 0; i < n; ++i)
    {
        z -= log(n + x.dot(v.col(i)));
    }

    return z;
}


double f(unsigned n, const double *x, double *grad, void *data)
{
    Eigen::MatrixXd* v = (Eigen::MatrixXd *) data;
    Eigen::Map<Eigen::Vector3d> gradient(grad, n);
    Eigen::Map<const Eigen::Vector3d> X(x);
    gradient = - (*v) * (1 / ((((*v).transpose()*X).array() + 1.0*(*v).cols()))).matrix();

    return f(X, *v);
}
