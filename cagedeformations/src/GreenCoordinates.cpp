#define _USE_MATH_DEFINES
#include <cmath>

#include <cagedeformations/GreenCoordinates.h>

#include <iostream>
#include <vector>
#include <Eigen/Geometry>

/// Taken from https://github.com/superboubek/QMVC

template< class point_t >
point_t triMeanVector(point_t const& p, point_t const& p0, point_t const& p1, point_t const& p2)
{
	point_t pttri[3] = { p0,p1,p2 };
	point_t utri[3];
	for (unsigned int i = 0; i < 3; ++i) {
		utri[i] = (pttri[i] - p);
		if (utri[i].stableNorm() < 0.0000000000000001)
		{
			utri[i] = { 0, 0, 0 };
		}
		else
		{
			utri[i] = utri[i].normalized();
		}
		assert(utri[i] == utri[i]);
	}

	double thetatri[3];
	for (unsigned int i = 0; i < 3; ++i) {
		assert(!std::isnan((utri[(i) % 3] - utri[(i + 1) % 3]).norm()));
		thetatri[i] = 2.0 * std::asin(std::min<double>(1.0, std::max<double>(-1.0, (utri[(i) % 3] - utri[(i + 1) % 3]).norm() / 2.0)));
		assert(!std::isnan(thetatri[i]));
	}

	point_t Ntri[3];
	for (unsigned int i = 0; i < 3; ++i) {
		Ntri[i] = (pttri[(i + 1) % 3] - p).cross(pttri[(i) % 3] - p);
		assert(Ntri[i] == Ntri[i]);
	}

	auto Ntri_norm_0 = Ntri[0].stableNorm();
	if (Ntri_norm_0 < 0.0000000000000001)
	{
		Ntri[0] = { 0, 0, 0 };
	}
	else
	{
		Ntri[0] = Ntri[0].normalized();
	}
	auto Ntri_norm_1 = Ntri[1].stableNorm();
	if (Ntri_norm_1 < 0.0000000000000001)
	{
		Ntri[1] = { 0, 0, 0 };
	}
	else
	{
		Ntri[1] = Ntri[1].normalized();
	}
	auto Ntri_norm_2 = Ntri[2].stableNorm();
	if (Ntri_norm_2 < 0.0000000000000001)
	{
		Ntri[2] = { 0, 0, 0 };
	}
	else
	{
		Ntri[2] = Ntri[2].normalized();
	}
	point_t m_tri = -0.5 * (thetatri[0] * Ntri[0] + thetatri[1] * Ntri[1] + thetatri[2] * Ntri[2]);
	return m_tri;
}



template< class point_t >
point_t quadMeanVector(point_t const& p, point_t const& p0, point_t const& p1, point_t const& p2, point_t const& p3)
{
	point_t mv(0, 0, 0);
	point_t t0, t1, t2, pc = (p0 + p1 + p2 + p3) / 4.0;
	t0 = p0; t1 = p1; t2 = pc;
	{
		mv += triMeanVector(p, t0, t1, t2);
	}
	t0 = p1; t1 = p2; t2 = pc;
	{
		mv += triMeanVector(p, t0, t1, t2);
	}
	t0 = p2; t1 = p3; t2 = pc;
	{
		mv += triMeanVector(p, t0, t1, t2);
	}
	t0 = p3; t1 = p0; t2 = pc;
	{
		mv += triMeanVector(p, t0, t1, t2);
	}
	return mv;
}

template< class point_t >
double computeClosestPointOnLineFromLine(point_t const& A, point_t const& B, point_t const& eta, point_t const& d) {
	point_t const& e = B - A;
	double eDotD = e.dot(d);
	double dDotD = d.dot(d);
	double eDotE = e.dot(e);

	double alpha = eDotE - eDotD * eDotD / dDotD;
	double beta = (A - eta).dot(e) - (A - eta).dot(d) * eDotD / dDotD;

	return -beta / alpha;
}

template< class point_t >
void computeClosestPointOnQuadFromLine(point_t const& q0, point_t const& q1, point_t const& q2, point_t const& q3,
	point_t const& origin, point_t const& direction,
	double& uProj, double& vProj, bool ForceIntoUnitSquare = true) {
	double epsilon = 0.0;

	double aV = ((q3 - q0).cross(q2 - q1)).dot(direction);
	double aU = ((q1 - q0).cross(q2 - q3)).dot(direction);

	if (std::fabs(aV) > std::fabs(aU)) {
		// solve for v :
		double a = ((q3 - q0).cross(q2 - q1)).dot(direction);
		double b = ((q0 - origin).cross(q2 - q1) + (q3 - q0).cross(q1 - origin)).dot(direction);
		double c = ((q0 - origin).cross(q1 - origin)).dot(direction);
		if (std::fabs(a) > epsilon) {
			double Delta = b * b - 4 * a * c;
			if (Delta >= 0.0) {
				double vPlus = (-b + std::sqrt(Delta)) / (2 * a);
				double vMinus = (-b - std::sqrt(Delta)) / (2 * a);
				if (std::fabs(vMinus - 0.5) < std::fabs(vPlus - 0.5))
					vProj = vMinus;
				else
					vProj = vPlus;
			}
		}
		else {
			vProj = -c / b;
		}

		if (ForceIntoUnitSquare) vProj = std::min<double>(1.0, std::max<double>(0.0, vProj));

		point_t AV = (1.0 - vProj) * q0 + vProj * q3;
		point_t BV = (1.0 - vProj) * q1 + vProj * q2;

		uProj = computeClosestPointOnLineFromLine(AV, BV, origin, direction);
		if (ForceIntoUnitSquare) uProj = std::min<double>(1.0, std::max<double>(0.0, uProj));
	}
	else {
		// solve for u :
		double a = ((q1 - q0).cross(q2 - q3)).dot(direction);
		double b = ((q0 - origin).cross(q2 - q3) + (q1 - q0).cross(q3 - origin)).dot(direction);
		double c = ((q0 - origin).cross(q3 - origin)).dot(direction);
		if (std::fabs(a) > epsilon) {
			double Delta = b * b - 4 * a * c;
			if (Delta >= 0.0) {
				double uPlus = (-b + std::sqrt(Delta)) / (2 * a);
				double uMinus = (-b - std::sqrt(Delta)) / (2 * a);
				if (std::fabs(uMinus - 0.5) < std::fabs(uPlus - 0.5))
					uProj = uMinus;
				else
					uProj = uPlus;
			}
		}
		else {
			uProj = -c / b;
		}

		if (ForceIntoUnitSquare) uProj = std::min<double>(1.0, std::max<double>(0.0, uProj));

		point_t AU = (1.0 - uProj) * q0 + uProj * q1;
		point_t BU = (1.0 - uProj) * q3 + uProj * q2;

		vProj = computeClosestPointOnLineFromLine(AU, BU, origin, direction);
		if (ForceIntoUnitSquare) vProj = std::min<double>(1.0, std::max<double>(0.0, vProj));
	}
}

template< class float_t, class point_t >
bool computeUnnormalizedMVCForOneTriangle(
	point_t const& eta,
	point_t* tri_vertices,
	float_t* w_weights)
{
	typedef double   T;

	T epsilon = 0.000000001;

	T d[3];
	point_t u[3];

	for (unsigned int v = 0; v < 3; ++v)
	{
		d[v] = (eta - tri_vertices[v]).norm();
		u[v] = (tri_vertices[v] - eta) / d[v];
	}

	T l[3]; T theta[3];

	{
		// the Norm is CCW :
		for (unsigned int i = 0; i <= 2; ++i) {
			l[i] = (u[(i + 1) % 3] - u[(i + 2) % 3]).norm();
		}

		for (unsigned int i = 0; i <= 2; ++i) {
			theta[i] = 2.0 * asin(l[i] / 2.0);
		}

		T determinant = (tri_vertices[0] - eta).dot(
			(tri_vertices[1] - tri_vertices[0]).cross(tri_vertices[2] - tri_vertices[0]));
		T sqrdist = determinant * determinant / (4 * (tri_vertices[1] - tri_vertices[0]).cross(
			tri_vertices[2] - tri_vertices[0]).squaredNorm());
		T dist = std::sqrt(sqrdist);

		if (dist < epsilon)
		{
			// then the point eta lies on the support plane of the triangle
			T h = (theta[0] + theta[1] + theta[2]) / 2.0;
			if (M_PI - h < epsilon)
			{
				// then eta lies inside the triangle t
				return true;
			}
			else {
				w_weights[0] = w_weights[1] = w_weights[2] = 0.0;
				return false;
			}
		}

		point_t N[3];

		for (unsigned int i = 0; i < 3; ++i)
			N[i] = (tri_vertices[(i + 1) % 3] - eta).cross(tri_vertices[(i + 2) % 3] - eta);

		for (unsigned int i = 0; i <= 2; ++i)
		{
			w_weights[i] = 0.0;
			for (unsigned int j = 0; j <= 2; ++j)
				w_weights[i] += theta[j] * (N[i].dot(N[j])) / (2.0 * N[j].norm());

			w_weights[i] /= determinant;
		}
	}
	return false;
}

template< class point_t >
point_t smoothProjectInsideTet(point_t const& eta, point_t const& q0, point_t const& q1, point_t const& q2, point_t const& q3) {
	point_t proj(0, 0, 0);
	point_t tri[3];
	double sumWeights = 0.0;
	double phi[3];

	tri[0] = q0; tri[1] = q1; tri[2] = q2;
	{
		computeUnnormalizedMVCForOneTriangle(eta, tri, phi);
		sumWeights += fabs(phi[0]) + fabs(phi[1]) + fabs(phi[2]);
		proj += fabs(phi[0]) * tri[0] + fabs(phi[1]) * tri[1] + fabs(phi[2]) * tri[2];
	}

	tri[0] = q0; tri[1] = q1; tri[2] = q3;
	{
		computeUnnormalizedMVCForOneTriangle(eta, tri, phi);
		sumWeights += fabs(phi[0]) + fabs(phi[1]) + fabs(phi[2]);
		proj += fabs(phi[0]) * tri[0] + fabs(phi[1]) * tri[1] + fabs(phi[2]) * tri[2];
	}

	tri[0] = q0; tri[1] = q3; tri[2] = q2;
	{
		computeUnnormalizedMVCForOneTriangle(eta, tri, phi);
		sumWeights += fabs(phi[0]) + fabs(phi[1]) + fabs(phi[2]);
		proj += fabs(phi[0]) * tri[0] + fabs(phi[1]) * tri[1] + fabs(phi[2]) * tri[2];
	}

	tri[0] = q3; tri[1] = q1; tri[2] = q2;
	{
		computeUnnormalizedMVCForOneTriangle(eta, tri, phi);
		sumWeights += fabs(phi[0]) + fabs(phi[1]) + fabs(phi[2]);
		proj += fabs(phi[0]) * tri[0] + fabs(phi[1]) * tri[1] + fabs(phi[2]) * tri[2];
	}

	return proj / sumWeights;
}

template< class point_t >
point_t smoothProjectInsideTet_second(point_t const& eta, point_t const& q0, point_t const& q1, point_t const& q2, point_t const& q3) {
	point_t proj(0, 0, 0);
	point_t tri[3];
	double sumWeights = 0.0;
	double phi[3];

	tri[0] = q0; tri[1] = q1; tri[2] = q2;
	{
		computeUnnormalizedMVCForOneTriangle(eta, tri, phi);
		double dist_to_t = (tri[0] - eta).dot(((tri[1] - tri[0]).cross(tri[2] - tri[0])).normalized());
		if (fabs(dist_to_t) != 0.0) {
			phi[0] /= dist_to_t;
			phi[1] /= dist_to_t;
			phi[2] /= dist_to_t;
		}
		sumWeights += phi[0] + phi[1] + phi[2];
		proj += phi[0] * tri[0] + phi[1] * tri[1] + phi[2] * tri[2];
	}

	tri[0] = q0; tri[1] = q1; tri[2] = q3;
	{
		computeUnnormalizedMVCForOneTriangle(eta, tri, phi);
		double dist_to_t = (tri[0] - eta).dot(((tri[1] - tri[0]).cross(tri[2] - tri[0])).normalized());
		if (fabs(dist_to_t) != 0.0) {
			phi[0] /= dist_to_t;
			phi[1] /= dist_to_t;
			phi[2] /= dist_to_t;
		}
		sumWeights += phi[0] + phi[1] + phi[2];
		proj += phi[0] * tri[0] + phi[1] * tri[1] + phi[2] * tri[2];
	}

	tri[0] = q0; tri[1] = q3; tri[2] = q2;
	{
		computeUnnormalizedMVCForOneTriangle(eta, tri, phi);
		double dist_to_t = (tri[0] - eta).dot(((tri[1] - tri[0]).cross(tri[2] - tri[0])).normalized());
		if (fabs(dist_to_t) != 0.0) {
			phi[0] /= dist_to_t;
			phi[1] /= dist_to_t;
			phi[2] /= dist_to_t;
		}
		sumWeights += phi[0] + phi[1] + phi[2];
		proj += phi[0] * tri[0] + phi[1] * tri[1] + phi[2] * tri[2];
	}

	tri[0] = q3; tri[1] = q1; tri[2] = q2;
	{
		computeUnnormalizedMVCForOneTriangle(eta, tri, phi);
		double dist_to_t = (tri[0] - eta).dot(((tri[1] - tri[0]).cross(tri[2] - tri[0])).normalized());
		if (fabs(dist_to_t) != 0.0) {
			phi[0] /= dist_to_t;
			phi[1] /= dist_to_t;
			phi[2] /= dist_to_t;
		}
		sumWeights += phi[0] + phi[1] + phi[2];
		proj += phi[0] * tri[0] + phi[1] * tri[1] + phi[2] * tri[2];
	}

	return proj / sumWeights;
}

template<class point_t>
bool isInConvexHull(point_t const& p, point_t const& p0, point_t const& p1, point_t const& p2, point_t const& p3) {
	double s0 = 0.0, s1 = 0.0, s2 = 0.0, s3 = 0.0;
	point_t t0, t1, t2;
	t0 = p0; t1 = p1; t2 = p3;
	{
		point_t nTri = (t1 - t0).cross(t2 - t0);
		s0 = (t0 - p).dot(nTri);
	}
	t0 = p1; t1 = p2; t2 = p3;
	{
		point_t nTri = (t1 - t0).cross(t2 - t0);
		s1 = (t0 - p).dot(nTri);
	}
	t0 = p0; t1 = p2; t2 = p1;
	{
		point_t nTri = (t1 - t0).cross(t2 - t0);
		s2 = (t0 - p).dot(nTri);
	}
	t0 = p0; t1 = p3; t2 = p2;
	{
		point_t nTri = (t1 - t0).cross(t2 - t0);
		s3 = (t0 - p).dot(nTri);
	}
	return (s0 >= 0.0 && s1 >= 0.0 && s2 >= 0.0 && s3 >= 0.0) || (s0 <= 0.0 && s1 <= 0.0 && s2 <= 0.0 && s3 <= 0.0);
}

template< class point_t >
void smoothProjectOnQuad(point_t const& eta, point_t* quad_vertices, double& uProject, double& vProject) {
	point_t pToProject = eta, m;
	bool pointIsStrictlyInConvexHull = true;

	if (!isInConvexHull(pToProject, quad_vertices[0], quad_vertices[1], quad_vertices[2], quad_vertices[3])) {
		{
			pToProject = smoothProjectInsideTet_second(eta, quad_vertices[0], quad_vertices[1], quad_vertices[2], quad_vertices[3]); // either smoothProjectInsideTet or smoothProjectInsideTet_second... try them
		}
	}

	{
		m = quadMeanVector(pToProject, quad_vertices[0], quad_vertices[1], quad_vertices[2], quad_vertices[3]).normalized();

		if (pointIsStrictlyInConvexHull)
			assert(m == m && "m.isnan() : bilMeanVector seems to be problematic for a point strictly inside the convex hull");
		else
			assert(m == m && "m.isnan() : bilMeanVector seems to be problematic for a point ON the convex hull");

		computeClosestPointOnQuadFromLine(quad_vertices[0], quad_vertices[1], quad_vertices[2], quad_vertices[3], pToProject, m, uProject, vProject);
	}
}


template< class point_t >
double get_signed_solid_angle(point_t const& a, point_t const& b, point_t const& c) {
	typedef double    T;
	T det = a.dot(b.cross(c));
	if (fabs(det) < 0.0000000001) // then you're on the limit case where you cover half the sphere
		return 2.0 * M_PI; // that is particularly shitty, because the sign is difficult to estimate...

	T al = a.norm(), bl = b.norm(), cl = c.norm();

	T div = al * bl * cl + a.dot(b) * cl + a.dot(c) * bl + b.dot(c) * al;
	T at = std::atan2(std::abs(det), div);
	if (at < 0) at += M_PI; // If det>0 && div<0 atan2 returns < 0, so add pi.
	T omega = 2.0 * at;

	if (det > 0.0) return omega;
	return -omega;
}

template< class float_t, class point_t >
void computePhiAndPsiForOneTriangle(point_t const& eta,
	point_t* tri_vertices, // an array of 3 points
	float_t* phi, // an array of 3 floats
	float_t& psi) {
	typedef double    T;
	point_t Nt = (tri_vertices[1] - tri_vertices[0]).cross(tri_vertices[2] - tri_vertices[0]);
	T NtNorm = Nt.norm();
	T At = NtNorm / 2.0;
	Nt /= NtNorm;

	psi = 0.0;
	for (unsigned int v = 0; v < 3; ++v) phi[v] = 0.0;

	point_t e[3];    T e_norm[3];   point_t e_normalized[3];    T R[3];    point_t d[3];    T d_norm[3];     T C[3];     point_t J[3];
	for (unsigned int v = 0; v < 3; ++v) e[v] = tri_vertices[v] - eta;
	for (unsigned int v = 0; v < 3; ++v) e_norm[v] = e[v].norm();
	for (unsigned int v = 0; v < 3; ++v) e_normalized[v] = e[v] / e_norm[v];

	T signed_solid_angle = get_signed_solid_angle(e_normalized[0], e_normalized[1], e_normalized[2]) / (4.f * M_PI);
	T signed_volume = (e[0].cross(e[1])).dot(e[2]) / 6.0;

	for (unsigned int v = 0; v < 3; ++v) R[v] = e_norm[(v + 1) % 3] + e_norm[(v + 2) % 3];
	for (unsigned int v = 0; v < 3; ++v) d[v] = tri_vertices[(v + 1) % 3] - tri_vertices[(v + 2) % 3];
	for (unsigned int v = 0; v < 3; ++v) d_norm[v] = d[v].norm();
	for (unsigned int v = 0; v < 3; ++v) C[v] = std::log((R[v] + d_norm[v]) / (R[v] - d_norm[v])) / (4.0 * M_PI * d_norm[v]);

	point_t Pt(-signed_solid_angle * Nt);
	for (unsigned int v = 0; v < 3; ++v) Pt += Nt.cross(C[v] * d[v]);
	for (unsigned int v = 0; v < 3; ++v) J[v] = e[(v + 2) % 3].cross(e[(v + 1) % 3]);

	psi = -3.0 * signed_solid_angle * signed_volume / At;
	for (unsigned int v = 0; v < 3; ++v) psi -= C[v] * J[v].dot(Nt);
	for (unsigned int v = 0; v < 3; ++v) phi[v] += Pt.dot(J[v]) / (2.0 * At);
}

Eigen::Vector4d bilinear_sheet(double u, double v)
{
	return { (1. - u) * (1. - v), u * (1. - v), u * v, (1. - u) * v };
}

template<class point_t>
point_t interpolate_on_quad(point_t* quad_vertices, double u, double v)
{
	auto const b = bilinear_sheet(u, v);
	return quad_vertices[0] * b(0) + quad_vertices[1] * b(1) + quad_vertices[2] * b(2) + quad_vertices[3] * b(3);
}

template<class point_t>
point_t quad_u_tangent(point_t* quad_vertices, double v)
{
	return (1. - v) * (quad_vertices[1] - quad_vertices[0]) + v * (quad_vertices[2] - quad_vertices[3]);
}

template<class point_t>
point_t quad_v_tangent(point_t* quad_vertices, double u)
{
	return (1. - u) * (quad_vertices[3] - quad_vertices[0]) + u * (quad_vertices[2] - quad_vertices[1]);
}


template<class point_t>
point_t interpolated_quad_normal(point_t* quad_vertices, double u, double v)
{
	auto const u_tangent = quad_u_tangent(quad_vertices, v);
	auto const v_tangent = quad_v_tangent(quad_vertices, u);
	return u_tangent.cross(v_tangent);
}

template<class point_t>
Eigen::VectorXd computePhiAndPsiForOneQuad(point_t const& eta, point_t* quad_vertices)
{
	double u_projected = 0, v_projected = 0;
	smoothProjectOnQuad(eta, quad_vertices, u_projected, v_projected);
	assert(0. <= u_projected && u_projected <= 1.);
	assert(0. <= v_projected && v_projected <= 1.);

	Eigen::VectorXd Phi;
	Phi.resize(8);
	Phi.fill(0);
	const int n = 2;
	const double m = 3.;
	const int N = 2 * n + 2;
	double u[N + 1], v[N + 1];

	for (int i = 0; i < N + 1; ++i)
	{
		if (i < N / 2)
		{
			const double factor = 1. - std::pow((static_cast<double>(N / 2) - static_cast<double>(i)) / static_cast<double>(N / 2), m);
			u[i] = factor * u_projected;
			v[i] = factor * v_projected;
		}
		else if (i == N / 2)
		{
			u[i] = u_projected;
			v[i] = v_projected;
		}
		else
		{
			const double factor = 1. - std::pow((static_cast<double>(i) - static_cast<double>(N / 2)) / static_cast<double>(N / 2), m);
			u[i] = (u_projected - 1.) * factor + 1.;
			v[i] = (v_projected - 1.) * factor + 1.;
		}
	}

	auto Proc = [&](Eigen::Vector3i u_idx, Eigen::Vector3i v_idx)
	{
		double u_tri[3], v_tri[3];
		for (int j = 0; j < 3; ++j)
		{
			u_tri[j] = u[u_idx(j)];
			v_tri[j] = v[v_idx(j)];
		}
		auto const u_avg = (u_tri[0] + u_tri[1] + u_tri[2]) / 3.;
		auto const v_avg = (v_tri[0] + v_tri[1] + v_tri[2]) / 3.;
		point_t tesselated_tri[3];
		for (int k = 0; k < 3; ++k)
		{
			tesselated_tri[k] = interpolate_on_quad(quad_vertices, u_tri[k], v_tri[k]);
		}
		auto const b_avg = bilinear_sheet(u_avg, v_avg);

		point_t Nt = (tesselated_tri[1] - tesselated_tri[0]).cross(tesselated_tri[2] - tesselated_tri[0]);
		double NtNorm = Nt.norm();
		double At = NtNorm / 2.0;
		Nt /= NtNorm;

		if (At < 0.0000000001)
		{
			return;
		}

		double psi_tri = 0.0;

		point_t e[3];    double e_norm[3];   point_t e_normalized[3];    double R[3];    point_t d[3];    double d_norm[3];     double C[3];     point_t J[3];
		for (unsigned int v = 0; v < 3u; ++v) e[v] = tesselated_tri[v] - eta;
		for (unsigned int v = 0; v < 3u; ++v) e_norm[v] = e[v].norm();
		for (unsigned int v = 0; v < 3u; ++v) e_normalized[v] = e[v] / e_norm[v];

		auto const omega_tri = get_signed_solid_angle(e_normalized[0], e_normalized[1], e_normalized[2]);
		auto const signed_solid_angle = omega_tri / (4.f * M_PI);
		auto const signed_volume = (e[0].cross(e[1])).dot(e[2]) / 6.0;

		for (unsigned int v = 0; v < 3; ++v) R[v] = e_norm[(v + 1) % 3] + e_norm[(v + 2) % 3];
		for (unsigned int v = 0; v < 3; ++v) d[v] = tesselated_tri[(v + 1) % 3] - tesselated_tri[(v + 2) % 3];
		for (unsigned int v = 0; v < 3; ++v) d_norm[v] = d[v].norm();
		for (unsigned int v = 0; v < 3; ++v) C[v] = std::log((R[v] + d_norm[v]) / (R[v] - d_norm[v])) / (4.0 * M_PI * d_norm[v]);

		point_t Pt(-signed_solid_angle * Nt);
		for (unsigned int v = 0; v < 3; ++v) Pt += Nt.cross(C[v] * d[v]);
		for (unsigned int v = 0; v < 3; ++v) J[v] = e[(v + 2) % 3].cross(e[(v + 1) % 3]);

		psi_tri = -3.0 * signed_solid_angle * signed_volume / At;
		for (unsigned int v = 0; v < 3; ++v) psi_tri -= C[v] * J[v].dot(Nt);

		auto const phi_quad = b_avg * omega_tri / (4.f * M_PI);
		auto const interpolated_normal = interpolated_quad_normal(quad_vertices, u_avg, v_avg);
		auto const psi_quad = psi_tri * b_avg / interpolated_normal.norm();
		for (unsigned int k = 0; k < 4u; ++k)
		{
			assert(phi_quad(k) == phi_quad(k));
			auto const phi_k = phi_quad(k);
			Phi(k) += phi_k;
			assert(psi_quad(k) == psi_quad(k));
			Phi(k + 4u) += psi_quad(k);
		}
	};

	for (int v_it = 0; v_it <= n; ++v_it)
	{
		for (int u_it = v_it; u_it <= (2 * n - v_it); ++u_it)
		{
			if ((u_it + v_it) % 2 == 0)
			{
				Proc({ u_it, u_it + 2, u_it + 1 }, { v_it, v_it, v_it + 1 });
				Proc({ u_it, u_it + 1, u_it + 2 }, { N - v_it, N - v_it - 1, N - v_it });
				Proc({ v_it, v_it + 1, v_it }, { u_it, u_it + 1, u_it + 2 });
				Proc({ N - v_it, N - v_it, N - v_it - 1 }, { u_it, u_it + 2, u_it + 1 });
			}
			else
			{
				Proc({ u_it, u_it + 1, u_it + 2 }, { v_it + 1, v_it, v_it + 1 });
				Proc({ u_it, u_it + 2, u_it + 1 }, { N - v_it - 1, N - v_it - 1, N - v_it });
				Proc({ v_it + 1, v_it + 1, v_it }, { u_it, u_it + 2, u_it + 1 });
				Proc({ N - v_it - 1, N - v_it, N - v_it - 1 }, { u_it, u_it + 1, u_it + 2 });
			}
		}
	}

	return Phi;
}

template<class point_t>
float_t sigma_L(point_t* old_quad_vertices, point_t* new_quad_vertices, double u, double v)
{
	auto const old_u_tangent = quad_u_tangent(old_quad_vertices, v);
	auto const old_v_tangent = quad_v_tangent(old_quad_vertices, u);
	auto const new_u_tangent = quad_u_tangent(new_quad_vertices, v);
	auto const new_v_tangent = quad_v_tangent(new_quad_vertices, u);

	return std::sqrt((new_u_tangent.squaredNorm() * old_v_tangent.squaredNorm() + old_u_tangent.squaredNorm() * new_v_tangent.squaredNorm() - 2. *
		new_u_tangent.dot(new_v_tangent) * old_u_tangent.dot(old_v_tangent)) / (2. * (old_u_tangent.cross(old_v_tangent)).squaredNorm()));
}

template<class point_t>
float_t sigma_a(point_t* old_quad_vertices, point_t* new_quad_vertices, double u, double v)
{
	auto const old_normal = interpolated_quad_normal(old_quad_vertices, u, v);
	auto const new_normal = interpolated_quad_normal(new_quad_vertices, u, v);
	return new_normal.norm() / old_normal.norm();
}

template<class point_t>
Eigen::Vector4d numerically_approx_sigma_q(point_t* old_quad_vertices, point_t* new_quad_vertices, int n = 10)
{
	auto const numSamples = n * n;
	auto const dx = 1. / static_cast<double>(n);
	Eigen::Vector4d res_sigma_a = { 0, 0, 0, 0 };
	Eigen::Vector4d res_sigma_L = { 0, 0, 0, 0 };
	for (int i = 1; i <= n; ++i)
	{
		auto const u = static_cast<double>(i) * dx - dx * .5;

		for (int j = 1; j <= n; ++j)
		{
			auto const v = static_cast<double>(j) * dx - dx * .5;

			auto const b = bilinear_sheet(u, v);
			auto const normal = interpolated_quad_normal(old_quad_vertices, u, v);
			auto const sigma_a_approx = sigma_a(old_quad_vertices, new_quad_vertices, u, v);
			auto const sigma_L_approx = sigma_L(old_quad_vertices, new_quad_vertices, u, v);
			res_sigma_a += dx * dx * b * sigma_a_approx / normal.stableNorm();
			res_sigma_L += dx * dx * b * sigma_L_approx / normal.stableNorm();
		}
	}
	Eigen::Vector4d sigma = { 0, 0, 0, 0 };
	for (int i = 0; i < 4; ++i)
	{
		sigma(i) = res_sigma_L(i) / res_sigma_a(i);
	}

	return sigma;
}

void calculateGreenCoordinatesFromQMVC(const Eigen::MatrixXd& C, const Eigen::MatrixXi& CF, const Eigen::MatrixXd& normals, const Eigen::MatrixXd& eta_m,
	Eigen::MatrixXd& phi, Eigen::MatrixXd& psi)
{
	phi.resize(C.rows(), eta_m.rows());
	psi.resize(CF.rows(), eta_m.rows());
	phi.fill(0); psi.fill(0);

	for (int i = 0; i < eta_m.rows(); ++i)
	{
		const Eigen::Vector3d eta = eta_m.row(i);

		for (unsigned int face_idx = 0; face_idx < CF.rows(); ++face_idx)
		{
			Eigen::Vector3d tri_verts[3];
			const Eigen::Vector3i tri_indices = CF.row(face_idx);
			double phi_[3];
			for (unsigned int l = 0; l < 3u; ++l)
			{
				tri_verts[l] = C.row(tri_indices[l]);
			}
			computePhiAndPsiForOneTriangle(eta, tri_verts, phi_, psi(face_idx, i));

			for (unsigned int v = 0; v < 3u; ++v)
			{
				phi(tri_indices[v], i) += phi_[v];
			}
		}
	}

	for (unsigned int i = 0; i < eta_m.rows(); ++i)
	{
		double res = 0;
		for (unsigned int j = 0; j < C.rows(); ++j)
		{
			res += phi(j, i);
		}
		assert(std::abs(1. - res) < 1e-3);
	}
}

#define EPS 0.00000000001

template <typename T> T sgn(T val) {
	return val >= EPS ? T(1) : T(-1);
}

double GCTriInt(const Eigen::Vector3d p, const Eigen::Vector3d t1, const Eigen::Vector3d t2, const Eigen::Vector3d eta = { 0., 0., 0. })
{
	auto const t2mt1 = t2 - t1;
	auto const pmt1 = p - t1;

	auto tmpVal = t2mt1.dot(p - t1) / (t2mt1.stableNorm() * pmt1.stableNorm());

	if (std::abs(tmpVal) > 1.0 - EPS)
	{
		return 0;
	}

	auto const alpha = std::acos(tmpVal);

	if (std::abs(alpha - M_PI) < EPS || std::abs(alpha) < EPS)
	{
		return 0.0;
	}

	auto const t1mp = t1 - p;
	auto const t2mp = t2 - p;

	tmpVal = t1mp.dot(t2mp) / (t1mp.stableNorm() * t2mp.stableNorm());

	if (std::abs(tmpVal) > 1.0 - EPS)
	{
		return 0;
	}

	auto const beta = std::acos(tmpVal);

	auto const lambda = pmt1.squaredNorm() * std::sin(alpha) * std::sin(alpha);
	auto const pmeta = p - eta;
	auto const c = pmeta.squaredNorm();

	auto const sqrt_c = std::sqrt(c);
	auto const sqrt_lambda = std::sqrt(lambda);

	auto getI = [&](double theta) {
		auto const S = std::sin(theta);
		auto const C = std::cos(theta);
		auto const fac_sign = -.5 * static_cast<double>(sgn(S));
		auto const add_1 = 2. * sqrt_c * std::atan(C * sqrt_c / std::sqrt(lambda + S * S * c));
		auto const denom1 = (1. - C) * (1. - C);
		auto const denom2 = c * (1. + C) + lambda + std::sqrt(lambda * lambda + lambda * c * S * S);
		auto const add_2 = sqrt_lambda * std::log((2. * sqrt_lambda * S * S) / denom1 * (1. - 2. * c * C / denom2));
		return fac_sign * (add_1 + add_2);
	};

	auto const I_1 = getI(M_PI - alpha);
	auto const I_2 = getI(M_PI - alpha - beta);

	return (-1. / (4 * M_PI)) * std::abs(I_1 - I_2 - sqrt_c * beta);
}

void calculateGreenCoordinates(const Eigen::MatrixXd& C, const Eigen::MatrixXi& CF, const Eigen::MatrixXd& normals, const Eigen::MatrixXd& eta_m,
	Eigen::MatrixXd& phi, Eigen::MatrixXd& psi)
{

	std::cout << "GC" << std::endl;

	phi.resize(C.rows(), eta_m.rows());
	psi.resize(CF.rows(), eta_m.rows());
	phi.fill(0); psi.fill(0);

	for (int i = 0; i < eta_m.rows(); ++i)
	{
		const Eigen::Vector3d eta = eta_m.row(i);

		for (int face_idx = 0; face_idx < CF.rows(); ++face_idx)
		{
			const Eigen::Vector3i index_vec = CF.row(face_idx);
			const Eigen::Vector3d t_0 = C.row(index_vec[0]);
			const Eigen::Vector3d t_1 = C.row(index_vec[1]);
			const Eigen::Vector3d t_2 = C.row(index_vec[2]);

			Eigen::Vector3d v[3];

			v[0] = t_0 - eta;
			v[1] = t_1 - eta;
			v[2] = t_2 - eta;

			const Eigen::Vector3d normal = normals.row(face_idx);
			assert(std::abs(1.0 - normal.stableNorm()) < 1e-4);

			auto const p = v[0].dot(normal) * normal;

			double s[3];
			double I[3];
			double II[3];
			Eigen::Vector3d N[3];

			for (unsigned int l = 0; l < 3u; ++l)
			{
				s[l] = sgn(((v[l] - p).cross(v[(l + 1u) % 3u] - p)).dot(normal));
				I[l] = GCTriInt(p, v[l], v[(l + 1) % 3]);
				II[l] = GCTriInt({ 0., 0., 0. }, v[(l + 1) % 3], v[l]);
				if (II[l] != II[l])
				{
					std::cout << "NAN";
				}

				auto const q = v[(l + 1) % 3].cross(v[l]);
				N[l] = q.stableNormalized();
			}

			auto const I_final = -1. * std::abs(s[0] * I[0] + s[1] * I[1] + s[2] * I[2]);
			auto const psi_normal = -I_final;

			if (psi_normal != psi_normal)
			{
				std::cout << "NAN\n";
			}

			psi(face_idx, i) = psi_normal;

			Eigen::Vector3d const w = normal * I_final + N[0] * II[0] + N[1] * II[1] + N[2] * II[2];

			auto const w_norm = w.stableNorm();

			if (w.stableNorm() > 1.e-3)
			{
				for (unsigned int l = 0; l < 3u; ++l)
				{
					auto const phi_vert = (N[(l + 1u) % 3u].dot(w)) / (N[(l + 1u) % 3u].dot(v[l]));
					auto const phi_vert_abs = std::abs(phi_vert);
					phi(index_vec(l), i) += phi_vert;
				}
			}

		}
	}

	for (unsigned int i = 0; i < eta_m.rows(); ++i)
	{
		double res = 0;
		for (unsigned int j = 0; j < C.rows(); ++j)
		{
			res += phi(j, i);
		}
		assert(std::abs(1. - res) < 1e-3);
	}
}

void calculateGreenCoordinatesTriQuad(const Eigen::MatrixXd& C, const Eigen::MatrixXi& CF, Eigen::MatrixXd const& eta_m,
	Eigen::MatrixXd& phi, std::vector<double>& psi_tri, std::vector<Eigen::Vector4d>& psi_quad)
{
	phi.resize(C.rows(), eta_m.rows());
	phi.fill(0);

	bool contains_quads = CF.cols() == 4;

	for (int eta_idx = 0; eta_idx < eta_m.rows(); ++eta_idx)
	{
		const Eigen::Vector3d eta = eta_m.row(eta_idx);

		for (int face_idx = 0; face_idx < CF.rows(); ++face_idx)
		{
			auto const face = CF.row(face_idx);
			Eigen::Vector3d face_points[4];

			bool isTriangle = face.size() == 3 || (contains_quads && face(3) == -1);
			bool isQuad = contains_quads && face(3) != -1;

			for (int k = 0; k < face.size(); ++k)
			{
				auto const v_idx = face(k);
				if (v_idx != -1)
				{
					face_points[k] = C.row(v_idx);
				}
			}

			if (isTriangle)
			{
				double phi_[3];
				double psi = 0;
				computePhiAndPsiForOneTriangle(eta, face_points, phi_, psi);

				for (int k = 0; k < 3; ++k)
				{
					phi(face(k), eta_idx) += phi_[k];
				}
				psi_tri.push_back(psi);
			}
			else if (isQuad)
			{
				auto const Phi = computePhiAndPsiForOneQuad(eta, face_points);
				assert(Phi.size() == 8);

				for (int k = 0; k < 4; ++k)
				{
					phi(face[k], eta_idx) += Phi(k);
				}
				psi_quad.push_back({ Phi(4), Phi(5), Phi(6), Phi(7) });
			}
			else
			{
				std::cerr << "QGC supports only triangles and quads! Unsupported face type found!\n";
			}
		}
	}
}

// MVC : Code from "Mean Value Coordinates for Closed Triangular Meshes" Schaeffer Siggraph 2005
void computeMVCForOneVertex(Eigen::MatrixXd const & C, Eigen::MatrixXi const & CF,
	Eigen::Vector3d eta, Eigen::VectorXd& weights, Eigen::VectorXd& w_weights) {
	auto const num_vertices_cage = C.rows();
	auto const num_faces_cage = CF.rows();

    double epsilon = 0.00000001;

    w_weights.setZero();
    weights.setZero();
    double sumWeights = 0.0;

	Eigen::VectorXd d(num_vertices_cage);
	d.setZero();
	Eigen::MatrixXd u(num_vertices_cage, 3);

    for (unsigned int v = 0 ; v < num_vertices_cage ; ++v) {
		Eigen::Vector3d cage_vertex = C.row(v);
        d(v) = (eta - cage_vertex).norm();
        if (d(v) < epsilon) {
            weights(v) = 1.0;
            return;
        }
        u.row(v) = (cage_vertex - eta) / d(v);
    }

    unsigned int vid[3]; 
	double l[3]; 
	double theta[3]; 
	double w[3]; 
	double c[3]; 
	double s[3];
    for(unsigned int t = 0 ; t < num_faces_cage ; ++t) { // the Norm is CCW :
        for(unsigned int i = 0 ; i <= 2 ; ++i) {
			vid[i] =  CF(t,i);
		}
        for(unsigned int i = 0 ; i <= 2 ; ++i) {
			Eigen::Vector3d v_0 = u.row(vid[(i + 1) % 3]);
			Eigen::Vector3d v_1 = u.row(vid[(i + 2) % 3]);
			l[i] = (v_0 - v_1).norm();
		}
        for(unsigned int i = 0 ; i <= 2 ; ++i) {
			theta[i] = 2.0 * asin( l[i] / 2.0 );
		}
        double h = ( theta[0] + theta[1] + theta[2] ) / 2.0;
        if(M_PI - h < epsilon) { // eta is on the triangle t , use 2d barycentric coordinates :
            for(unsigned int i = 0 ; i <= 2 ; ++i) {
				 w[i] = sin(theta[i]) * l[(i+2) % 3] * l[(i+1) % 3];
			}

            sumWeights = w[0] + w[1] + w[2];

            w_weights.setZero();
            weights(vid[0]) = w[0] / sumWeights;
            weights(vid[1]) = w[1] / sumWeights;
            weights(vid[2]) = w[2] / sumWeights;

            return;
        }

        for(unsigned int i = 0; i <= 2; ++i) {
			c[i] = (2.0 * sin(h) * sin(h - theta[i])) / (sin(theta[(i+1) % 3]) * sin(theta[(i+2) % 3])) - 1.0;
		}

        double sign_Basis_u0u1u2 = 1;
		Eigen::Vector3d v_0 = u.row(vid[0]);
		Eigen::Vector3d v_1 = u.row(vid[1]);
		Eigen::Vector3d v_2 = u.row(vid[2]);
        if( v_2.dot(v_0.cross(v_1)) < 0.0 ) {
			sign_Basis_u0u1u2 = -1;
		}

        for(unsigned int i = 0; i <= 2; ++i) {
			 s[i] = sign_Basis_u0u1u2 * std::sqrt(std::max<double>(0.0 , 1.0 - c[i] * c[i]));
		}
        if(std::fabs(s[0]) < epsilon || std::fabs(s[1]) < epsilon || std::fabs(s[2]) < epsilon ) {
			continue; // eta is on the same plane, outside t  ->  ignore triangle t : 
		} 
        for(unsigned int i = 0; i <= 2; ++i) {
			w[i] = (theta[i] - c[(i+1)% 3] * theta[(i+2) % 3] - c[(i+2) % 3] * theta[(i+1) % 3]) / (2.0 * d(vid[i]) * sin(theta[(i+1) % 3]) * s[(i+2) % 3]);
		}
        sumWeights += (w[0] + w[1] + w[2]);
        w_weights(vid[0]) += w[0];
        w_weights(vid[1]) += w[1];
        w_weights(vid[2]) += w[2];
    }

    for(unsigned int v = 0 ; v < num_vertices_cage ; ++v) {
		weights(v)  = w_weights(v) / sumWeights;
	}
}

/// A simple but more robust scheme to compute MVC supposed in https://github.com/superboubek/QMVC
void computeMVCForOneVertexSimple(Eigen::MatrixXd const & C, Eigen::MatrixXi const & CF,
	Eigen::Vector3d eta, Eigen::VectorXd& weights, Eigen::VectorXd& w_weights) {
	double epsilon = 0.000000001;

    auto const num_vertices_cage = C.rows();
	auto const num_faces_cage = CF.rows();

    w_weights.setZero();
    weights.setZero();
    double sumWeights = 0.0;

	Eigen::VectorXd d(num_vertices_cage);
	d.setZero();
	Eigen::MatrixXd u(num_vertices_cage, 3);

    for(unsigned int v = 0; v < num_vertices_cage; ++v) {
		const Eigen::Vector3d cage_vertex = C.row(v);
        d(v) = (eta - cage_vertex).norm();
        if(d(v) < epsilon) {
            weights(v) = 1.0;
            return;
        }
        u.row(v) = (cage_vertex - eta ) / d(v);
    }

    unsigned int vid[3];
    double l[3], theta[3], w[3];

    for(unsigned int t = 0 ; t < num_faces_cage; ++t) {
        // the Norm is CCW :
        for(unsigned int i = 0; i <= 2; ++i) {
            vid[i] =  CF(t,i);
		}

        for(unsigned int i = 0; i <= 2; ++i) {
			const Eigen::Vector3d v_0 = u.row(vid[(i + 1) % 3]);
			const Eigen::Vector3d v_1 = u.row(vid[(i + 2) % 3]);
            l[i] = (v_0 - v_1).norm();
        }

        for( unsigned int i = 0; i <= 2; ++i) {
			const Eigen::Vector3d v_0 = u.row(vid[(i + 1) % 3]);
			const Eigen::Vector3d v_1 = u.row(vid[(i + 2) % 3]);
            theta[i] = 2. * asin((v_0 - v_1).norm() * .5);
        }

        // test in original MVC paper: (they test if one angle psi is close to 0: it is "distance sensitive" in the sense that it does not
        // relate directly to the distance to the support plane of the triangle, and the more far away you go from the triangle, the worse it is)
        // In our experiments, it is actually not the good way to do it, as it increases significantly the errors we get in the computation of weights and derivatives,
        // especially when evaluating Hfx, Hfy, Hfz which can be of norm of the order of 10^3 instead of 0 (when specifying identity on the cage, see paper)

        // simple test we suggest:
        // the determinant of the basis is 2*area(T)*d( eta , support(T) ), we can directly test for the distance to support plane of the triangle to be minimum

		const Eigen::Vector3d c_0 = C.row(vid[0]);
		const Eigen::Vector3d c_1 = C.row(vid[1]);
		const Eigen::Vector3d c_2 = C.row(vid[2]);
        double determinant = (c_0 - eta).dot((c_1 - c_0).cross(c_2 - c_0));
        double sqrdist = determinant*determinant / (4 * ((c_1 - c_0).cross(c_2 - c_0)).squaredNorm());
        double dist = std::sqrt(sqrdist);

        if(dist < epsilon) {
            // then the point eta lies on the support plane of the triangle
            double h = (theta[0] + theta[1] + theta[2]) * .5;
            if(M_PI - h < epsilon) {
                // eta lies inside the triangle t , use 2d barycentric coordinates :
                for( unsigned int i = 0; i <= 2; ++i) {
                    w[i] = sin(theta[i]) * l[(i+2) % 3] * l[(i+1) % 3];
                }
                sumWeights = w[0] + w[1] + w[2];

                w_weights.setZero();
                weights(vid[0]) = w[0] / sumWeights;
                weights(vid[1]) = w[1] / sumWeights;
                weights(vid[2]) = w[2] / sumWeights;
                return;
            }
        }

        Eigen::Vector3d pt[3], N[3];
        for(unsigned int i = 0; i < 3; ++i) {
            pt[i] = C.row(CF(t, i));
		}
        for(unsigned int i = 0; i < 3; ++i) {
            N[i] = (pt[(i+1)%3] - eta).cross(pt[(i+2)%3] - eta);
		}

        for(unsigned int i = 0; i <= 2; ++i) {
            w[i] = 0.0;
            for(unsigned int j = 0; j <= 2; ++j) {
                w[i] += theta[j] * N[i].dot(N[j]) / (2.0 * N[j].norm());
			}
            w[i] /= determinant;
        }

        sumWeights += (w[0] + w[1] + w[2]);
        w_weights(vid[0]) += w[0];
        w_weights(vid[1]) += w[1];
        w_weights(vid[2]) += w[2];
    }

    for(unsigned int v = 0; v < num_vertices_cage; ++v) {
        weights(v)  = w_weights(v) / sumWeights;
	}
}


void computeMVC(const Eigen::MatrixXd& C, const Eigen::MatrixXi& CF, Eigen::MatrixXd const& eta_m,
    Eigen::MatrixXd& phi) {

		std::cout << "MVC" << std::endl;

		phi.resize(C.rows(), eta_m.rows());
		Eigen::VectorXd w_weights(C.rows());
		Eigen::VectorXd weights(C.rows());
	
	
		for (int eta_idx = 0; eta_idx < eta_m.rows(); ++eta_idx) {
			const Eigen::Vector3d eta = eta_m.row(eta_idx);
			computeMVCForOneVertexSimple(C, CF, eta, weights, w_weights);
			phi.col(eta_idx) = weights;
		}		
}

double getSpanDeterminant(Eigen::Vector3d const& v1, Eigen::Vector3d const& v2, Eigen::Vector3d const& v3) {
	return  v1.cross(v2).dot(v3);
}

Eigen::Vector3d direction(Eigen::Vector3d const & a)
{
	auto const n = a.norm();
	if (n < 0.0000000000000001)
	{
		return Eigen::Vector3d(0,0,0);
	}
	else
	{
		return a / n;
	}
}

double compute2DWindingNumberInQuad(Eigen::Vector3d const& eta, Eigen::Vector3d const * quad_vertices, Eigen::Vector3d const& Ntri) {
	const Eigen::Vector3d u0 = direction(quad_vertices[0] - eta);
	const Eigen::Vector3d u1 = direction(quad_vertices[1] - eta);
	const Eigen::Vector3d u2 = direction(quad_vertices[2] - eta);
	const Eigen::Vector3d u3 = direction(quad_vertices[3] - eta);

	const double t0 = asin(std::min<double>(1.0, std::max<double>(-1.0, (Ntri.dot(u0.cross(u1))))));
	const double t1 = asin(std::min<double>(1.0, std::max<double>(-1.0, (Ntri.dot(u1.cross(u2))))));
	const double t2 = asin(std::min<double>(1.0, std::max<double>(-1.0, (Ntri.dot(u2.cross(u3))))));
	const double t3 = asin(std::min<double>(1.0, std::max<double>(-1.0, (Ntri.dot(u3.cross(u0))))));
	return t0 + t1 + t2 + t3;
}

void computeFloaterBarycentricCoordinatesInPlanarQuad(
	Eigen::Vector3d const& eta,
	Eigen::Vector3d const * quad_vertices,
	Eigen::Vector3d const& nq,
	double& u, double& v)
{
	const double A0 = nq.dot((quad_vertices[0] - eta).cross(quad_vertices[1] - eta)) / 2;
	const double A1 = nq.dot((quad_vertices[1] - eta).cross(quad_vertices[2] - eta)) / 2;
	const double A2 = nq.dot((quad_vertices[2] - eta).cross(quad_vertices[3] - eta)) / 2;
	const double A3 = nq.dot((quad_vertices[3] - eta).cross(quad_vertices[0] - eta)) / 2;
	const double B0 = nq.dot((quad_vertices[3] - eta).cross(quad_vertices[1] - eta)) / 2;
	const double B1 = nq.dot((quad_vertices[0] - eta).cross(quad_vertices[2] - eta)) / 2;
	// double B2 = point_t::dot(nq , point_t::cross(q1-eta,q3-eta)) / 2;
	const double B3 = nq.dot((quad_vertices[2] - eta).cross(quad_vertices[0] - eta)) / 2;

	const double D = std::max<double>(0.0, B0 * B0 + B1 * B1 + 2 * A0 * A2 + 2 * A1 * A3);
	const double D_sqrt = sqrt(D);

	const double E0 = 2 * A0 - B0 - B1 + D_sqrt;
	const double E3 = 2 * A3 - B3 - B0 + D_sqrt;

	u = 2 * A3 / E3;
	v = 2 * A0 / E0;
}

bool computeMVCTriQuadCoordinates(
	unsigned int eta_idx,
	Eigen::Vector3d eta,
	std::vector<Eigen::Vector3i> const& cage_triangles,
	std::vector<Eigen::Vector4i> const& cage_quads,
	const Eigen::MatrixXd & cage_vertices,
	Eigen::MatrixXd & weights,
	Eigen::MatrixXd & w_weights)
{
	double epsilon = 0.000001;

	unsigned int n_vertices = cage_vertices.rows();
	unsigned int n_triangles = cage_triangles.size();
	unsigned int n_quads = cage_quads.size();

	double sumWeights = 0.0;

	std::vector<double> d(n_vertices, 0.0);
	std::vector<Eigen::Vector3d> u(n_vertices);

	for (unsigned int v = 0; v < n_vertices; ++v)
	{
		const Eigen::Vector3d cage_vertex = cage_vertices.row(v);
		d[v] = (eta - cage_vertex).norm();
		if (d[v] < epsilon)
		{
			weights(v, eta_idx) = 1.0;
			return true;
		}
		u[v] = (cage_vertex - eta) / d[v];
	}

	w_weights.col(eta_idx).setZero();

	// CAGE TRIANGLES:
	{
		unsigned int vid[3];
		double l[3]; double theta[3]; double w[3];

		for (unsigned int t = 0; t < n_triangles; ++t)
		{
			// the Norm is CCW :
			for (unsigned int i = 0; i <= 2; ++i)
				vid[i] = cage_triangles[t][i];

			for (unsigned int i = 0; i <= 2; ++i)
				l[i] = (u[vid[(i + 1) % 3]] - u[vid[(i + 2) % 3]]).norm();

			for (unsigned int i = 0; i <= 2; ++i)
				theta[i] = 2.0 * asin(l[i] / 2.0);

			const Eigen::Vector3d t_0 = cage_vertices.row(vid[0]);
			const Eigen::Vector3d t_1 = cage_vertices.row(vid[1]);
			const Eigen::Vector3d t_2 = cage_vertices.row(vid[2]);

			double determinant = (t_0 - eta).dot((t_1 - t_0).cross(t_2 - t_0));
			double sqrdist = determinant * determinant / (4 * ((t_1 - t_0).cross(t_2 - t_0)).squaredNorm());
			double dist = sqrt(sqrdist);

			if (dist < epsilon) {
				// then the point eta lies on the support plane of the triangle
				double h = (theta[0] + theta[1] + theta[2]) / 2.0;
				if (M_PI - h < epsilon) {
					// eta lies inside the triangle t , use 2d barycentric coordinates :
					for (unsigned int i = 0; i <= 2; ++i) {
						w[i] = sin(theta[i]) * l[(i + 2) % 3] * l[(i + 1) % 3];
					}
					sumWeights = w[0] + w[1] + w[2];

					w_weights.col(eta_idx).setZero();
					weights.col(eta_idx).setZero();
					weights(vid[0], eta_idx) = w[0] / sumWeights;
					weights(vid[1], eta_idx) = w[1] / sumWeights;
					weights(vid[2], eta_idx) = w[2] / sumWeights;
					return true;
				}
			}

			Eigen::Vector3d pt[3], N[3];

			for (unsigned int i = 0; i < 3; ++i)
				pt[i] = cage_vertices.row(vid[i]);

			for (unsigned int i = 0; i < 3; ++i)
				N[i] = (pt[(i + 1) % 3] - eta).cross(pt[(i + 2) % 3] - eta);

			for (unsigned int i = 0; i <= 2; ++i) {
				w[i] = 0.0;
				for (unsigned int j = 0; j <= 2; ++j)
					w[i] += theta[j] * N[i].dot(N[j]) / (2.0 * N[j].norm());

				w[i] /= determinant;
			}

			sumWeights += (w[0] + w[1] + w[2]);
			w_weights(vid[0], eta_idx) += w[0];
			w_weights(vid[1], eta_idx) += w[1];
			w_weights(vid[2], eta_idx) += w[2];
		}
	}

	// CAGE QUADS :
	{
		unsigned int vid[4];
		Eigen::Vector4d w;

		for (unsigned int q = 0; q < n_quads; ++q) {
			// the Norm is CCW :
			Eigen::Vector3d pt[4];
			for (unsigned int i = 0; i < 4; ++i) {
				vid[i] = cage_quads[q][i];
				pt[i] = cage_vertices.row(vid[i]);
			}

			if (fabs(getSpanDeterminant(pt[1] - pt[0], pt[2] - pt[0], pt[3] - pt[0])) < epsilon
				&& fabs(getSpanDeterminant(pt[0] - eta, pt[1] - eta, pt[2] - eta)) < epsilon) {
				// then the quad is planar, eta is on the support plane.
				// i) if eta is INSIDE the quad, then output barycentric functions of the quad's vertices
				// ii) if eta is OUTSIDE the quad, just set weights w.r.t. the quad's vertices to 0 and go on with other faces
				// find out if eta is inside the quad:
				Eigen::Vector3d const& Ntri = direction(pt[1] - pt[0]).cross(pt[2] - pt[0]);
				double wn = fabs(compute2DWindingNumberInQuad(eta, pt, Ntri));
				bool eta_is_inside_the_quad = (M_PI < wn) && (wn < 3.0 * M_PI);
				if (eta_is_inside_the_quad) {
					// compute barycentric coordinates:
					double u, v;
					computeFloaterBarycentricCoordinatesInPlanarQuad(eta, pt, Ntri, u, v);

					w_weights.col(eta_idx).setZero();
					weights.col(eta_idx).setZero();
					weights(vid[0], eta_idx) = (1.0 - u) * (1.0 - v);
					weights(vid[1], eta_idx) = u * (1.0 - v);
					weights(vid[2], eta_idx) = u * v;
					weights(vid[3], eta_idx) = (1.0 - u) * v;
					return true;
				}
				else {
					for (unsigned int i = 0; i < 4; ++i)
						w[i] = 0.0;
					continue;
				}
			}
			else {
				// 1) identify null space:
				const Eigen::Vector4d nullSpaceBasis = Eigen::Vector4d(
					getSpanDeterminant(pt[1] - eta, pt[3] - eta, pt[2] - eta),
					getSpanDeterminant(pt[0] - eta, pt[2] - eta, pt[3] - eta),
					getSpanDeterminant(pt[1] - eta, pt[0] - eta, pt[3] - eta),
					getSpanDeterminant(pt[0] - eta, pt[1] - eta, pt[2] - eta)
				);

				// nullSpaceBasis.normalize(); // THIS IS HIGHLY UNSTABLE !!!

				// check that the point is not on the bilinear quad:
				double alphaCand = nullSpaceBasis(0) + nullSpaceBasis(1) + nullSpaceBasis(2) + nullSpaceBasis(3);
				double uCand = (nullSpaceBasis(1) + nullSpaceBasis(2)) / alphaCand;
				double vCand = (nullSpaceBasis(2) + nullSpaceBasis(3)) / alphaCand;
				if (uCand <= 1.0 && vCand <= 1.0 && uCand >= 0.0 && vCand >= 0.0) {
					Eigen::Vector4d buvCand((1 - uCand) * (1 - vCand), uCand * (1 - vCand), uCand * vCand, (1 - uCand) * vCand);
					if ((alphaCand * buvCand - nullSpaceBasis).squaredNorm() < epsilon * epsilon) {
						// Then the point is on the bilinear quad.
						w_weights.col(eta_idx).setZero();
						weights.col(eta_idx).setZero();
						weights(vid[0], eta_idx) = buvCand(0);
						weights(vid[1], eta_idx) = buvCand(1);
						weights(vid[2], eta_idx) = buvCand(2);
						weights(vid[3], eta_idx) = buvCand(3);
						return true;
					}
				}

				// the point is NOT on the bilinear quad.
				// 2) solve full rank system:
				double theta[4];
				for (unsigned int i = 0; i < 4; ++i) {
					theta[i] = 2.0 * asin((u[vid[(i) % 4]] - u[vid[(i + 1) % 4]]).norm() / 2.0);
				}

				Eigen::Vector3d N[4];
				for (unsigned int i = 0; i < 4; ++i) {
					N[i] = (pt[(i + 1) % 4] - eta).cross(pt[(i) % 4] - eta);
				}

				Eigen::Vector3d m_quad = -0.5 * (theta[0] * N[0] / N[0].norm() + theta[1] * N[1] / N[1].norm() + theta[2] * N[2] / 
					N[2].norm() + theta[3] * N[3] / N[3].norm());

				w(0) = nullSpaceBasis(1) * getSpanDeterminant(m_quad, pt[2] - eta, pt[3] - eta)
					- nullSpaceBasis(2) * getSpanDeterminant(m_quad, pt[1] - eta, pt[3] - eta)
					+ nullSpaceBasis(3) * getSpanDeterminant(m_quad, pt[1] - eta, pt[2] - eta);
				w(1) = -nullSpaceBasis(0) * getSpanDeterminant(m_quad, pt[2] - eta, pt[3] - eta)
					- nullSpaceBasis(2) * getSpanDeterminant(pt[0] - eta, m_quad, pt[3] - eta)
					+ nullSpaceBasis(3) * getSpanDeterminant(pt[0] - eta, m_quad, pt[2] - eta);
				w(2) = -nullSpaceBasis(0) * getSpanDeterminant(pt[1] - eta, m_quad, pt[3] - eta)
					+ nullSpaceBasis(1) * getSpanDeterminant(pt[0] - eta, m_quad, pt[3] - eta)
					+ nullSpaceBasis(3) * getSpanDeterminant(pt[0] - eta, pt[1] - eta, m_quad);
				w(3) = -nullSpaceBasis(0) * getSpanDeterminant(pt[1] - eta, pt[2] - eta, m_quad)
					+ nullSpaceBasis(1) * getSpanDeterminant(pt[0] - eta, pt[2] - eta, m_quad)
					- nullSpaceBasis(2) * getSpanDeterminant(pt[0] - eta, pt[1] - eta, m_quad);

				w /= nullSpaceBasis.squaredNorm();

				// WE NEED TO FIND THE VALUE TO ADD TO W (the component along nullSpaceBasis) :
				double lambda = 0.0;

				double uCenter, vCenter;
				smoothProjectOnQuad(eta, pt, uCenter, vCenter);

				assert(uCenter >= 0.0);
				assert(uCenter <= 1.0);
				assert(vCenter >= 0.0);
				assert(vCenter <= 1.0);

				std::vector<double> uValues;
				std::vector<double> vValues;
				{
					// SAMPLING FOR SUBMISSION : n = 4
					unsigned int n = 4;

					uValues.clear();
					vValues.clear();

					{
						if (uCenter > 0.0) {
							for (unsigned int i = 0; i < n; ++i) {
								double x = (double)(i) / (double)(n);
								uValues.push_back(x * uCenter);
							}
						}
						uValues.push_back(uCenter);
						if (uCenter < 1.0) {
							for (unsigned int i = 0; i < n; ++i) {
								double x = 1.0 - (double)(i + 1) / (double)(n);
								uValues.push_back(uCenter * x + 1.0 * (1.0 - x));
							}
						}

						if (vCenter > 0.0) {
							for (unsigned int i = 0; i < n; ++i) {
								double x = (double)(i) / (double)(n);
								vValues.push_back(x * vCenter);
							}
						}
						vValues.push_back(vCenter);
						if (vCenter < 1.0) {
							for (unsigned int i = 0; i < n; ++i) {
								double x = (double)(i + 1) / (double)(n);
								vValues.push_back(vCenter * (1.0 - x) + 1.0 * x);
							}
						}
					}
				}

				// compute APPROXIMATE weights :
				Eigen::Vector4d integratedBilinearWeights(0, 0, 0, 0);
				for (unsigned int uIt = 0; uIt < uValues.size(); ++uIt) {
					for (unsigned int vIt = 0; vIt < vValues.size(); ++vIt) {
						double u = uValues[uIt];
						double v = vValues[vIt];

						Eigen::Vector4d bilinearWeights((1 - u) * (1 - v), u * (1 - v), u * v, (1 - u) * v);
						Eigen::Vector3d const& Puv = interpolate_on_quad(pt, u, v);

						{
							// ESTIMATE dB USING ELEMENTARY SOLID ANGLE AT THE POINT
							double du = (uValues[std::min<unsigned int>(uIt + 1, uValues.size() - 1)] - uValues[std::max<int>((int)(uIt)-1, 0)]) / 2.0;
							double dv = (vValues[std::min<unsigned int>(vIt + 1, vValues.size() - 1)] - vValues[std::max<int>((int)(vIt)-1, 0)]) / 2.0; // OK, triple checked
							double dist = (Puv - eta).norm();
							Eigen::Vector3d const& Nuv = interpolated_quad_normal(pt, u, v);
							double dB = (Puv - eta).dot(Nuv) * du * dv / (dist * dist * dist);

							integratedBilinearWeights += (dB / dist) * bilinearWeights;
						}
					}
				}

				// NORM-CORRECT lambda:
				{
					lambda = (integratedBilinearWeights.dot(nullSpaceBasis) * w.squaredNorm()) /
						(nullSpaceBasis.squaredNorm() * integratedBilinearWeights.dot(w)); // with norm correction
				}

				w += lambda * nullSpaceBasis;
			}

			sumWeights += (w[0] + w[1] + w[2] + w[3]);
			w_weights(vid[0], eta_idx) += w[0];
			w_weights(vid[1], eta_idx) += w[1];
			w_weights(vid[2], eta_idx) += w[2];
			w_weights(vid[3], eta_idx) += w[3];
		}
	}

	for (unsigned int v = 0; v < n_vertices; ++v)
		weights(v, eta_idx) = w_weights(v, eta_idx) / sumWeights;

	assert(abs(1. - weights.col(eta_idx).sum()) < 1.e-5);
	return false;
}

bool computeMVCTriQuad(const Eigen::MatrixXd& C, const Eigen::MatrixXi& CF, Eigen::MatrixXd const& eta_m,
	Eigen::MatrixXd& phi)
{
	std::vector<Eigen::Vector3i> triangles;
	std::vector<Eigen::Vector4i> quads;

	bool contains_quads = CF.cols() == 4;

	for (int face_idx = 0; face_idx < CF.rows(); ++face_idx)
	{
		auto const face = CF.row(face_idx);
		Eigen::Vector3d face_points[4];

		bool isTriangle = face.size() == 3 || (contains_quads && face(3) == -1);
		bool isQuad = face.size() == 4;

		if (isTriangle)
		{
			triangles.push_back(Eigen::Vector3i(face(0), face(1), face(2)));
		}
		else if (isQuad)
		{
			quads.push_back(Eigen::Vector4i(face));
		}
		else
		{
			std::cerr << "Unsupported polygon type!\n";
			return false;
		}
	}

	phi.resize(C.rows(), eta_m.rows());
	Eigen::MatrixXd w_weights(C.rows(), eta_m.rows());


	for (int eta_idx = 0; eta_idx < eta_m.rows(); ++eta_idx)
	{
		const Eigen::Vector3d eta = eta_m.row(eta_idx);

		computeMVCTriQuadCoordinates(static_cast<unsigned int>(eta_idx), eta, triangles, quads, C, phi, w_weights);
	}

	return true;
}

void calcNormals(const Eigen::MatrixXd& C, const Eigen::MatrixXi& CF, Eigen::MatrixXd& normals)
{
	normals.resize(CF.rows(), 3);

	for (int i = 0; i < CF.rows(); ++i)
	{
		Eigen::Vector3i index_vector = CF.row(i);

		const Eigen::Vector3d t_0 = C.row(index_vector[0]);
		const Eigen::Vector3d t_1 = C.row(index_vector[1]);
		const Eigen::Vector3d t_2 = C.row(index_vector[2]);

		auto const normal = ((t_1 - t_0).cross(t_2 - t_0)).normalized();

		normals.row(i) = normal;
	}
}

double calc_scaling_factor_tri(Eigen::Vector3d old_tri[3], Eigen::Vector3d new_tri[3])
{
	auto const old_u = old_tri[0] - old_tri[1];
	auto const old_v = old_tri[0] - old_tri[2];

	auto const area = .5 * (old_u.cross(old_v)).stableNorm();

	auto const new_u = new_tri[0] - new_tri[1];
	auto const new_v = new_tri[0] - new_tri[2];

	return std::sqrt(new_u.squaredNorm() * old_v.squaredNorm() - 2. * new_u.dot(new_v) * old_u.dot(old_v) + new_v.squaredNorm() * old_u.squaredNorm()) /
		(2.8284271247461903 * area);
}

void calcScalingFactors(const Eigen::MatrixXd& C, const Eigen::MatrixXd& C_deformed, const Eigen::MatrixXi& CF, Eigen::MatrixXd& normals)
{
	for (int i = 0; i < CF.rows(); ++i)
	{
		const Eigen::Vector3i index_vec = CF.row(i);

		Eigen::Vector3d old_tri[3], new_tri[3];
		for (int k = 0; k < 3; ++k)
		{
			old_tri[k] = C.row(index_vec(k));
			new_tri[k] = C_deformed.row(index_vec(k));
		}

		auto const scaling_factor = calc_scaling_factor_tri(old_tri, new_tri);
		normals.row(i) *= scaling_factor;
	}
}

void calcNewPositionsTriQuad(const Eigen::MatrixXd& C, const Eigen::MatrixXd& C_deformed, const Eigen::MatrixXi & CF,
	const Eigen::MatrixXd& phi, std::vector<double> const& psi_tri, std::vector<Eigen::Vector4d> const& psi_quad, Eigen::MatrixXd& eta_deformed)
{
	// Adding weighted control vertices first
	eta_deformed = phi.transpose() * C_deformed;

	// Precalculate scaling factors of quads
	std::vector<Eigen::Vector4d> sigma_quads;
	for (unsigned int face_idx = 0; face_idx < CF.rows(); ++face_idx)
	{
		auto const face = CF.row(face_idx);

		if (face.size() != 4 || face(3) == -1)
		{
			continue;
		}
		Eigen::Vector3d new_face_points[4], old_face_points[4];
		for (int k = 0; k < face.size(); ++k)
		{
			old_face_points[k] = C.row(face(k));
			new_face_points[k] = C_deformed.row(face(k));
		}

		auto const sigma_q = numerically_approx_sigma_q(old_face_points, new_face_points);
		sigma_quads.push_back(sigma_q);
	}

	// Now adding psi for every face
	unsigned int tri_count = 0, quad_count = 0;
	for (unsigned int eta_idx = 0; eta_idx < eta_deformed.rows(); ++eta_idx)
	{
		Eigen::Vector3d eta = eta_deformed.row(eta_idx);
		unsigned int loc_quad_count = 0;

		for (unsigned int face_idx = 0; face_idx < CF.rows(); ++face_idx)
		{
			auto const face = CF.row(face_idx);
			Eigen::Vector3d new_face_points[4], old_face_points[4];

			for (int k = 0; k < face.size(); ++k)
			{
				old_face_points[k] = C.row(face(k));
				new_face_points[k] = C_deformed.row(face(k));
			}

			if (face.size() == 3 || (face.size() == 4 && face(3) == -1))
			{
				auto const sigma_t = calc_scaling_factor_tri(old_face_points, new_face_points);
				auto const psi_t = psi_tri[tri_count++];
				auto const new_normal = ((new_face_points[1] - new_face_points[0]).cross(new_face_points[2] - new_face_points[0])).normalized();
				eta += sigma_t * psi_t * new_normal;
			}
			else if (face.size() == 4)
			{
				auto const sigma_q = sigma_quads[loc_quad_count++];
				auto const psi_q = psi_quad[quad_count++];
				for (int k = 0; k < 4; ++k)
				{
					auto const new_N_k = (new_face_points[(k + 1) % 4] - new_face_points[k]).cross(new_face_points[(k + 3) % 4] - new_face_points[k]);
					eta += psi_q(k) * sigma_q(k) * new_N_k;
				}
			}
			else
			{
				std::cerr << "QGC supports only triangles and quads! Unsupported face type found!\n";
			}

		}

		eta_deformed.row(eta_idx) = eta;
	}
}

//BGC related code begin
//Pre compute coeff
std::vector<std::vector<double>> precompute_binomials(int max_degree)
{
	std::vector<std::vector<double>> binom(max_degree + 1);
	for (int n = 0; n <= max_degree; ++n)
	{
		binom[n].resize(n + 1);
		binom[n][0] = 1.0;
		//n!/(k!(n-k)!)
		for (int k = 1; k <= n; ++k)
		{
			binom[n][k] = binom[n][k - 1] * (n - k + 1) / k;
		}
	}
	return binom;
}


static std::vector<std::vector<double>> binom = precompute_binomials(5);


Eigen::Vector3d bezier_triangle_interpolate_n(std::vector<Eigen::Vector3d>& control_points, double u, double v, int n)
{
	double w = 1 - u - v;
	
	if (n == 3)
	{
		Eigen::Vector3d result = control_points[0] * (u * u * u) +
			control_points[1] * (3 * u * u * v) +
			control_points[2] * (3 * u * u * w) +
			control_points[3] * (3 * u * v * v) +
			control_points[4] * (6 * u * v * w) +
			control_points[5] * (3 * u * w * w) +
			control_points[6] * (v * v * v) +
			control_points[7] * (3 * v * v * w) +
			control_points[8] * (3 * v * w * w) +
			control_points[9] * (w * w * w);
		return result;
	}
	
	Eigen::Vector3d result = Eigen::Vector3d::Zero();
	int index = 0;

	for (int i = n; i >= 0; i--)
	{
		for (int j = n - i; j >= 0; j--)
		{
			int k = n - i - j;
			//(n!/i!(n-i)!) * ((n-i)!/j!(n-i-j)!)=(n!/(i!j!k!))
			double coeff = binom[n][i] * binom[n - i][j];
			result += control_points[index] * coeff * pow(u, i) * pow(v, j) * pow(w, k);
			++index;
		}
	}
	return result;
}

Eigen::Vector3d bezier_quad_interpolate_n(std::vector<Eigen::Vector3d>& control_points, double u, double v, int m, int n)
{
	
	if (m == 3 && n == 3)
	{
		double l = 1 - u;
		double s = 1 - v;
		Eigen::Vector3d result = control_points[0] * (l * l * l) * (s * s * s) +
			control_points[1] * (3 * u * l * l) * (s * s * s) +
			control_points[2] * (3 * u * u * l) * (s * s * s) +
			control_points[3] * (u * u * u) * (s * s * s) +
			control_points[4] * (l * l * l) * (3 * v * s * s) +
			control_points[5] * (3 * u * l * l) * (3 * v * s * s) +
			control_points[6] * (3 * u * u * l) * (3 * v * s * s) +
			control_points[7] * (u * u * u) * (3 * v * s * s) +
			control_points[8] * (l * l * l) * (3 * v * v * s) +
			control_points[9] * (3 * u * l * l) * (3 * v * v * s) +
			control_points[10] * (3 * u * u * l) * (3 * v * v * s) +
			control_points[11] * (u * u * u) * (3 * v * v * s) +
			control_points[12] * (l * l * l) * (v * v * v) +
			control_points[13] * (3 * u * l * l) * (v * v * v) +
			control_points[14] * (3 * u * u * l) * (v * v * v) +
			control_points[15] * (u * u * u) * (v * v * v);
		return result;
	}
	
	Eigen::Vector3d result = Eigen::Vector3d::Zero();
	int index = 0;

	for (int j = 0; j <= n; ++j)
	{
		for (int i = 0; i <= m; ++i)
		{
			double coeff_u = binom[m][i] * pow(u, i) * pow(1 - u, m - i);
			double coeff_v = binom[n][j] * pow(v, j) * pow(1 - v, n - j);
			result += control_points[index] * coeff_u * coeff_v;
			++index;
		}
	}
	return result;
}

Eigen::Vector3d bezier_triangle_u_tangent_n(std::vector<Eigen::Vector3d>& control_points, double u, double v, int n) {
	if (n == 0)
	{
		return Eigen::Vector3d::Zero();
	}
	
	if (n == 3)
	{
		return 3 * u * u * control_points[0] + 6 * u * v * control_points[1]
			+ (6 * u * (1 - u - v) - 3 * u * u) * control_points[2] + 3 * v * v * control_points[3] + (6 * v * (1 - u - v) - 6 * u * v) * control_points[4]
			+ (3 * (1 - u - v) * (1 - u - v) - 6 * u * (1 - u - v)) * control_points[5] - 3 * v * v * control_points[7]
			- 6 * v * (1 - u - v) * control_points[8] - 3 * (1 - u - v) * (1 - u - v) * control_points[9];
	}
	
	double w = 1 - u - v;
	Eigen::Vector3d result = Eigen::Vector3d::Zero();
	int index = 0;

	for (int i = 0; i <= n; ++i) {
		for (int j = 0; j <= n - i; ++j) {
			int k = n - i - j;
			double coeff = binom[n][i] * binom[n - i][j];
			double du = 0.0;
			if (i > 0) {
				du = i * pow(u, i - 1) * pow(v, j) * pow(w, k);
			}
			if (k > 0) {
				du -= k * pow(u, i) * pow(v, j) * pow(w, k - 1);
			}
			result += control_points[index] * coeff * du;
			++index;
		}
	}
	return result;
}

Eigen::Vector3d bezier_triangle_v_tangent_n(std::vector<Eigen::Vector3d>& control_points, double u, double v, int n) {
	if (n == 0)
	{
		return Eigen::Vector3d::Zero();
	}
	
	if (n == 3)
	{
		return  3 * u * u * control_points[1]
			- 3 * u * u * control_points[2] + 6 * u * v * control_points[3] + (6 * u * (1 - u - v) - 6 * u * v) * control_points[4]
			- (6 * u * (1 - u - v)) * control_points[5] + 3 * v * v * control_points[6] + (6 * v * (1 - u - v) - 3 * v * v) * control_points[7]
			+ (3 * (1 - u - v) * (1 - u - v) - 6 * v * (1 - u - v)) * control_points[8] - 3 * (1 - u - v) * (1 - u - v) * control_points[9];
	}
	
	double w = 1 - u - v;
	Eigen::Vector3d result = Eigen::Vector3d::Zero();
	int index = 0;

	for (int i = n; i >= 0; i--)
	{
		for (int j = n - i; j >= 0; j--)
		{
			int k = n - i - j;
			double coeff = binom[n][i] * binom[n - i][j];
			double dv = 0.0;
			if (j > 0) {
				dv = j * pow(u, i) * pow(v, j - 1) * pow(w, k);
			}
			if (k > 0) {
				dv -= k * pow(u, i) * pow(v, j) * pow(w, k - 1);
			}
			result += control_points[index] * coeff * dv;
			++index;
		}
	}
	return result;
}


Eigen::Vector3d bezier_quad_u_tangent_n(std::vector<Eigen::Vector3d>& control_points, double u, double v, int m, int n) {
	if (m == 0)
	{
		return Eigen::Vector3d::Zero();
	}
	
	if (m == 3 && n == 3)
	{
		double a1, a2, a3, a4, b1, b2, b3, b4;
		a1 = -3 * (1 - u) * (1 - u);
		a2 = 3 * (1 - u) * (1 - u) - 6 * u * (1 - u);
		a3 = 6 * u * (1 - u) - 3 * u * u;
		a4 = 3 * u * u;
		b1 = (1 - v) * (1 - v) * (1 - v);
		b2 = 3 * v * (1 - v) * (1 - v);
		b3 = 3 * v * v * (1 - v);
		b4 = v * v * v;
		return a1 * b1 * control_points[0] + a2 * b1 * control_points[1] + a3 * b1 * control_points[2] + a4 * b1 * control_points[3] +
			a1 * b2 * control_points[4] + a2 * b2 * control_points[5] + a3 * b2 * control_points[6] + a4 * b2 * control_points[7] +
			a1 * b3 * control_points[8] + a2 * b3 * control_points[9] + a3 * b3 * control_points[10] + a4 * b3 * control_points[11] +
			a1 * b4 * control_points[12] + a2 * b4 * control_points[13] + a3 * b4 * control_points[14] + a4 * b4 * control_points[15];
	}
	
	Eigen::Vector3d result = Eigen::Vector3d::Zero();
	int index = 0;

	for (int j = 0; j <= n; ++j) {
		for (int i = 0; i <= m; ++i) {
			double coeff_u = 0.0;
			if (i > 0) {
				coeff_u = binom[m][i] * i * pow(u, i - 1) * pow(1 - u, m - i);
			}
			if (m - i > 0) {
				coeff_u -= binom[m][i] * (m - i) * pow(u, i) * pow(1 - u, m - i - 1);
			}
			double coeff_v = binom[n][j] * pow(v, j) * pow(1 - v, n - j);
			result += control_points[index] * coeff_u * coeff_v;
			++index;
		}
	}
	return result;
}

Eigen::Vector3d bezier_quad_v_tangent_n(std::vector<Eigen::Vector3d>& control_points, double u, double v, int m, int n) {
	if (n == 0)
	{
		return Eigen::Vector3d::Zero();
	}
	
	if (m == 3 && n == 3)
	{
		double a1, a2, a3, a4, b1, b2, b3, b4;
		a1 = (1 - u) * (1 - u) * (1 - u);
		a2 = 3 * u * (1 - u) * (1 - u);
		a3 = 3 * u * u * (1 - u);
		a4 = u * u * u;
		b1 = -3 * (1 - v) * (1 - v);
		b2 = 3 * (1 - v) * (1 - v) - 6 * v * (1 - v);
		b3 = 6 * v * (1 - v) - 3 * v * v;
		b4 = 3 * v * v;
		return a1 * b1 * control_points[0] + a2 * b1 * control_points[1] + a3 * b1 * control_points[2] + a4 * b1 * control_points[3] +
			a1 * b2 * control_points[4] + a2 * b2 * control_points[5] + a3 * b2 * control_points[6] + a4 * b2 * control_points[7] +
			a1 * b3 * control_points[8] + a2 * b3 * control_points[9] + a3 * b3 * control_points[10] + a4 * b3 * control_points[11] +
			a1 * b4 * control_points[12] + a2 * b4 * control_points[13] + a3 * b4 * control_points[14] + a4 * b4 * control_points[15];
	}
	
	Eigen::Vector3d result = Eigen::Vector3d::Zero();
	int index = 0;

	for (int j = 0; j <= n; ++j) {
		for (int i = 0; i <= m; ++i) {
			double coeff_u = binom[m][i] * pow(u, i) * pow(1 - u, m - i);
			double coeff_v = 0.0;
			if (j > 0) {
				coeff_v = binom[n][j] * j * pow(v, j - 1) * pow(1 - v, n - j);
			}
			if (n - j > 0) {
				coeff_v -= binom[n][j] * (n - j) * pow(v, j) * pow(1 - v, n - j - 1);
			}
			result += control_points[index] * coeff_u * coeff_v;
			++index;
		}
	}
	return result;
}


Eigen::VectorXd bezier_triangle_sheet_n(double u, double v, int n) {
	double w = 1 - u - v;
	int num_terms = (n + 1) * (n + 2) / 2;
	Eigen::VectorXd result(num_terms);
	
	if (n == 3)
	{
		result <<
			u * u * u,
			3 * u * u * v,
			3 * u * u * w,
			3 * u * v * v,
			6 * u * v * w,
			3 * u * w * w,
			v* v* v,
			3 * v * v * w,
			3 * v * w * w,
			w* w* w;
		return result;
	}
	
	int index = 0;

	for (int i = n; i >= 0; i--)
	{
		for (int j = n - i; j >= 0; j--)
		{
			int k = n - i - j;
			double coeff = binom[n][i] * binom[n - i][j];
			result(index) = coeff * pow(u, i) * pow(v, j) * pow(w, k);
			++index;
		}
	}
	return result;
}


Eigen::VectorXd bezier_triangle_sheet_u_n(double u, double v, int n)
{
	double w = 1 - u - v;
	int num_terms = (n + 1) * (n + 2) / 2;
	Eigen::VectorXd result(num_terms);
	
	if (n == 3)
	{
		result <<
			3 * u * u,
			6 * u * v,
			6 * u * (1 - u - v) - 3 * u * u,
			3 * v * v,
			6 * v * (1 - u - v) - 6 * u * v,
			3 * (1 - u - v) * (1 - u - v) - 6 * u * (1 - u - v),
			0,
			-3 * v * v,
			-6 * v * (1 - u - v),
			-3 * (1 - u - v) * (1 - u - v);
		return result;
	}
	
	int index = 0;

	for (int i = n; i >= 0; i--)
	{
		for (int j = n - i; j >= 0; j--)
		{
			int k = n - i - j;
			double coeff = binom[n][i] * binom[n - i][j];
			double du = i * pow(u, i - 1) * pow(v, j) * pow(w, k) - k * pow(u, i) * pow(v, j) * pow(w, k - 1);
			result(index) = coeff * du;
			++index;
		}
	}
	return result;
}

Eigen::VectorXd bezier_triangle_sheet_v_n(double u, double v, int n)
{
	double w = 1 - u - v;
	int num_terms = (n + 1) * (n + 2) / 2;
	Eigen::VectorXd result(num_terms);
	int index = 0;
	
	if (n == 3)
	{
		result <<
			0,
			3 * u * u,
			-3 * u * u,
			6 * u * v,
			6 * u * (1 - u - v) - 6 * u * v,
			-6 * u * (1 - u - v),
			3 * v * v,
			6 * v * (1 - u - v) - 3 * v * v,
			3 * (1 - u - v) * (1 - u - v) - 6 * v * (1 - u - v),
			-3 * (1 - u - v) * (1 - u - v);
		return result;
	}
	
	for (int i = n; i >= 0; i--)
	{
		for (int j = n - i; j >= 0; j--)
		{
			int k = n - i - j;
			double coeff = binom[n][i] * binom[n - i][j];
			double dv = j * pow(u, i) * pow(v, j - 1) * pow(w, k) - k * pow(u, i) * pow(v, j) * pow(w, k - 1);
			result(index) = coeff * dv;
			++index;
		}
	}
	return result;
}
Eigen::VectorXd mergeVectors_triangle(const Eigen::VectorXd& vec1, const Eigen::VectorXd& vec2, int dim)
{
	int numPointBezierTriangle = (dim + 1) * (dim + 2) / 2;
	int numCrossBezierTriangle = numPointBezierTriangle * (numPointBezierTriangle - 1) / 2;
	Eigen::VectorXd merged(numCrossBezierTriangle);
	int index = 0;
	for (int i = 0; i < numPointBezierTriangle; ++i)
	{
		for (int j = i + 1; j < numPointBezierTriangle; ++j)
		{
			merged(index) = vec1(i) * vec2(j) - vec1(j) * vec2(i);
			++index;
		}
	}
	return merged;
}


Eigen::VectorXd bezier_quad_sheet_n(double u, double v, int m, int n)
{
	
	if (m == 3 && n == 3)
	{
		double l = 1 - u;
		double s = 1 - v;
		Eigen::VectorXd result(16);

		result <<
			(l * l * l) * (s * s * s),
			(3 * u * l * l)* (s * s * s),
			(3 * u * u * l)* (s * s * s),
			(u * u * u)* (s * s * s),
			(l * l * l)* (3 * v * s * s),
			(3 * u * l * l)* (3 * v * s * s),
			(3 * u * u * l)* (3 * v * s * s),
			(u * u * u)* (3 * v * s * s),
			(l * l * l)* (3 * v * v * s),
			(3 * u * l * l)* (3 * v * v * s),
			(3 * u * u * l)* (3 * v * v * s),
			(u * u * u)* (3 * v * v * s),
			(l * l * l)* (v * v * v),
			(3 * u * l * l)* (v * v * v),
			(3 * u * u * l)* (v * v * v),
			(u * u * u)* (v * v * v);
		return result;
	}
	
	int num_terms = (m + 1) * (n + 1);
	Eigen::VectorXd result(num_terms);
	int index = 0;

	for (int j = 0; j <= n; ++j) {
		for (int i = 0; i <= m; ++i) {
			double coeff_u = binom[m][i] * pow(u, i) * pow(1 - u, m - i);
			double coeff_v = binom[n][j] * pow(v, j) * pow(1 - v, n - j);
			result(index) = coeff_u * coeff_v;
			++index;
		}
	}
	return result;
}
Eigen::VectorXd bezier_quad_sheet_u_n(double u, double v, int m, int n) {
	int num_terms = (m + 1) * (n + 1);
	Eigen::VectorXd result(num_terms);
	
	if (m == 3 && n == 3)
	{
		double a1, a2, a3, a4, b1, b2, b3, b4;
		a1 = -3 * (1 - u) * (1 - u);
		a2 = 3 * (1 - u) * (1 - u) - 6 * u * (1 - u);
		a3 = 6 * u * (1 - u) - 3 * u * u;
		a4 = 3 * u * u;
		b1 = (1 - v) * (1 - v) * (1 - v);
		b2 = 3 * v * (1 - v) * (1 - v);
		b3 = 3 * v * v * (1 - v);
		b4 = v * v * v;

		result <<
			a1 * b1,
			a2* b1,
			a3* b1,
			a4* b1,
			a1* b2,
			a2* b2,
			a3* b2,
			a4* b2,
			a1* b3,
			a2* b3,
			a3* b3,
			a4* b3,
			a1* b4,
			a2* b4,
			a3* b4,
			a4* b4;
		return result;
	}
	
	int index = 0;

	for (int i = 0; i <= m; ++i) {
		for (int j = 0; j <= n; ++j) {
			double coeff_u = binom[m][i];
			double coeff_v = binom[n][j] * pow(v, j) * pow(1 - v, n - j);
			double du = i * pow(u, i - 1) * pow(1 - u, m - i) - (m - i) * pow(u, i) * pow(1 - u, m - i - 1);
			result(index) = coeff_u * coeff_v * du;
			++index;
		}
	}
	return result;
}
Eigen::VectorXd bezier_quad_sheet_v_n(double u, double v, int m, int n) {
	int num_terms = (m + 1) * (n + 1);
	Eigen::VectorXd result(num_terms);
	
	if (m == 3 && n == 3)
	{
		double a1, a2, a3, a4, b1, b2, b3, b4;
		a1 = (1 - u) * (1 - u) * (1 - u);
		a2 = 3 * u * (1 - u) * (1 - u);
		a3 = 3 * u * u * (1 - u);
		a4 = u * u * u;
		b1 = -3 * (1 - v) * (1 - v);
		b2 = 3 * (1 - v) * (1 - v) - 6 * v * (1 - v);
		b3 = 6 * v * (1 - v) - 3 * v * v;
		b4 = 3 * v * v;

		result <<
			a1 * b1,
			a2* b1,
			a3* b1,
			a4* b1,
			a1* b2,
			a2* b2,
			a3* b2,
			a4* b2,
			a1* b3,
			a2* b3,
			a3* b3,
			a4* b3,
			a1* b4,
			a2* b4,
			a3* b4,
			a4* b4;

		return result;
	}
	
	int index = 0;

	for (int i = 0; i <= m; ++i) {
		for (int j = 0; j <= n; ++j) {
			double coeff_u = binom[m][i] * pow(u, i) * pow(1 - u, m - i);
			double coeff_v = binom[n][j];
			double dv = j * pow(v, j - 1) * pow(1 - v, n - j) - (n - j) * pow(v, j) * pow(1 - v, n - j - 1);
			result(index) = coeff_u * coeff_v * dv;
			++index;
		}
	}
	return result;
}

Eigen::VectorXd mergeVectors_quad(const Eigen::VectorXd& vec1, const Eigen::VectorXd& vec2, int dim)
{
	int num_terms = (dim + 1) * (dim + 1);
	int num_cross = num_terms * (num_terms - 1) / 2;
	Eigen::VectorXd merged(num_cross);
	int index = 0;
	for (int i = 0; i < num_terms; ++i)
	{
		for (int j = i + 1; j < num_terms; ++j)
		{
			merged(index) = vec1(i) * vec2(j) - vec1(j) * vec2(i);
			++index;
		}
	}
	return merged;
}

// Give a good initial guess
Eigen::Vector3d closestPointOnTriangle(const Eigen::Vector3d& point, const Eigen::Vector3d& a, const Eigen::Vector3d& b, const Eigen::Vector3d& c, double& u, double& v, double& w)
{

	Eigen::Vector3d ab = b - a;
	Eigen::Vector3d ac = c - a;
	Eigen::Vector3d normal = ab.cross(ac).normalized();
	double distance = (point - a).dot(normal);
	Eigen::Vector3d projection = point - normal * distance;
	Eigen::Vector3d ap = projection - a;
	Eigen::Vector3d bp = projection - b;
	Eigen::Vector3d cp = projection - c;
	double areaABC = ab.cross(ac).norm();
	double areaPBC = bp.cross(cp).norm();
	double areaPCA = cp.cross(ap).norm();
	double areaPAB = ap.cross(bp).norm();
	double bary_u = areaPBC / areaABC;
	double bary_v = areaPCA / areaABC;
	double bary_w = areaPAB / areaABC;
	if (bary_u >= 0.0 && bary_v >= 0.0 && bary_w >= 0.0 && bary_u <= 1.0 && bary_v <= 1.0 && bary_w <= 1.0)
	{
		u = bary_u;
		v = bary_v;
		w = bary_w;
		return projection;
	}
	auto closestPointOnSegment = [](const Eigen::Vector3d& p, const Eigen::Vector3d& a, const Eigen::Vector3d& b, double& t0)
		{
			Eigen::Vector3d ab = b - a;
			Eigen::Vector3d ap = p - a;
			double t = ap.dot(ab) / ab.dot(ab);
			t = std::max(0.0, std::min(1.0, t));
			t0 = t;
			return a + ab * t;
		};
	double t1, t2, t3;
	Eigen::Vector3d closestOnAB = closestPointOnSegment(point, a, b, t1);
	Eigen::Vector3d closestOnAC = closestPointOnSegment(point, a, c, t2);
	Eigen::Vector3d closestOnBC = closestPointOnSegment(point, b, c, t3);
	double distToAB = (closestOnAB - point).squaredNorm();
	double distToAC = (closestOnAC - point).squaredNorm();
	double distToBC = (closestOnBC - point).squaredNorm();
	if (distToAB <= distToAC && distToAB <= distToBC)
	{
		u = (1 - t1);
		v = t1;
		w = 0;
		return closestOnAB;
	}
	else if (distToAC <= distToAB && distToAC <= distToBC)
	{
		u = (1 - t2);
		v = 0;
		w = t2;
		return closestOnAC;
	}
	else
	{
		u = 0;
		v = (1 - t3);
		w = t3;
		return closestOnBC;
	}
}



Eigen::Vector3d closestPointOnBezierSurface(const Eigen::Vector3d& point, std::vector<Eigen::Vector3d>& control_points, double& u0, double& v0, double stepSize, int maxIterations, int dim)
{
	double u = u0;
	double v = v0;

	Eigen::Vector3d a1 = control_points[0];
	Eigen::Vector3d b1 = control_points[dim];
	Eigen::Vector3d c1 = control_points[(dim + 1) * dim];

	Eigen::Vector3d a2 = control_points[dim];
	Eigen::Vector3d b2 = control_points[(dim + 1) * (dim + 1) - 1];
	Eigen::Vector3d c2 = control_points[(dim + 1) * dim];
	double u1, v1, w1, u2, v2, w2;
	Eigen::Vector3d closest1 = closestPointOnTriangle(point, a1, b1, c1, u1, v1, w1);
	Eigen::Vector3d closest2 = closestPointOnTriangle(point, a2, b2, c2, u2, v2, w2);
	double dist1 = (closest1 - point).squaredNorm();
	double dist2 = (closest2 - point).squaredNorm();

	Eigen::Vector3d closest_point;
	if (dist1 < dist2)
	{
		u = v1;
		v = w1;
	}
	else
	{
		u = 1 - w2;
		v = 1 - u2;
	}


	double epsilon = 1e-5;

	for (int i = 0; i < maxIterations; ++i)
	{
		Eigen::Vector3d p = bezier_quad_interpolate_n(control_points, u, v, dim, dim);

		Eigen::Vector3d gradient_u = bezier_quad_u_tangent_n(control_points, u, v, dim, dim);
		Eigen::Vector3d gradient_v = bezier_quad_v_tangent_n(control_points, u, v, dim, dim);


		Eigen::Vector3d gradient = (point - p).dot(gradient_u) * gradient_u + (point - p).dot(gradient_v) * gradient_v;
		if (gradient.norm() < epsilon)
		{
			break;
		}

		u += stepSize * (point - p).dot(gradient_u) / gradient_u.norm();
		v += stepSize * (point - p).dot(gradient_v) / gradient_v.norm();

		u = std::min(std::max(u, 0.05), 0.95);
		v = std::min(std::max(v, 0.05), 0.95);
	}
	u0 = u;
	v0 = v;
	return bezier_quad_interpolate_n(control_points, u0, v0, dim, dim);
}
Eigen::VectorXd computePhiAndPsiForOneBezierTriangle(const Eigen::Vector3d& eta, std::vector<Eigen::Vector3d>& control_points, std::vector<Eigen::Vector3d>& vertex_normals, int dim)
{

	int numPointBezierTriangle = (dim + 1) * (dim + 2) / 2;
	Eigen::VectorXd Phi(2 * numPointBezierTriangle);
	Phi.fill(0);

	auto Proc = [&](Eigen::Vector3d u_triangle, Eigen::Vector3d v_triangle)
		{
			double u_tri[3], v_tri[3], w_tri[3];
			for (int j = 0; j < 3; ++j)
			{
				u_tri[j] = u_triangle(j);
				v_tri[j] = v_triangle(j);
				w_tri[j] = 1.0 - u_tri[j] - v_tri[j];
			}

			auto const u_avg = (u_tri[0] + u_tri[1] + u_tri[2]) / 3.;
			auto const v_avg = (v_tri[0] + v_tri[1] + v_tri[2]) / 3.;
			auto const w_avg = 1.0 - u_avg - v_avg;

			Eigen::Vector3d tessellated_tri[3];
			for (int k = 0; k < 3; ++k)
			{
				tessellated_tri[k] = bezier_triangle_interpolate_n(control_points, u_tri[k], v_tri[k], dim);
			}

			Eigen::Vector3d Nt = (tessellated_tri[1] - tessellated_tri[0]).cross(tessellated_tri[2] - tessellated_tri[0]);
			double NtNorm = Nt.norm();
			double At = NtNorm / 2.0;
			Nt /= NtNorm;
			if (At < 1e-15) return;

			double psi_tri = 0.0;
			Eigen::Vector3d e[3];    double e_norm[3];   Eigen::Vector3d e_normalized[3];    double R[3];    Eigen::Vector3d d[3];    double d_norm[3];     double C[3];     Eigen::Vector3d J[3];
			for (unsigned int v = 0; v < 3u; ++v) e[v] = tessellated_tri[v] - eta;
			for (unsigned int v = 0; v < 3u; ++v) e_norm[v] = e[v].norm();
			for (unsigned int v = 0; v < 3u; ++v) e_normalized[v] = e[v] / e_norm[v];

			auto const omega_tri = get_signed_solid_angle(e_normalized[0], e_normalized[1], e_normalized[2]);
			auto const signed_solid_angle = omega_tri / (4.f * M_PI);
			auto const signed_volume = (e[0].cross(e[1])).dot(e[2]) / 6.0;

			for (unsigned int v = 0; v < 3; ++v) R[v] = e_norm[(v + 1) % 3] + e_norm[(v + 2) % 3];
			for (unsigned int v = 0; v < 3; ++v) d[v] = tessellated_tri[(v + 1) % 3] - tessellated_tri[(v + 2) % 3];
			for (unsigned int v = 0; v < 3; ++v) d_norm[v] = d[v].norm();
			for (unsigned int v = 0; v < 3; ++v) C[v] = std::log((R[v] + d_norm[v]) / (R[v] - d_norm[v])) / (4.0 * M_PI * d_norm[v]);

			Eigen::Vector3d Pt(-signed_solid_angle * Nt);
			for (unsigned int v = 0; v < 3; ++v) Pt += Nt.cross(C[v] * d[v]);
			for (unsigned int v = 0; v < 3; ++v) J[v] = e[(v + 2) % 3].cross(e[(v + 1) % 3]);

			psi_tri = -3.0 * signed_solid_angle * signed_volume / At;
			for (unsigned int v = 0; v < 3; ++v) psi_tri -= C[v] * J[v].dot(Nt);

			auto const b_avg = bezier_triangle_sheet_n(u_avg, v_avg, dim);
			auto const phi_bezier = b_avg * omega_tri / (4.f * M_PI);
			auto const interpolated_normal = bezier_triangle_interpolate_n(vertex_normals, u_avg, v_avg, dim);
			auto const psi_bezier = psi_tri * b_avg / interpolated_normal.norm();

			for (unsigned int k = 0; k < numPointBezierTriangle; ++k)
			{
				Phi(k) += phi_bezier(k);
				Phi(k + numPointBezierTriangle) += psi_bezier(k);
			}
		};

	//Uniform tessellation was found to be feasible for the Bézier triangle, although we explored some potentially smarter alternatives,
	// which did not yield better results.
	const int divisions = 5;
	const double stepU = 1.0 / divisions;
	const double stepV = 1.0 / divisions;
	for (int i = 0; i <= divisions; ++i)
	{
		for (int j = 0; j <= divisions - i; ++j)
		{
			double u1 = i * stepU;
			double v1 = j * stepV;
			double u2 = (i + 1) * stepU;
			double v2 = j * stepV;
			double u3 = i * stepU;
			double v3 = (j + 1) * stepV;

			if (u2 + v2 <= 1 && u3 + v3 <= 1)
			{
				Proc({ u1, u2, u3 }, { v1, v2, v3 });
			}
			if (u2 + v3 <= 1)
			{
				Proc({ u3, u2, u2 }, { v3, v2, v3 });
			}
		}
	}

	return Phi;
}


Eigen::VectorXd computePhiAndPsiForOneBezierSurface(const Eigen::Vector3d& eta, std::vector<Eigen::Vector3d>& control_points, std::vector<Eigen::Vector3d>& vertex_normals, int dim)
{
	
	double u_projected = 0.5, v_projected = 0.5;

	// project eta to the Bezier patch
	double stepSize = 0.05;
	int maxIterations = 50;
	Eigen::Vector3d closest_point = closestPointOnBezierSurface(eta, control_points, u_projected, v_projected, stepSize, maxIterations, dim);
	Eigen::VectorXd Phi;
	int numPointBezierQuad = (dim + 1) * (dim + 1);
	Phi.resize(2 * numPointBezierQuad);
	Phi.fill(0);

	const int n = 2;
	const double m = 3.;
	const int N = 2 * n + 2;
	double u[N + 1], v[N + 1];

	for (int i = 0; i < N + 1; ++i)
	{
		if (i < N / 2)
		{
			const double factor = 1. - std::pow((static_cast<double>(N / 2) - static_cast<double>(i)) / static_cast<double>(N / 2), m);
			u[i] = factor * u_projected;
			v[i] = factor * v_projected;
		}
		else if (i == N / 2)
		{
			u[i] = u_projected;
			v[i] = v_projected;
		}
		else
		{
			const double factor = 1. - std::pow((static_cast<double>(i) - static_cast<double>(N / 2)) / static_cast<double>(N / 2), m);
			u[i] = (u_projected - 1.) * factor + 1.;
			v[i] = (v_projected - 1.) * factor + 1.;
		}
	}

	
	std::function<void(Eigen::Vector3d, Eigen::Vector3d)> Proc = [&](Eigen::Vector3d u_triangle, Eigen::Vector3d v_triangle)
		{
			double u_tri[3], v_tri[3];
			for (int j = 0; j < 3; ++j)
			{
				u_tri[j] = u_triangle(j);
				v_tri[j] = v_triangle(j);
			}

			if (u_tri[0] < 0 || u_tri[0] > 1 || u_tri[1] < 0 || u_tri[1] > 1 || u_tri[2] < 0 || u_tri[2] > 1 ||
				v_tri[0] < 0 || v_tri[0] > 1 || v_tri[1] < 0 || v_tri[1] > 1 || v_tri[2] < 0 || v_tri[2] > 1)
			{
				return;
			}
			auto const u_avg = (u_tri[0] + u_tri[1] + u_tri[2]) / 3.;
			auto const v_avg = (v_tri[0] + v_tri[1] + v_tri[2]) / 3.;

			Eigen::Vector3d tessellated_tri[3];
			for (int k = 0; k < 3; ++k)
			{
				tessellated_tri[k] = bezier_quad_interpolate_n(control_points, u_tri[k], v_tri[k], dim, dim);
			}
			Eigen::Vector3d Nt = (tessellated_tri[1] - tessellated_tri[0]).cross(tessellated_tri[2] - tessellated_tri[0]);
			double NtNorm = Nt.norm();
			double At = NtNorm / 2.0;
			Nt /= NtNorm;
			if (At < 1e-15)
			{
				return;
			}

			double psi_tri = 0.0;

			Eigen::Vector3d e[3];    double e_norm[3];   Eigen::Vector3d e_normalized[3];    double R[3];    Eigen::Vector3d d[3];    double d_norm[3];     double C[3];     Eigen::Vector3d J[3];
			for (unsigned int v = 0; v < 3u; ++v) e[v] = tessellated_tri[v] - eta;
			for (unsigned int v = 0; v < 3u; ++v)
			{
				e_norm[v] = e[v].norm();
			}

			for (unsigned int v = 0; v < 3u; ++v) e_normalized[v] = e[v] / e_norm[v];

			auto const omega_tri = get_signed_solid_angle(e_normalized[0], e_normalized[1], e_normalized[2]);
			auto const signed_solid_angle = omega_tri / (4.f * M_PI);
			auto const signed_volume = (e[0].cross(e[1])).dot(e[2]) / 6.0;
			for (unsigned int v = 0; v < 3; ++v) R[v] = e_norm[(v + 1) % 3] + e_norm[(v + 2) % 3];
			for (unsigned int v = 0; v < 3; ++v) d[v] = tessellated_tri[(v + 1) % 3] - tessellated_tri[(v + 2) % 3];
			for (unsigned int v = 0; v < 3; ++v) d_norm[v] = d[v].norm();
			for (unsigned int v = 0; v < 3; ++v) C[v] = std::log((R[v] + d_norm[v]) / (R[v] - d_norm[v])) / (4.0 * M_PI * d_norm[v]);

			Eigen::Vector3d Pt(-signed_solid_angle * Nt);
			for (unsigned int v = 0; v < 3; ++v) Pt += Nt.cross(C[v] * d[v]);
			for (unsigned int v = 0; v < 3; ++v) J[v] = e[(v + 2) % 3].cross(e[(v + 1) % 3]);

			psi_tri = -3.0 * signed_solid_angle * signed_volume / At;
			for (unsigned int v = 0; v < 3; ++v) psi_tri -= C[v] * J[v].dot(Nt);

			auto const b_avg = bezier_quad_sheet_n(u_avg, v_avg, dim, dim);
			auto const phi_bezier = b_avg * omega_tri / (4.f * M_PI);
			auto const interpolated_normal = bezier_quad_interpolate_n(vertex_normals, u_avg, v_avg, dim, dim);
			auto const psi_bezier = psi_tri * b_avg / interpolated_normal.norm();


			for (unsigned int k = 0; k < numPointBezierQuad; ++k)
			{
				auto const phi_k = phi_bezier(k);
				Phi(k) += phi_k;
				Phi(k + numPointBezierQuad) += psi_bezier(k);
			}
		};

	for (int v_it = 0; v_it <= n; ++v_it)
	{
		for (int u_it = v_it; u_it <= (2 * n - v_it); ++u_it)
		{
			if ((u_it + v_it) % 2 == 0)
			{
				Proc({ u[u_it], u[u_it + 2], u[u_it + 1] }, { v[v_it], v[v_it], v[v_it + 1] });
				Proc({ u[u_it], u[u_it + 1], u[u_it + 2] }, { v[N - v_it], v[N - v_it - 1], v[N - v_it] });
				Proc({ u[v_it], u[v_it + 1], u[v_it] }, { v[u_it], v[u_it + 1], v[u_it + 2] });
				Proc({ u[N - v_it], u[N - v_it], u[N - v_it - 1] }, { v[u_it], v[u_it + 2], v[u_it + 1] });
			}
			else
			{
				Proc({ u[u_it], u[u_it + 1], u[u_it + 2] }, { v[v_it + 1], v[v_it], v[v_it + 1] });
				Proc({ u[u_it], u[u_it + 2], u[u_it + 1] }, { v[N - v_it - 1], v[N - v_it - 1], v[N - v_it] });
				Proc({ u[v_it + 1], u[v_it + 1], u[v_it] }, { v[u_it], v[u_it + 2], v[u_it + 1] });
				Proc({ u[N - v_it - 1], u[N - v_it], u[N - v_it - 1] }, { v[u_it], v[u_it + 1], v[u_it + 2] });
			}
		}
	}


	return Phi;
}


Eigen::VectorXd computePhiAndPsiForOneBezierTriangleCrossProduct(const Eigen::Vector3d& eta, std::vector<Eigen::Vector3d>& control_points, int dim)
{
	
	int numPointBezierTriangle = (dim + 1) * (dim + 2) / 2;
	int numCrossBezierTriangle = numPointBezierTriangle * (numPointBezierTriangle - 1) / 2;
	Eigen::VectorXd Phi(numPointBezierTriangle + numCrossBezierTriangle);
	Phi.fill(0);

	auto Proc = [&](Eigen::Vector3d u_triangle, Eigen::Vector3d v_triangle)
		{
			double u_tri[3], v_tri[3], w_tri[3];
			for (int j = 0; j < 3; ++j)
			{
				u_tri[j] = u_triangle(j);
				v_tri[j] = v_triangle(j);
				w_tri[j] = 1.0 - u_tri[j] - v_tri[j];
			}
			auto const u_avg = (u_tri[0] + u_tri[1] + u_tri[2]) / 3.;
			auto const v_avg = (v_tri[0] + v_tri[1] + v_tri[2]) / 3.;
			auto const w_avg = 1.0 - u_avg - v_avg;

			Eigen::Vector3d tessellated_tri[3];
			for (int k = 0; k < 3; ++k)
			{
				tessellated_tri[k] = bezier_triangle_interpolate_n(control_points, u_tri[k], v_tri[k], dim);
			}

			Eigen::Vector3d Nt = (tessellated_tri[1] - tessellated_tri[0]).cross(tessellated_tri[2] - tessellated_tri[0]);
			double NtNorm = Nt.norm();
			double At = NtNorm / 2.0;
			Nt /= NtNorm;
			if (At < 1e-15) return;

			double psi_tri = 0.0;
			Eigen::Vector3d e[3];    double e_norm[3];   Eigen::Vector3d e_normalized[3];    double R[3];    Eigen::Vector3d d[3];    double d_norm[3];     double C[3];     Eigen::Vector3d J[3];
			for (unsigned int v = 0; v < 3u; ++v) e[v] = tessellated_tri[v] - eta;
			for (unsigned int v = 0; v < 3u; ++v) e_norm[v] = e[v].norm();
			for (unsigned int v = 0; v < 3u; ++v) e_normalized[v] = e[v] / e_norm[v];

			auto const omega_tri = get_signed_solid_angle(e_normalized[0], e_normalized[1], e_normalized[2]);
			auto const signed_solid_angle = omega_tri / (4.f * M_PI);
			auto const signed_volume = (e[0].cross(e[1])).dot(e[2]) / 6.0;

			for (unsigned int v = 0; v < 3; ++v) R[v] = e_norm[(v + 1) % 3] + e_norm[(v + 2) % 3];
			for (unsigned int v = 0; v < 3; ++v) d[v] = tessellated_tri[(v + 1) % 3] - tessellated_tri[(v + 2) % 3];
			for (unsigned int v = 0; v < 3; ++v) d_norm[v] = d[v].norm();
			for (unsigned int v = 0; v < 3; ++v) C[v] = std::log((R[v] + d_norm[v]) / (R[v] - d_norm[v])) / (4.0 * M_PI * d_norm[v]);

			Eigen::Vector3d Pt(-signed_solid_angle * Nt);
			for (unsigned int v = 0; v < 3; ++v) Pt += Nt.cross(C[v] * d[v]);
			for (unsigned int v = 0; v < 3; ++v) J[v] = e[(v + 2) % 3].cross(e[(v + 1) % 3]);

			psi_tri = -3.0 * signed_solid_angle * signed_volume / At;
			for (unsigned int v = 0; v < 3; ++v) psi_tri -= C[v] * J[v].dot(Nt);

			Eigen::VectorXd vec_u = bezier_triangle_sheet_u_n(u_avg, v_avg, dim);
			Eigen::VectorXd vec_v = bezier_triangle_sheet_v_n(u_avg, v_avg, dim);
			Eigen::VectorXd merged = mergeVectors_triangle(vec_u, vec_v, dim);
			auto const b_avg = merged;
			auto const phi_avg = bezier_triangle_sheet_n(u_avg, v_avg, dim);
			auto const phi_bezier = phi_avg * omega_tri / (4.f * M_PI);
			auto const interpolated_u = bezier_triangle_u_tangent_n(control_points, u_avg, v_avg, dim);
			auto const interpolated_v = bezier_triangle_v_tangent_n(control_points, u_avg, v_avg, dim);
			auto const interpolated_normal = interpolated_u.cross(interpolated_v);
			auto const psi_bezier = psi_tri * b_avg / interpolated_normal.norm();

			for (unsigned int k = 0; k < numPointBezierTriangle; ++k)
			{
				Phi(k) += phi_bezier(k);
			}
			for (unsigned int k = 0; k < numCrossBezierTriangle; ++k)
			{
				Phi(k + numPointBezierTriangle) += psi_bezier(k);
			}
		};
	const int divisions = 5;
	const double stepU = 1.0 / divisions;
	const double stepV = 1.0 / divisions;
	for (int i = 0; i <= divisions; ++i)
	{
		for (int j = 0; j <= divisions - i; ++j)
		{
			double u1 = i * stepU;
			double v1 = j * stepV;
			double u2 = (i + 1) * stepU;
			double v2 = j * stepV;
			double u3 = i * stepU;
			double v3 = (j + 1) * stepV;

			if (u2 + v2 <= 1 && u3 + v3 <= 1)
			{
				Proc({ u1, u2, u3 }, { v1, v2, v3 });
			}
			if (u2 + v3 <= 1)
			{
				Proc({ u3, u2, u2 }, { v3, v2, v3 }); 
			}
		}
	}

	return Phi;
}

Eigen::VectorXd computePhiAndPsiForOneBezierSurfaceCrossProduct(const Eigen::Vector3d& eta, std::vector<Eigen::Vector3d>& control_points, int dim)
{
	double u_projected = 0.5, v_projected = 0.5;

	double stepSize = 0.05;
	int maxIterations = 50;
	Eigen::Vector3d closest_point = closestPointOnBezierSurface(eta, control_points, u_projected, v_projected, stepSize, maxIterations, dim);
	Eigen::VectorXd Phi;
	int numPointBezierQuad = (dim + 1) * (dim + 1);
	int numCrossBezierQuad = numPointBezierQuad * (numPointBezierQuad - 1) / 2;
	Phi.resize(numPointBezierQuad + numCrossBezierQuad);
	Phi.fill(0);

	const int n = 2;
	const double m = 3.;
	const int N = 2 * n + 2;
	double u[N + 1], v[N + 1];

	for (int i = 0; i < N + 1; ++i)
	{
		if (i < N / 2)
		{
			const double factor = 1. - std::pow((static_cast<double>(N / 2) - static_cast<double>(i)) / static_cast<double>(N / 2), m);
			u[i] = factor * u_projected;
			v[i] = factor * v_projected;
		}
		else if (i == N / 2)
		{
			u[i] = u_projected;
			v[i] = v_projected;
		}
		else
		{
			const double factor = 1. - std::pow((static_cast<double>(i) - static_cast<double>(N / 2)) / static_cast<double>(N / 2), m);
			u[i] = (u_projected - 1.) * factor + 1.;
			v[i] = (v_projected - 1.) * factor + 1.;
		}
	}

	
	std::function<void(Eigen::Vector3d, Eigen::Vector3d)> Proc = [&](Eigen::Vector3d u_triangle, Eigen::Vector3d v_triangle)
		{
			double u_tri[3], v_tri[3];
			for (int j = 0; j < 3; ++j)
			{
				u_tri[j] = u_triangle(j);
				v_tri[j] = v_triangle(j);
			}
			if (u_tri[0] < 0 || u_tri[0] > 1 || u_tri[1] < 0 || u_tri[1] > 1 || u_tri[2] < 0 || u_tri[2] > 1 ||
				v_tri[0] < 0 || v_tri[0] > 1 || v_tri[1] < 0 || v_tri[1] > 1 || v_tri[2] < 0 || v_tri[2] > 1)
			{
				return;
			}
			auto const u_avg = (u_tri[0] + u_tri[1] + u_tri[2]) / 3.;
			auto const v_avg = (v_tri[0] + v_tri[1] + v_tri[2]) / 3.;

			Eigen::Vector3d tessellated_tri[3];
			for (int k = 0; k < 3; ++k)
			{
				tessellated_tri[k] = bezier_quad_interpolate_n(control_points, u_tri[k], v_tri[k], dim, dim);
			}
			Eigen::Vector3d Nt = (tessellated_tri[1] - tessellated_tri[0]).cross(tessellated_tri[2] - tessellated_tri[0]);
			double NtNorm = Nt.norm();
			double At = NtNorm / 2.0;
			Nt /= NtNorm;
			if (At < 1e-15)
			{
				return;
			}

			double psi_tri = 0.0;

			Eigen::Vector3d e[3];    double e_norm[3];   Eigen::Vector3d e_normalized[3];    double R[3];    Eigen::Vector3d d[3];    double d_norm[3];     double C[3];     Eigen::Vector3d J[3];
			for (unsigned int v = 0; v < 3u; ++v) e[v] = tessellated_tri[v] - eta;
			for (unsigned int v = 0; v < 3u; ++v)
			{
				e_norm[v] = e[v].norm();
			}

			for (unsigned int v = 0; v < 3u; ++v) e_normalized[v] = e[v] / e_norm[v];

			auto const omega_tri = get_signed_solid_angle(e_normalized[0], e_normalized[1], e_normalized[2]);
			auto const signed_solid_angle = omega_tri / (4.f * M_PI);
			auto const signed_volume = (e[0].cross(e[1])).dot(e[2]) / 6.0;


			for (unsigned int v = 0; v < 3; ++v) R[v] = e_norm[(v + 1) % 3] + e_norm[(v + 2) % 3];
			for (unsigned int v = 0; v < 3; ++v) d[v] = tessellated_tri[(v + 1) % 3] - tessellated_tri[(v + 2) % 3];
			for (unsigned int v = 0; v < 3; ++v) d_norm[v] = d[v].norm();
			for (unsigned int v = 0; v < 3; ++v) C[v] = std::log((R[v] + d_norm[v]) / (R[v] - d_norm[v])) / (4.0 * M_PI * d_norm[v]);

			Eigen::Vector3d Pt(-signed_solid_angle * Nt);
			for (unsigned int v = 0; v < 3; ++v) Pt += Nt.cross(C[v] * d[v]);
			for (unsigned int v = 0; v < 3; ++v) J[v] = e[(v + 2) % 3].cross(e[(v + 1) % 3]);

			psi_tri = -3.0 * signed_solid_angle * signed_volume / At;
			for (unsigned int v = 0; v < 3; ++v) psi_tri -= C[v] * J[v].dot(Nt);

			Eigen::VectorXd vec_u = bezier_quad_sheet_u_n(u_avg, v_avg, dim, dim);
			Eigen::VectorXd vec_v = bezier_quad_sheet_v_n(u_avg, v_avg, dim, dim);
			Eigen::VectorXd merged = mergeVectors_quad(vec_u, vec_v, dim);
			auto const b_avg = merged;
			auto const phi_avg = bezier_quad_sheet_n(u_avg, v_avg, dim, dim);
			auto const phi_bezier = phi_avg * omega_tri / (4.f * M_PI);
			auto const interpolated_u = bezier_quad_u_tangent_n(control_points, u_avg, v_avg, dim, dim);
			auto const interpolated_v = bezier_quad_v_tangent_n(control_points, u_avg, v_avg, dim, dim);
			auto const interpolated_normal = interpolated_u.cross(interpolated_v);
			auto const psi_bezier = psi_tri * b_avg / interpolated_normal.norm();

			for (unsigned int k = 0; k < numPointBezierQuad; ++k)
			{
				auto const phi_k = phi_bezier(k);
				Phi(k) += phi_k;
			}
			for (unsigned int k = 0; k < numCrossBezierQuad; ++k)
			{
				Phi(k + numPointBezierQuad) += psi_bezier(k);
			}
		};

	for (int v_it = 0; v_it <= n; ++v_it)
	{
		for (int u_it = v_it; u_it <= (2 * n - v_it); ++u_it)
		{
			if ((u_it + v_it) % 2 == 0)
			{
				Proc({ u[u_it], u[u_it + 2], u[u_it + 1] }, { v[v_it], v[v_it], v[v_it + 1] });
				Proc({ u[u_it], u[u_it + 1], u[u_it + 2] }, { v[N - v_it], v[N - v_it - 1], v[N - v_it] });
				Proc({ u[v_it], u[v_it + 1], u[v_it] }, { v[u_it], v[u_it + 1], v[u_it + 2] });
				Proc({ u[N - v_it], u[N - v_it], u[N - v_it - 1] }, { v[u_it], v[u_it + 2], v[u_it + 1] });
			}
			else
			{
				Proc({ u[u_it], u[u_it + 1], u[u_it + 2] }, { v[v_it + 1], v[v_it], v[v_it + 1] });
				Proc({ u[u_it], u[u_it + 2], u[u_it + 1] }, { v[N - v_it - 1], v[N - v_it - 1], v[N - v_it] });
				Proc({ u[v_it + 1], u[v_it + 1], u[v_it] }, { v[u_it], v[u_it + 2], v[u_it + 1] });
				Proc({ u[N - v_it - 1], u[N - v_it], u[N - v_it - 1] }, { v[u_it], v[u_it + 1], v[u_it + 2] });
			}
		}
	}
	return Phi;
}


void calculateGreenCoordinatesBezier(const Eigen::MatrixXd& C, const Eigen::MatrixXi& CF, int dim, std::vector<Eigen::MatrixXd>& patch_Bezier_normals, Eigen::MatrixXd const& eta_m,
	Eigen::MatrixXd& phi_bezier, std::vector<Eigen::MatrixXd>& psi_bezier, std::vector<int>& num_vertices_per_line)
{
	phi_bezier.resize(C.rows(), eta_m.rows());
	phi_bezier.fill(0);
	int numPointBezierTriangle = (dim + 1) * (dim + 2) / 2;
	int numPointBezierQuad = (dim + 1) * (dim + 1);
	for (int eta_idx = 0; eta_idx < eta_m.rows(); ++eta_idx)
	{
		const Eigen::Vector3d eta = eta_m.row(eta_idx);
		//phi and psi -> 2*
		Eigen::VectorXd y(num_vertices_per_line.size() * 2 * numPointBezierQuad);
		Eigen::MatrixXd A(4, num_vertices_per_line.size() * 2 * numPointBezierQuad);

		int y_index = 0;
		int A_index = 0;

		bool have_bezier_triangle = false;
		for (int face_idx = 0; face_idx < CF.rows(); ++face_idx)
		{
			auto const face = CF.row(face_idx);
			if (num_vertices_per_line[face_idx] == numPointBezierTriangle)
			{
				have_bezier_triangle = true;
				std::vector<Eigen::Vector3d> face_points(numPointBezierTriangle);
				std::vector<Eigen::Vector3d> vertex_normals(numPointBezierTriangle);

				for (int k = 0; k < numPointBezierTriangle; ++k) {
					auto const v_idx = face(k);
					face_points[k] = C.row(v_idx);
					vertex_normals[k] = patch_Bezier_normals[face_idx].row(k);
				}

				auto const Phi = computePhiAndPsiForOneBezierTriangle(eta, face_points, vertex_normals, dim);
				assert(Phi.size() == 2 * numPointBezierTriangle);


				for (int k = 0; k < numPointBezierTriangle; ++k)
				{
					y(y_index + k) = Phi(k);
					Eigen::Vector4d four_dimensional_vector;
					four_dimensional_vector.head(3) = face_points[k];
					four_dimensional_vector(3) = 1;
					A.col(A_index + k) = four_dimensional_vector;
				}


				for (int k = 0; k < numPointBezierTriangle; ++k)
				{
					y(y_index + numPointBezierTriangle + k) = Phi(numPointBezierTriangle + k);
					Eigen::Vector4d four_dimensional_vector;
					four_dimensional_vector.head(3) = vertex_normals[k];
					four_dimensional_vector(3) = 0;
					A.col(A_index + numPointBezierTriangle + k) = four_dimensional_vector;
				}

				y_index += 2 * numPointBezierTriangle;
				A_index += 2 * numPointBezierTriangle;
			}
			else
			{

				std::vector<Eigen::Vector3d> face_points(numPointBezierQuad);
				std::vector<Eigen::Vector3d> vertex_normals(numPointBezierQuad);

				for (int k = 0; k < numPointBezierQuad; ++k)
				{
					auto const v_idx = face(k);
					face_points[k] = C.row(v_idx);
					vertex_normals[k] = patch_Bezier_normals[face_idx].row(k);
				}

				auto const Phi = computePhiAndPsiForOneBezierSurface(eta, face_points, vertex_normals, dim);
				assert(Phi.size() == 2 * numPointBezierQuad);


				for (int k = 0; k < numPointBezierQuad; ++k)
				{
					y(y_index + k) = Phi(k);
					Eigen::Vector4d four_dimensional_vector;
					four_dimensional_vector.head(3) = face_points[k];
					four_dimensional_vector(3) = 1;
					A.col(A_index + k) = four_dimensional_vector;
				}


				for (int k = 0; k < numPointBezierQuad; ++k)
				{
					y(y_index + numPointBezierQuad + k) = Phi(numPointBezierQuad + k);
					Eigen::Vector4d four_dimensional_vector;
					four_dimensional_vector.head(3) = vertex_normals[k];
					four_dimensional_vector(3) = 0;
					A.col(A_index + numPointBezierQuad + k) = four_dimensional_vector;
				}
				y_index += 2 * numPointBezierQuad;
				A_index += 2 * numPointBezierQuad;
			}
		}

		int new_A_cols = A_index;
		if (have_bezier_triangle)
		{
			Eigen::MatrixXd new_A = A.leftCols(new_A_cols);
			Eigen::VectorXd new_y = y.head(new_A_cols);
			A = new_A;
			y = new_y;
		}

		Eigen::Vector4d eta_1;
		eta_1.head(3) = eta;
		eta_1(3) = 1;
		Eigen::VectorXd Ay = A * y;

		Eigen::MatrixXd AT = A.transpose();
		Eigen::MatrixXd AAT = A * AT;
		Eigen::JacobiSVD<Eigen::MatrixXd> svd(AAT, Eigen::ComputeThinU | Eigen::ComputeThinV);

		Eigen::VectorXd AAT_inv_eta_minus_Ay = svd.solve(eta_1 - Ay);
		Eigen::VectorXd y_prime = y + AT * AAT_inv_eta_minus_Ay;
		y_index = 0;
		A_index = 0;
		for (int face_idx = 0; face_idx < CF.rows(); ++face_idx)
		{
			auto const face = CF.row(face_idx);
			if (num_vertices_per_line[face_idx] == numPointBezierTriangle)
			{
				for (int k = 0; k < numPointBezierTriangle; ++k)
				{
					phi_bezier(face(k), eta_idx) += y_prime(y_index + k);
				}
				Eigen::VectorXd vector_b_tri(numPointBezierTriangle);
				for (int i = 0; i < numPointBezierTriangle; ++i)
				{
					vector_b_tri(i) = y_prime(y_index + numPointBezierTriangle + i);
				}
				psi_bezier.push_back(vector_b_tri);
				y_index += 2 * numPointBezierTriangle;
			}
			else
			{
				for (int k = 0; k < numPointBezierQuad; ++k)
				{
					phi_bezier(face(k), eta_idx) += y_prime(y_index + k);
				}

				Eigen::VectorXd vector_b_quad(numPointBezierQuad);
				for (int i = 0; i < numPointBezierQuad; ++i)
				{
					vector_b_quad(i) = y_prime(y_index + numPointBezierQuad + i);
				}
				psi_bezier.push_back(vector_b_quad);
				y_index += 2 * numPointBezierQuad;
			}
		}
	}
}


void calculateGreenCoordinatesBezierCrossProduct(const Eigen::MatrixXd& C, const Eigen::MatrixXi& CF, int dim, std::vector<Eigen::MatrixXd>& patch_point_cross, Eigen::MatrixXd const& eta_m,
	Eigen::MatrixXd& phi_bezier, std::vector<Eigen::MatrixXd>& psi_bezier, std::vector<int>& num_vertices_per_line)
{
	phi_bezier.resize(C.rows(), eta_m.rows());
	phi_bezier.fill(0);
	int numPointBezierTriangle = (dim + 1) * (dim + 2) / 2;
	int numPointBezierQuad = (dim + 1) * (dim + 1);
	int numCrossBezierTriangle = numPointBezierTriangle * (numPointBezierTriangle - 1) / 2;
	int numCrossBezierQuad = numPointBezierQuad * (numPointBezierQuad - 1) / 2;
	//eta is the vertex of triangle.
	bool have_bezier_triangle = false;
	for (int eta_idx = 0; eta_idx < eta_m.rows(); ++eta_idx)
	{
		Eigen::VectorXd y(num_vertices_per_line.size() * (numPointBezierQuad + numCrossBezierQuad));
		Eigen::MatrixXd A(4, num_vertices_per_line.size() * (numPointBezierQuad + numCrossBezierQuad));

		int y_index = 0;
		int A_index = 0;
		const Eigen::Vector3d eta = eta_m.row(eta_idx);

		for (int face_idx = 0; face_idx < CF.rows(); ++face_idx)
		{
			auto const face = CF.row(face_idx);
			if (num_vertices_per_line[face_idx] == numPointBezierTriangle)
			{
				have_bezier_triangle = true;
				std::vector<Eigen::Vector3d> face_points(numPointBezierTriangle);
				Eigen::MatrixXd point_cross = patch_point_cross[face_idx];
				for (int k = 0; k < numPointBezierTriangle; ++k)
				{
					auto const v_idx = face(k);
					face_points[k] = C.row(v_idx);
				}

				auto const Phi = computePhiAndPsiForOneBezierTriangleCrossProduct(eta, face_points, dim);
				assert(Phi.size() == numPointBezierTriangle + numCrossBezierTriangle);

				for (int k = 0; k < numPointBezierTriangle; ++k)
				{
					y(y_index + k) = Phi(k);
					Eigen::Vector4d four_dimensional_vector;
					four_dimensional_vector.head(3) = face_points[k];
					four_dimensional_vector(3) = 1;
					A.col(A_index + k) = four_dimensional_vector;
				}

				for (int k = 0; k < numCrossBezierTriangle; ++k)
				{
					y(y_index + numPointBezierTriangle + k) = Phi(numPointBezierTriangle + k);
					Eigen::Vector4d four_dimensional_vector;
					four_dimensional_vector.head(3) = point_cross.row(k);
					four_dimensional_vector(3) = 0;
					A.col(A_index + numPointBezierTriangle + k) = four_dimensional_vector;
				}

				y_index += (numPointBezierTriangle + numCrossBezierTriangle);
				A_index += (numPointBezierTriangle + numCrossBezierTriangle);
			}
			else
			{
				std::vector<Eigen::Vector3d> face_points(numPointBezierQuad);
				Eigen::MatrixXd point_cross = patch_point_cross[face_idx];
				for (int k = 0; k < num_vertices_per_line[face_idx]; ++k)
				{
					auto const v_idx = face(k);
					face_points[k] = C.row(v_idx);
				}

				auto const Phi = computePhiAndPsiForOneBezierSurfaceCrossProduct(eta, face_points, dim);
				assert(Phi.size() == numPointBezierQuad + numCrossBezierQuad);

				for (int k = 0; k < numPointBezierQuad; ++k)
				{
					y(y_index + k) = Phi(k);
					Eigen::Vector4d four_dimensional_vector;
					four_dimensional_vector.head(3) = face_points[k];
					four_dimensional_vector(3) = 1;
					A.col(A_index + k) = four_dimensional_vector;
				}
				for (int k = 0; k < numCrossBezierQuad; ++k)
				{
					y(y_index + numPointBezierQuad + k) = Phi(numPointBezierQuad + k);
					Eigen::Vector4d four_dimensional_vector;
					four_dimensional_vector.head(3) = point_cross.row(k);
					four_dimensional_vector(3) = 0;
					A.col(A_index + numPointBezierQuad + k) = four_dimensional_vector;
				}
				y_index += (numPointBezierQuad + numCrossBezierQuad);
				A_index += (numPointBezierQuad + numCrossBezierQuad);
			}
		}

		int new_A_cols = A_index;
		if (have_bezier_triangle)
		{
			Eigen::MatrixXd new_A = A.leftCols(new_A_cols);
			Eigen::VectorXd new_y = y.head(new_A_cols);
			A = new_A;
			y = new_y;
		}

		Eigen::VectorXd Ay = A * y;
		Eigen::MatrixXd AT = A.transpose();
		Eigen::MatrixXd AAT = A * AT;
		Eigen::JacobiSVD<Eigen::MatrixXd> svd(AAT, Eigen::ComputeThinU | Eigen::ComputeThinV);
		Eigen::Vector4d eta_1;
		eta_1.head(3) = eta;
		eta_1(3) = 1;

		Eigen::VectorXd AAT_inv_eta_minus_Ay = svd.solve(eta_1 - Ay);
		Eigen::VectorXd y_prime = y + AT * AAT_inv_eta_minus_Ay;

		y_index = 0;
		A_index = 0;
		for (int face_idx = 0; face_idx < CF.rows(); ++face_idx)
		{
			auto const face = CF.row(face_idx);
			if (num_vertices_per_line[face_idx] == numPointBezierTriangle)
			{
				for (int k = 0; k < numPointBezierTriangle; ++k)
				{
					phi_bezier(face(k), eta_idx) += y_prime(y_index + k);
				}
				Eigen::VectorXd vector_b_tri(numCrossBezierTriangle);
				for (int i = 0; i < numCrossBezierTriangle; ++i)
				{
					vector_b_tri(i) = y_prime(y_index + numPointBezierTriangle + i);
				}
				psi_bezier.push_back(vector_b_tri);
				y_index += (numPointBezierTriangle + numCrossBezierTriangle);
			}
			else
			{
				for (int k = 0; k < numPointBezierQuad; ++k)
				{
					phi_bezier(face(k), eta_idx) += y_prime(y_index + k);
				}

				Eigen::VectorXd vector_b_quad(numCrossBezierQuad);
				for (int i = 0; i < numCrossBezierQuad; ++i)
				{
					vector_b_quad(i) = y_prime(y_index + numPointBezierQuad + i);
				}
				psi_bezier.push_back(vector_b_quad);
				y_index += (numPointBezierQuad + numCrossBezierQuad);
			}
		}
	}
}
float_t sigma_L_bezier_triangle_n(std::vector<Eigen::Vector3d>& old_bezier_triangle_vertices, std::vector<Eigen::Vector3d>& new_bezier_triangle_vertices, double u, double v, int dim)
{
	auto const old_u_tangent = bezier_triangle_u_tangent_n(old_bezier_triangle_vertices, u, v, dim);
	auto const old_v_tangent = bezier_triangle_v_tangent_n(old_bezier_triangle_vertices, u, v, dim);
	auto const new_u_tangent = bezier_triangle_u_tangent_n(new_bezier_triangle_vertices, u, v, dim);
	auto const new_v_tangent = bezier_triangle_v_tangent_n(new_bezier_triangle_vertices, u, v, dim);

	return std::sqrt((new_u_tangent.squaredNorm() * old_v_tangent.squaredNorm() + old_u_tangent.squaredNorm() * new_v_tangent.squaredNorm() - 2. *
		new_u_tangent.dot(new_v_tangent) * old_u_tangent.dot(old_v_tangent)) / (2. * (old_u_tangent.cross(old_v_tangent)).squaredNorm()));
}



float_t sigma_L_bezier_quad_n(std::vector<Eigen::Vector3d>& old_bezier_quad_vertices, std::vector<Eigen::Vector3d>& new_bezier_quad_vertices, double u, double v, int m, int n)
{
	auto const old_u_tangent = bezier_quad_u_tangent_n(old_bezier_quad_vertices, u, v, m, n);
	auto const old_v_tangent = bezier_quad_v_tangent_n(old_bezier_quad_vertices, u, v, m, n);
	auto const new_u_tangent = bezier_quad_u_tangent_n(new_bezier_quad_vertices, u, v, m, n);
	auto const new_v_tangent = bezier_quad_v_tangent_n(new_bezier_quad_vertices, u, v, m, n);

	return std::sqrt((new_u_tangent.squaredNorm() * old_v_tangent.squaredNorm() + old_u_tangent.squaredNorm() * new_v_tangent.squaredNorm() - 2. *
		new_u_tangent.dot(new_v_tangent) * old_u_tangent.dot(old_v_tangent)) / (2. * (old_u_tangent.cross(old_v_tangent)).squaredNorm()));
}

float_t sigma_a_bezier_triangle_n(std::vector<Eigen::Vector3d>& old_bezier_triangle_vertices, std::vector<Eigen::Vector3d>& new_bezier_triangle_vertices, double u, double v, int dim)
{
	auto const old_normal = bezier_triangle_interpolate_n(old_bezier_triangle_vertices, u, v, dim);
	auto const new_normal = bezier_triangle_interpolate_n(new_bezier_triangle_vertices, u, v, dim);
	return new_normal.norm() / old_normal.norm();
}

float_t sigma_a_bezier_quad_n(std::vector<Eigen::Vector3d>& old_bezier_quad_vertices, std::vector<Eigen::Vector3d>& new_bezier_quad_vertices, double u, double v, int m, int n)
{
	auto const old_normal = bezier_quad_interpolate_n(old_bezier_quad_vertices, u, v, m, n);
	auto const new_normal = bezier_quad_interpolate_n(new_bezier_quad_vertices, u, v, m, n);
	return new_normal.norm() / old_normal.norm();
}

std::vector < double > numerically_approx_sigma_q_bezier_triangle(std::vector<Eigen::Vector3d>& old_bezier_triangle_vertices, std::vector<Eigen::Vector3d>& new_bezier_triangle_vertices, std::vector<Eigen::Vector3d>& old_bezier_triangle_normals, std::vector<Eigen::Vector3d>& new_bezier_triangle_normals, int n = 10, int dim = 3)
{
	auto const numSamples = n * n;
	auto const dx = 1. / static_cast<double>(n);
	int numPointBezierTriangle = (dim + 1) * (dim + 2) / 2;
	std::vector<double> res_sigma_a(numPointBezierTriangle, 0.0);
	std::vector<double> res_sigma_L(numPointBezierTriangle, 0.0);
	for (int i = 1; i <= n; ++i)
	{
		auto const u = static_cast<double>(i) * dx - dx * .5;

		for (int j = 1; j <= n; ++j)
		{
			auto const v = static_cast<double>(j) * dx - dx * .5;
			if (u + v > 1.0)
			{
				continue;
			}
			auto const b = bezier_triangle_sheet_n(u, v, dim);
			auto const normal = bezier_triangle_interpolate_n(old_bezier_triangle_normals, u, v, dim);
			auto const sigma_a_approx = sigma_a_bezier_triangle_n(old_bezier_triangle_normals, new_bezier_triangle_normals, u, v, dim);
			auto const sigma_L_approx = sigma_L_bezier_triangle_n(old_bezier_triangle_vertices, new_bezier_triangle_vertices, u, v, dim);
			double norm = normal.stableNorm();
			for (int k = 0; k < numPointBezierTriangle; ++k)
			{
				res_sigma_a[k] += dx * dx * b[k] * sigma_a_approx / norm;
				res_sigma_L[k] += dx * dx * b[k] * sigma_L_approx / norm;
			}
		}
	}
	std::vector<double> sigma(numPointBezierTriangle, 0.0);
	for (int i = 0; i < numPointBezierTriangle; ++i)
	{
		sigma[i] = (res_sigma_a[i] != 0) ? res_sigma_L[i] / res_sigma_a[i] : 0.0;
	}

	return sigma;
}



std::vector < double > numerically_approx_sigma_q_bezier_triangle_cross_product(std::vector<Eigen::Vector3d>& old_bezier_triangle_vertices, std::vector<Eigen::Vector3d>& new_bezier_triangle_vertices, int n = 10, int dim = 3)
{
	int numPointBezierTriangle = (dim + 1) * (dim + 2) / 2;
	int numCrossBezierTriangle = numPointBezierTriangle * (numPointBezierTriangle - 1) / 2;
	auto const numSamples = n * n;
	auto const dx = 1. / static_cast<double>(n);
	std::vector<double> res_sigma_a(numCrossBezierTriangle, 0.0);
	std::vector<double> res_sigma_L(numCrossBezierTriangle, 0.0);
	for (int i = 1; i <= n; ++i)
	{
		auto const u = static_cast<double>(i) * dx - dx * .5;

		for (int j = 1; j <= n; ++j)
		{
			auto const v = static_cast<double>(j) * dx - dx * .5;
			if (u + v > 1.0)
			{
				continue;
			}
			auto const old_normal = bezier_triangle_u_tangent_n(old_bezier_triangle_vertices, u, v, dim).cross(bezier_triangle_v_tangent_n(old_bezier_triangle_vertices, u, v, dim));
			auto const new_normal = bezier_triangle_u_tangent_n(new_bezier_triangle_vertices, u, v, dim).cross(bezier_triangle_v_tangent_n(new_bezier_triangle_vertices, u, v, dim));
			auto const sigma_a_approx = new_normal.norm() / old_normal.norm();
			auto const sigma_L_approx = sigma_L_bezier_triangle_n(old_bezier_triangle_vertices, new_bezier_triangle_vertices, u, v, dim);
			double norm = old_normal.stableNorm();
			for (int k = 0; k < numCrossBezierTriangle; ++k)
			{
				res_sigma_a[k] += dx * dx * sigma_a_approx / norm;
				res_sigma_L[k] += dx * dx * sigma_L_approx / norm;
			}
		}
	}
	std::vector<double> sigma(numCrossBezierTriangle, 0.0);
	for (int i = 0; i < numCrossBezierTriangle; ++i)
	{
		sigma[i] = (res_sigma_a[i] != 0) ? res_sigma_L[i] / res_sigma_a[i] : 0.0;
	}

	return sigma;
}



std::vector < double > numerically_approx_sigma_q_bezier_quad(std::vector<Eigen::Vector3d>& old_bezier_quad_vertices, std::vector<Eigen::Vector3d>& new_bezier_quad_vertices, std::vector<Eigen::Vector3d>& old_bezier_quad_normals, std::vector<Eigen::Vector3d>& new_bezier_quad_normals, int n = 10, int dim = 3)
{
	auto const numSamples = n * n;
	auto const dx = 1. / static_cast<double>(n);
	int numPointBezierQuad = (dim + 1) * (dim + 1);
	std::vector < double > res_sigma_a(numPointBezierQuad, 0);
	std::vector < double > res_sigma_L(numPointBezierQuad, 0);
	for (int i = 1; i <= n; ++i)
	{
		auto const u = static_cast<double>(i) * dx - dx * .5;

		for (int j = 1; j <= n; ++j)
		{
			auto const v = static_cast<double>(j) * dx - dx * .5;

			auto const b = bezier_quad_sheet_n(u, v, dim, dim);
			auto const normal = bezier_quad_interpolate_n(old_bezier_quad_normals, u, v, dim, dim);
			auto const sigma_a_approx = sigma_a_bezier_quad_n(old_bezier_quad_normals, new_bezier_quad_normals, u, v, dim, dim);
			auto const sigma_L_approx = sigma_L_bezier_quad_n(old_bezier_quad_vertices, new_bezier_quad_vertices, u, v, dim, dim);
			double norm = normal.stableNorm();
			for (int k = 0; k < numPointBezierQuad; ++k)
			{
				res_sigma_a[k] += dx * dx * b[k] * sigma_a_approx / norm;
				res_sigma_L[k] += dx * dx * b[k] * sigma_L_approx / norm;
			}
		}
	}
	std::vector < double > sigma(numPointBezierQuad, 0);
	for (int i = 0; i < numPointBezierQuad; ++i)
	{
		sigma[i] = (res_sigma_a[i] != 0) ? res_sigma_L[i] / res_sigma_a[i] : 0.0;
	}

	return sigma;
}

std::vector < double > numerically_approx_sigma_q_bezier_quad_cross_product(std::vector<Eigen::Vector3d>& old_bezier_quad_vertices, std::vector<Eigen::Vector3d>& new_bezier_quad_vertices, int n = 10, int dim = 3)
{
	auto const numSamples = n * n;
	auto const dx = 1. / static_cast<double>(n);
	int numPointBezierQuad = (dim + 1) * (dim + 1);
	int numCrossBezierQuad = numPointBezierQuad * (numPointBezierQuad - 1) / 2;
	std::vector < double > res_sigma_a(numCrossBezierQuad, 0);
	std::vector < double > res_sigma_L(numCrossBezierQuad, 0);
	for (int i = 1; i <= n; ++i)
	{
		auto const u = static_cast<double>(i) * dx - dx * .5;

		for (int j = 1; j <= n; ++j)
		{
			auto const v = static_cast<double>(j) * dx - dx * .5;
			auto const old_normal = bezier_quad_u_tangent_n(old_bezier_quad_vertices, u, v, dim, dim).cross(bezier_quad_v_tangent_n(old_bezier_quad_vertices, u, v, dim, dim));
			auto const new_normal = bezier_quad_u_tangent_n(new_bezier_quad_vertices, u, v, dim, dim).cross(bezier_quad_v_tangent_n(new_bezier_quad_vertices, u, v, dim, dim));
			auto const sigma_a_approx = new_normal.norm() / old_normal.norm();
			auto const sigma_L_approx = sigma_L_bezier_quad_n(old_bezier_quad_vertices, new_bezier_quad_vertices, u, v, dim, dim);
			double norm = old_normal.stableNorm();
			for (int k = 0; k < numCrossBezierQuad; ++k)
			{
				res_sigma_a[k] += dx * dx * sigma_a_approx / norm;
				res_sigma_L[k] += dx * dx * sigma_L_approx / norm;
			}
		}
	}
	std::vector < double > sigma(numCrossBezierQuad, 0);
	for (int i = 0; i < numCrossBezierQuad; ++i)
	{
		sigma[i] = (res_sigma_a[i] != 0) ? res_sigma_L[i] / res_sigma_a[i] : 0.0;
	}

	return sigma;
}



void calcNewPositionsBezier(const Eigen::MatrixXd& C, const Eigen::MatrixXd& C_deformed, const Eigen::MatrixXi& CF, int dim, std::vector<Eigen::MatrixXd>& patch_Bezier_normals_original, std::vector<Eigen::MatrixXd>& patch_Bezier_normals,
	const Eigen::MatrixXd& phi, std::vector<Eigen::MatrixXd> const& psi_bezier, Eigen::MatrixXd& eta_deformed, std::vector<int>& num_vertices_per_line)
{
	
	eta_deformed = phi.transpose() * C_deformed;

	
	int numPointBezierTriangle = (dim + 1) * (dim + 2) / 2;
	int numPointBezierQuad = (dim + 1) * (dim + 1);
	std::vector < std::vector < double >> sigma_bezier;
	for (unsigned int face_idx = 0; face_idx < CF.rows(); ++face_idx)
	{
		auto const face = CF.row(face_idx);
		if (num_vertices_per_line[face_idx] == numPointBezierTriangle)
		{
			std::vector<Eigen::Vector3d> new_face_points(numPointBezierTriangle), old_face_points(numPointBezierTriangle), old_face_normals(numPointBezierTriangle), new_face_normals(numPointBezierTriangle);

			for (int k = 0; k < numPointBezierTriangle; ++k)
			{
				old_face_points[k] = C.row(face(k));
				new_face_points[k] = C_deformed.row(face(k));
				old_face_normals[k] = patch_Bezier_normals_original[face_idx].row(k);
				new_face_normals[k] = patch_Bezier_normals[face_idx].row(k);
			}
			auto const sigma_q = numerically_approx_sigma_q_bezier_triangle(old_face_points, new_face_points, old_face_normals, new_face_normals, 10, dim);
			sigma_bezier.push_back((std::vector < double >)sigma_q);
		}
		else
		{
			std::vector<Eigen::Vector3d> new_face_points(numPointBezierQuad), old_face_points(numPointBezierQuad), old_face_normals(numPointBezierQuad), new_face_normals(numPointBezierQuad);

			for (int k = 0; k < numPointBezierQuad; ++k)
			{
				old_face_points[k] = C.row(face(k));
				new_face_points[k] = C_deformed.row(face(k));
				old_face_normals[k] = patch_Bezier_normals_original[face_idx].row(k);
				new_face_normals[k] = patch_Bezier_normals[face_idx].row(k);
			}
			auto const sigma_q = numerically_approx_sigma_q_bezier_quad(old_face_points, new_face_points, old_face_normals, new_face_normals, 10, dim);
			sigma_bezier.push_back((std::vector < double >)sigma_q);
		}
	}

	int psi_count = 0;
	for (unsigned int eta_idx = 0; eta_idx < eta_deformed.rows(); ++eta_idx)
	{
		Eigen::Vector3d eta = eta_deformed.row(eta_idx);
		for (unsigned int face_idx = 0; face_idx < CF.rows(); ++face_idx)
		{
			auto const face = CF.row(face_idx);
			if (num_vertices_per_line[face_idx] == numPointBezierTriangle)
			{
				std::vector<Eigen::Vector3d> new_face_points(numPointBezierTriangle), old_face_points(numPointBezierTriangle), new_face_normals(numPointBezierTriangle);

				for (int k = 0; k < numPointBezierTriangle; ++k)
				{
					old_face_points[k] = C.row(face(k));
					new_face_points[k] = C_deformed.row(face(k));
					new_face_normals[k] = patch_Bezier_normals[face_idx].row(k);
				}

				Eigen::MatrixXd psi = psi_bezier[psi_count];
				for (int k = 0; k < numPointBezierTriangle; ++k)
				{
					eta += psi(k) * new_face_normals[k] * sigma_bezier[face_idx][k];
				}
				psi_count++;
			}
			else
			{
				std::vector<Eigen::Vector3d> new_face_points(numPointBezierQuad), old_face_points(numPointBezierQuad), new_face_normals(numPointBezierQuad);

				for (int k = 0; k < numPointBezierQuad; ++k)
				{
					old_face_points[k] = C.row(face(k));
					new_face_points[k] = C_deformed.row(face(k));
					new_face_normals[k] = patch_Bezier_normals[face_idx].row(k);
				}

				Eigen::MatrixXd psi = psi_bezier[psi_count];
				for (int k = 0; k < numPointBezierQuad; ++k)
				{
					eta += psi(k) * new_face_normals[k] * sigma_bezier[face_idx][k];
				}
				psi_count++;
			}
		}
		eta_deformed.row(eta_idx) = eta;
	}

}


void calcNewPositionsBezierCrossProduct(const Eigen::MatrixXd& C, const Eigen::MatrixXd& C_deformed, const Eigen::MatrixXi& CF, int dim, std::vector<Eigen::MatrixXd>& patch_Bezier_cross_original, std::vector<Eigen::MatrixXd>& patch_Bezier_cross,
	const Eigen::MatrixXd& phi, std::vector<Eigen::MatrixXd> const& psi_bezier, Eigen::MatrixXd& eta_deformed, std::vector<int>& num_vertices_per_line)
{
	
	eta_deformed = phi.transpose() * C_deformed;

	
	int numPointBezierTriangle = (dim + 1) * (dim + 2) / 2;
	int numPointBezierQuad = (dim + 1) * (dim + 1);
	int numCrossBezierTriangle = numPointBezierTriangle * (numPointBezierTriangle - 1) / 2;
	int numCrossBezierQuad = numPointBezierQuad * (numPointBezierQuad - 1) / 2;
	std::vector < std::vector < double >> sigma_bezier;

	for (unsigned int face_idx = 0; face_idx < CF.rows(); ++face_idx)
	{
		auto const face = CF.row(face_idx);
		if (num_vertices_per_line[face_idx] == numPointBezierTriangle)
		{
			std::vector<Eigen::Vector3d> new_face_points(numPointBezierTriangle), old_face_points(numPointBezierTriangle);

			for (int k = 0; k < num_vertices_per_line[face_idx]; ++k)
			{
				old_face_points[k] = C.row(face(k));
				new_face_points[k] = C_deformed.row(face(k));
			}
			auto const sigma_q = numerically_approx_sigma_q_bezier_triangle_cross_product(old_face_points, new_face_points, 10, dim);
			sigma_bezier.push_back((std::vector < double >)sigma_q);
		}
		else
		{
			std::vector<Eigen::Vector3d> new_face_points(numPointBezierQuad), old_face_points(numPointBezierQuad);

			for (int k = 0; k < num_vertices_per_line[face_idx]; ++k)
			{
				old_face_points[k] = C.row(face(k));
				new_face_points[k] = C_deformed.row(face(k));
			}
			auto const sigma_q = numerically_approx_sigma_q_bezier_quad_cross_product(old_face_points, new_face_points, 10, dim);
			sigma_bezier.push_back((std::vector < double >)sigma_q);
		}
	}

	int psi_count = 0;
	for (unsigned int eta_idx = 0; eta_idx < eta_deformed.rows(); ++eta_idx)
	{
		Eigen::Vector3d eta = eta_deformed.row(eta_idx);
		for (unsigned int face_idx = 0; face_idx < CF.rows(); ++face_idx)
		{
			auto const face = CF.row(face_idx);
			if (num_vertices_per_line[face_idx] == numPointBezierTriangle)
			{
				std::vector<Eigen::Vector3d> new_face_points(numPointBezierTriangle), old_face_points(numPointBezierTriangle), new_cross(numCrossBezierTriangle);

				for (int k = 0; k < numPointBezierTriangle; ++k)
				{
					old_face_points[k] = C.row(face(k));
					new_face_points[k] = C_deformed.row(face(k));
				}
				for (int k = 0; k < numCrossBezierTriangle; ++k)
				{
					new_cross[k] = patch_Bezier_cross[face_idx].row(k);
				}

				Eigen::MatrixXd psi = psi_bezier[psi_count];
				for (int k = 0; k < numCrossBezierTriangle; ++k)
				{
					eta += psi(k) * new_cross[k] * sigma_bezier[face_idx][k];
					
				}
				psi_count++;
			}
			else
			{
				std::vector<Eigen::Vector3d> new_face_points(numPointBezierQuad), old_face_points(numPointBezierQuad), new_cross(numCrossBezierQuad);

				for (int k = 0; k < numPointBezierQuad; ++k)
				{
					old_face_points[k] = C.row(face(k));
					new_face_points[k] = C_deformed.row(face(k));
				}

				for (int k = 0; k < numCrossBezierQuad; ++k)
				{
					new_cross[k] = patch_Bezier_cross[face_idx].row(k);
				}
				Eigen::MatrixXd psi = psi_bezier[psi_count];
				for (int k = 0; k < numCrossBezierQuad; ++k)
				{
					eta += psi(k) * new_cross[k] * sigma_bezier[face_idx][k];
				}
				psi_count++;
			}
		}
		eta_deformed.row(eta_idx) = eta;
	}
}
//BGC related code end